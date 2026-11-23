#!/usr/bin/env python3
"""
Smoke-check one GPU detector backend without ROS.

Examples:
    # no model / no GPU needed
    python3 scripts/check_gpu_backend.py --backend mock

    # real model once Ultralytics is installed on the GPU host
    python3 scripts/check_gpu_backend.py \
      --backend ultralytics \
      --model /opt/jcheng/models/ext_yolov8n.pt \
      --image /data/sample.jpg

The script prints JSON with the model identity, detections and timings. It
exits 2 with a clear message when a runtime or model is missing; it does not try
to install anything.
"""

import argparse
import json
import sys
import time
from pathlib import Path

import cv2
import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[1]
for relative in (
        'ros2_ws/src/jcheng_perception_core',
        'ros2_ws/src/jcheng_perception_gpu'):
    path = str(REPO_ROOT / relative)
    if path not in sys.path:
        sys.path.insert(0, path)

from jcheng_perception_gpu.detectors import (  # noqa: E402
    DetectorError,
    build_detector,
)


def synthetic_image():
    """Return a grey image with a red square for the mock detector."""
    image = np.full((400, 640, 3), 55, dtype=np.uint8)
    cv2.rectangle(image, (280, 170), (360, 230), (0, 0, 235), -1)
    return image


def serialize(detections, limit=10):
    """Convert detections to JSON-friendly data."""
    values = []
    for detection in detections[:limit]:
        values.append({
            'label': detection.label,
            'class_id': int(detection.class_id),
            'score': round(float(detection.score), 4),
            'bbox_xyxy': [round(float(v), 2)
                          for v in detection.bbox_xyxy],
            'keypoints': [
                {
                    'x': round(float(point.x), 2),
                    'y': round(float(point.y), 2),
                    'confidence': round(float(point.confidence), 4),
                }
                for point in detection.keypoints[:10]
            ],
        })
    return values


def main(argv=None):
    """Run one detector smoke check."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--backend', default='mock',
                        choices=['mock', 'ultralytics'])
    parser.add_argument('--model', default='')
    parser.add_argument('--image', default='')
    parser.add_argument('--imgsz', type=int, default=640)
    parser.add_argument('--conf', type=float, default=0.35)
    parser.add_argument('--iou', type=float, default=0.45)
    parser.add_argument('--device', default='0')
    parser.add_argument('--task', default='detect')
    parser.add_argument('--json', action='store_true')
    args = parser.parse_args(argv)

    if args.image:
        image = cv2.imread(args.image, cv2.IMREAD_COLOR)
        if image is None:
            print('cannot read image: {}'.format(args.image),
                  file=sys.stderr)
            return 3
        image_source = args.image
    elif args.backend == 'mock':
        image = synthetic_image()
        image_source = 'synthetic-red-square'
    else:
        print('--image is required for the ultralytics backend',
              file=sys.stderr)
        return 3

    config = {
        'backend': args.backend,
        'path': args.model,
        'task': args.task,
        'imgsz': args.imgsz,
        'conf': args.conf,
        'iou': args.iou,
        'device': args.device,
        'half': False,
        'max_detections': 20,
        'keypoints': args.task == 'pose',
        'labels': [],
        'hot_reload': False,
    }
    started = time.perf_counter()
    try:
        detector = build_detector(config)
    except DetectorError as error:
        print('detector unavailable: {}'.format(error), file=sys.stderr)
        return 2
    load_ms = (time.perf_counter() - started) * 1000.0
    started = time.perf_counter()
    try:
        detections = detector.detect(image)
    except DetectorError as error:
        print('inference failed: {}'.format(error), file=sys.stderr)
        return 2
    inference_ms = (time.perf_counter() - started) * 1000.0
    result = {
        'backend': args.backend,
        'model_id': detector.describe(),
        'image': image_source,
        'image_shape': list(image.shape),
        'load_ms': round(load_ms, 2),
        'inference_ms': round(inference_ms, 2),
        'detection_count': len(detections),
        'detections': serialize(detections),
    }
    if args.json:
        print(json.dumps(result, indent=2, sort_keys=True))
    else:
        print('model   : {}'.format(result['model_id']))
        print('load    : {:.1f} ms'.format(load_ms))
        print('infer   : {:.1f} ms'.format(inference_ms))
        print('detections: {}'.format(len(detections)))
        for detection in result['detections']:
            print('  {} {:.3f} {}'.format(
                detection['label'], detection['score'],
                detection['bbox_xyxy']))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
