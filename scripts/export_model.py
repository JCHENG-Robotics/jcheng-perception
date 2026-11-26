#!/usr/bin/env python3
"""Export a trained .pt model to ONNX or OpenVINO.

The GPU node can load the .pt directly; the export is for the vendor/edge
paths, OpenVINO CPU hosts, or a smaller deployment artifact.

    ~/.venvs/jcheng-perception-gpu/bin/python scripts/export_model.py \
      --weights /opt/jcheng/models/ext_yolov8n.pt --format openvino --imgsz 640

Ultralytics may install the export backend (``onnx`` for ONNX) on first use.
"""

import argparse
import pathlib
import shutil
import sys


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--weights', required=True)
    parser.add_argument(
        '--format', choices=('openvino', 'onnx'), default='openvino')
    parser.add_argument('--imgsz', type=int, default=640)
    parser.add_argument('--half', action='store_true')
    parser.add_argument(
        '--out', default=None,
        help='copy the exported file/directory here')
    args = parser.parse_args()

    weights = pathlib.Path(args.weights).expanduser()
    if not weights.is_file():
        print('FAIL: weights not found: {}'.format(weights), file=sys.stderr)
        return 2
    try:
        from ultralytics import YOLO
    except ImportError as error:
        print('FAIL: Ultralytics not available: {}'.format(error),
              file=sys.stderr)
        return 2

    model = YOLO(str(weights))
    print('exporting {} as {} imgsz={} half={}'.format(
        weights, args.format, args.imgsz, args.half))
    exported = pathlib.Path(
        model.export(
            format=args.format, imgsz=args.imgsz, half=args.half))
    print('exported: {}'.format(exported))
    if not exported.exists():
        print('FAIL: export path does not exist', file=sys.stderr)
        return 1

    if args.out:
        out = pathlib.Path(args.out).expanduser()
        if exported.is_dir():
            if out.exists():
                shutil.rmtree(out)
            shutil.copytree(exported, out)
        else:
            out.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(exported, out)
        print('copied: {}'.format(out))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
