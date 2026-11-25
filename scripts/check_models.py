#!/usr/bin/env python3
"""Check the GPU node's model weights before a perception run.

The repository does not include weights; this reads the camera model paths
from a GPU config and reports which files are missing. With ``--load`` each
present model is loaded with Ultralytics and its task/classes are printed.

Exit codes: 0 all present (and loadable with --load), 1 missing/unloadable.
"""

import argparse
import pathlib
import sys

import yaml


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        '--config',
        default='ros2_ws/src/jcheng_perception_gpu/config/gpu.yaml',
        help='GPU config to inspect')
    parser.add_argument(
        '--load', action='store_true',
        help='load each present model with Ultralytics')
    args = parser.parse_args()

    path = pathlib.Path(args.config)
    if not path.is_file():
        print('FAIL: config not found: {}'.format(path), file=sys.stderr)
        return 2
    config = yaml.safe_load(path.read_text(encoding='utf-8')) or {}
    cameras = config.get('cameras') or {}
    missing = 0
    checked = 0

    for camera_id, spec in cameras.items():
        model = (spec or {}).get('model') or {}
        backend = str(model.get('backend', '?'))
        weights = model.get('path')
        if backend != 'ultralytics' or not weights:
            print('{}: backend={} (no weight file)'.format(camera_id, backend))
            continue
        checked += 1
        weight_path = pathlib.Path(weights)
        if not weight_path.is_file():
            missing += 1
            print('{}: MISSING {} (task={})'.format(
                camera_id, weights, model.get('task', '?')))
            continue
        print('{}: OK {} ({:.1f} MB)'.format(
            camera_id, weights, weight_path.stat().st_size / 1e6))
        if not args.load:
            continue
        try:
            from ultralytics import YOLO
        except ImportError:
            print('    --load needs Ultralytics in this interpreter')
            continue
        try:
            loaded = YOLO(str(weight_path))
            names = list((loaded.names or {}).values())
            print('    task={} classes={} {}'.format(
                loaded.task, len(names), names[:8]))
        except Exception as error:  # noqa: BLE001 - report any loader failure
            missing += 1
            print('    LOAD FAILED: {}: {}'.format(
                type(error).__name__, error))

    if checked == 0:
        print('no Ultralytics weight paths in {}'.format(path))
        return 0
    if missing:
        print()
        print('{} of {} weight files missing or unloadable.'.format(
            missing, checked))
        print('The repository does not ship weights; see docs/MODEL_ITERATION.md')
        print('and place the files at the paths above.')
        return 1
    return 0


if __name__ == '__main__':
    sys.exit(main())
