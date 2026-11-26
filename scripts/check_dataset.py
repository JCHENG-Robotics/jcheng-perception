#!/usr/bin/env python3
"""Validate a YOLO detect or pose dataset before training.

Checks image/label pairing, class ids, normalized boxes, and (for pose) the
keypoint count, coordinates and visibility values. Missing labels are reported
as background candidates; orphan labels and malformed lines are errors.

Examples:
    python3 scripts/check_dataset.py --root datasets/ext --task detect \
        --data datasets/ext/data.yaml
    python3 scripts/check_dataset.py --root datasets/wrist_pose --task pose \
        --data datasets/wrist_pose/data.yaml

Exit codes: 0 valid (warnings allowed), 1 errors, 2 bad arguments.
"""

import argparse
import pathlib
import sys

import yaml

IMAGE_EXTENSIONS = {'.jpg', '.jpeg', '.png', '.bmp', '.webp'}


def load_data_yaml(path):
    if not path:
        return None
    data = yaml.safe_load(pathlib.Path(path).read_text(encoding='utf-8')) or {}
    names = data.get('names')
    if isinstance(names, list):
        names = {index: str(name) for index, name in enumerate(names)}
    elif isinstance(names, dict):
        names = {int(key): str(value) for key, value in names.items()}
    else:
        names = None
    kpt_shape = data.get('kpt_shape')
    return {'names': names, 'kpt_shape': kpt_shape}


def label_path(image, images_root, labels_root, flat):
    if flat:
        return image.with_suffix('.txt')
    relative = image.relative_to(images_root)
    return labels_root / relative.with_suffix('.txt')


def validate_line(line, task, names, kpt_shape):
    parts = line.split()
    if task == 'detect':
        if len(parts) != 5:
            return 'expected 5 fields, got {}'.format(len(parts))
    else:
        if kpt_shape is None:
            return 'pose dataset needs kpt_shape in data.yaml'
        expected = 5 + 3 * int(kpt_shape[0])
        if len(parts) != expected:
            return 'expected {} fields, got {}'.format(expected, len(parts))
    try:
        class_id = int(parts[0])
        cx, cy, width, height = (float(value) for value in parts[1:5])
    except ValueError:
        return 'non-numeric class or box field'
    if class_id < 0:
        return 'negative class id'
    if names is not None and class_id not in names:
        return 'class id {} not in data.yaml names'.format(class_id)
    if not (0.0 <= cx <= 1.0 and 0.0 <= cy <= 1.0):
        return 'box centre outside [0,1]'
    if not (0.0 < width <= 1.0 and 0.0 < height <= 1.0):
        return 'box size must be in (0,1]'
    tolerance = 1e-6
    if (cx - width / 2 < -tolerance or cx + width / 2 > 1 + tolerance or
            cy - height / 2 < -tolerance or cy + height / 2 > 1 + tolerance):
        return 'box extends outside the image'
    if task == 'pose':
        count = int(kpt_shape[0])
        visible = 0
        for index in range(count):
            base = 5 + 3 * index
            try:
                x, y, visibility = (float(value) for value in parts[base:base + 3])
            except ValueError:
                return 'non-numeric keypoint {}'.format(index)
            if not (0.0 <= x <= 1.0 and 0.0 <= y <= 1.0):
                return 'keypoint {} outside [0,1]'.format(index)
            if visibility not in (0.0, 1.0, 2.0):
                return 'keypoint {} visibility must be 0, 1 or 2'.format(index)
            if visibility > 0:
                visible += 1
        if visible == 0:
            return 'all keypoints are invisible'
    return None


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', default='datasets/ext')
    parser.add_argument('--task', choices=('detect', 'pose'), default='detect')
    parser.add_argument('--images', default=None, help='default: <root>/images')
    parser.add_argument('--labels', default=None, help='default: <root>/labels')
    parser.add_argument(
        '--flat', action='store_true',
        help='labels sit next to images (raw capture layout)')
    parser.add_argument('--data', default=None, help='data.yaml for names/kpt_shape')
    parser.add_argument('--kpt-shape', type=int, nargs=2, default=None)
    args = parser.parse_args()

    root = pathlib.Path(args.root)
    images_root = pathlib.Path(args.images) if args.images else root / 'images'
    labels_root = pathlib.Path(args.labels) if args.labels else root / 'labels'
    if not images_root.is_dir():
        print('FAIL: images directory not found: {}'.format(images_root),
              file=sys.stderr)
        return 2

    data = load_data_yaml(args.data) if args.data else None
    names = data['names'] if data else None
    kpt_shape = data['kpt_shape'] if data else None
    if args.kpt_shape:
        kpt_shape = list(args.kpt_shape)
    if args.task == 'pose' and kpt_shape is None:
        print('FAIL: pose check needs --data with kpt_shape or --kpt-shape',
              file=sys.stderr)
        return 2

    images = sorted(
        path for path in images_root.rglob('*')
        if path.is_file() and path.suffix.lower() in IMAGE_EXTENSIONS)
    label_files = set()
    missing = 0
    errors = []
    empty = 0
    instances = 0
    class_counts = {}

    for image in images:
        label = label_path(image, images_root, labels_root, args.flat)
        label_files.add(label)
        if not label.is_file():
            missing += 1
            continue
        lines = [
            line.strip() for line in
            label.read_text(encoding='utf-8').splitlines() if line.strip()]
        if not lines:
            empty += 1
            continue
        for number, line in enumerate(lines, 1):
            problem = validate_line(line, args.task, names, kpt_shape)
            if problem:
                errors.append('{}:{}: {}'.format(label, number, problem))
                continue
            instances += 1
            class_id = int(line.split()[0])
            class_counts[class_id] = class_counts.get(class_id, 0) + 1

    orphans = []
    if not args.flat and labels_root.is_dir():
        for label in sorted(labels_root.rglob('*.txt')):
            if label not in label_files:
                orphans.append(str(label))

    print('images={} labels={} background={} missing={} instances={}'.format(
        len(images), len(label_files) - missing, empty, missing, instances))
    if names is not None:
        for class_id in sorted(names):
            print('  class {} {}: {}'.format(
                class_id, names[class_id], class_counts.get(class_id, 0)))
    if missing:
        print('WARN: {} images have no label file (background or unlabeled)'
              .format(missing))
    for orphan in orphans[:10]:
        print('ERROR: orphan label {}'.format(orphan))
    for error in errors[:20]:
        print('ERROR: {}'.format(error))
    if orphans or errors:
        print('FAIL: {} label errors, {} orphan labels'.format(
            len(errors), len(orphans)))
        return 1
    if not images:
        print('FAIL: no images found')
        return 1
    print('OK: dataset structure and labels are valid')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
