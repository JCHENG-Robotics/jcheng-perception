#!/usr/bin/env python3
"""Split labeled capture sessions into a YOLO train/val dataset.

The raw capture layout is ``<raw>/<session>/<camera>/<stamp>.jpg``; the labeler
writes a YOLO ``.txt`` next to each image. Sessions are split as whole units
(not frames) so frames from the same scene cannot leak across train and val.

    python3 scripts/split_dataset.py \
      --raw ~/jcheng/perception_datasets/raw --camera d435i_ext \
      --out ~/jcheng/perception_datasets/ext --task detect \
      --names cube,blue_block,red_cylinder,vlt_carton,snack_sachet

The output is ``images/{train,val}``, ``labels/{train,val}`` and ``data.yaml``.
"""

import argparse
import pathlib
import random
import shutil
import sys

import yaml

IMAGE_EXTENSIONS = {'.jpg', '.jpeg', '.png', '.bmp', '.webp'}


def collect(raw_root, camera):
    sessions = {}
    unlabeled = 0
    for session in sorted(path for path in raw_root.iterdir() if path.is_dir()):
        camera_dir = session / camera
        if not camera_dir.is_dir():
            continue
        pairs = []
        for image in sorted(
                path for path in camera_dir.iterdir()
                if path.is_file() and path.suffix.lower() in IMAGE_EXTENSIONS):
            label = image.with_suffix('.txt')
            if label.is_file():
                pairs.append((image, label))
            else:
                unlabeled += 1
        if pairs:
            sessions[session.name] = pairs
    return sessions, unlabeled


def copy_pairs(pairs, session, images_dir, labels_dir):
    images_dir.mkdir(parents=True, exist_ok=True)
    labels_dir.mkdir(parents=True, exist_ok=True)
    for image, label in pairs:
        stem = '{}__{}'.format(session, image.stem)
        shutil.copy2(
            image, images_dir / '{}{}'.format(stem, image.suffix.lower()))
        shutil.copy2(label, labels_dir / '{}.txt'.format(stem))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--raw', required=True)
    parser.add_argument('--out', required=True)
    parser.add_argument('--camera', default='d435i_ext')
    parser.add_argument('--task', choices=('detect', 'pose'), default='detect')
    parser.add_argument(
        '--names', required=True,
        help='comma-separated class names in class-id order')
    parser.add_argument('--kpt-shape', type=int, nargs=2, default=(2, 3))
    parser.add_argument('--val-fraction', type=float, default=0.2)
    parser.add_argument('--seed', type=int, default=0)
    parser.add_argument('--dry-run', action='store_true')
    args = parser.parse_args()

    names = [item.strip() for item in args.names.split(',') if item.strip()]
    if not names:
        parser.error('--names is empty')
    if not 0.0 <= args.val_fraction < 1.0:
        parser.error('--val-fraction must be in [0,1)')

    raw_root = pathlib.Path(args.raw).expanduser()
    out = pathlib.Path(args.out).expanduser()
    if not raw_root.is_dir():
        print('FAIL: raw root not found: {}'.format(raw_root), file=sys.stderr)
        return 2
    sessions, unlabeled = collect(raw_root, args.camera)
    if not sessions:
        print('FAIL: no labeled {} images under {}'.format(
            args.camera, raw_root), file=sys.stderr)
        return 1
    if unlabeled:
        print('WARN: {} images have no .txt label and are skipped'.format(
            unlabeled))

    names_list = list(sessions)
    random.Random(args.seed).shuffle(names_list)
    if len(names_list) == 1:
        print('FAIL: only one labeled session; collect at least two sessions '
              'so train and val do not share a scene', file=sys.stderr)
        return 1
    val_count = max(1, round(len(names_list) * args.val_fraction))
    val_count = min(val_count, len(names_list) - 1)
    val_sessions = set(names_list[:val_count])
    train_sessions = [name for name in names_list if name not in val_sessions]

    print('camera={} sessions={} val_sessions={}'.format(
        args.camera, len(names_list), sorted(val_sessions)))
    total = {'train': 0, 'val': 0}
    for split, selected in (('train', train_sessions),
                            ('val', sorted(val_sessions))):
        count = sum(len(sessions[name]) for name in selected)
        total[split] = count
        print('  {}: {} sessions, {} images'.format(split, len(selected), count))
        if args.dry_run:
            continue
        for name in selected:
            copy_pairs(
                sessions[name], name,
                out / 'images' / split, out / 'labels' / split)

    if total['train'] == 0 or total['val'] == 0:
        print('FAIL: empty split (train={}, val={})'.format(
            total['train'], total['val']), file=sys.stderr)
        return 1
    if args.dry_run:
        return 0

    data = {
        'path': str(out.resolve()),
        'train': 'images/train',
        'val': 'images/val',
        'names': {index: name for index, name in enumerate(names)},
    }
    if args.task == 'pose':
        data['kpt_shape'] = list(args.kpt_shape)
    (out / 'data.yaml').write_text(
        yaml.safe_dump(data, sort_keys=False), encoding='utf-8')
    print('wrote {}'.format(out / 'data.yaml'))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
