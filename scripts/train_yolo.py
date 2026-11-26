#!/usr/bin/env python3
"""Train the external detector or the wrist pose model with Ultralytics.

The dataset must already be split and validated (``split_dataset.py`` and
``check_dataset.py``). The default models are the stock COCO detector and pose
checkpoints, so only the task-specific head is learned; pass a larger model or
more epochs when the dataset justifies it.

    ~/.venvs/jcheng-perception-gpu/bin/python scripts/train_yolo.py \
      --task detect --data ~/jcheng/perception_datasets/ext/data.yaml \
      --epochs 100 --install

    ~/.venvs/jcheng-perception-gpu/bin/python scripts/train_yolo.py \
      --task pose --data ~/jcheng/perception_datasets/wrist_pose/data.yaml \
      --epochs 100 --install

``--install`` copies ``best.pt`` to /opt/jcheng/models/ext_yolov8n.pt (detect)
or /opt/jcheng/models/wrist_grasp_pose.pt (pose). The repository never holds
weights or datasets.
"""

import argparse
import datetime
import pathlib
import shutil
import sys

import yaml

DEFAULT_MODEL = {
    'detect': 'yolov8n.pt',
    'pose': 'yolov8n-pose.pt',
}
DEFAULT_IMGSZ = {
    'detect': 640,
    'pose': 416,
}
INSTALL_NAME = {
    'detect': 'ext_yolov8n.pt',
    'pose': 'wrist_grasp_pose.pt',
}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--task', choices=('detect', 'pose'), required=True)
    parser.add_argument('--data', required=True)
    parser.add_argument('--model', default=None)
    parser.add_argument('--epochs', type=int, default=100)
    parser.add_argument('--imgsz', type=int, default=None)
    parser.add_argument('--batch', type=int, default=16)
    parser.add_argument('--device', default='0')
    parser.add_argument('--workers', type=int, default=4)
    parser.add_argument('--patience', type=int, default=30)
    parser.add_argument('--seed', type=int, default=0)
    parser.add_argument('--project', default='runs')
    parser.add_argument('--name', default=None)
    parser.add_argument(
        '--install', action='store_true',
        help='copy best.pt into /opt/jcheng/models')
    parser.add_argument('--model-dir', default='/opt/jcheng/models')
    args = parser.parse_args()

    data_path = pathlib.Path(args.data).expanduser()
    if not data_path.is_file():
        print('FAIL: data.yaml not found: {}'.format(data_path), file=sys.stderr)
        return 2
    data = yaml.safe_load(data_path.read_text(encoding='utf-8')) or {}
    names = data.get('names')
    if not names:
        print('FAIL: data.yaml has no names', file=sys.stderr)
        return 2
    if args.task == 'pose' and not data.get('kpt_shape'):
        print('FAIL: pose data.yaml needs kpt_shape', file=sys.stderr)
        return 2
    imgsz = args.imgsz or DEFAULT_IMGSZ[args.task]
    model_name = args.model or DEFAULT_MODEL[args.task]
    name = args.name or '{}_{}'.format(
        args.task, datetime.datetime.now().strftime('%Y%m%d_%H%M'))

    try:
        from ultralytics import YOLO
    except ImportError as error:
        print('FAIL: Ultralytics not available: {}'.format(error),
              file=sys.stderr)
        print('Use the GPU venv: ~/.venvs/jcheng-perception-gpu/bin/python',
              file=sys.stderr)
        return 2

    print('task={} model={} data={} imgsz={} epochs={} batch={} device={}'
          .format(args.task, model_name, data_path, imgsz, args.epochs,
                  args.batch, args.device))
    model = YOLO(model_name)
    model.train(
        data=str(data_path),
        epochs=args.epochs,
        imgsz=imgsz,
        batch=args.batch,
        device=args.device,
        workers=args.workers,
        patience=args.patience,
        seed=args.seed,
        project=args.project,
        name=name,
        plots=True,
        exist_ok=False,
    )
    save_dir = pathlib.Path(model.trainer.save_dir)
    best = save_dir / 'weights' / 'best.pt'
    print('run: {}'.format(save_dir))
    print('best: {}'.format(best))
    if not best.is_file():
        print('FAIL: best.pt was not produced', file=sys.stderr)
        return 1
    if args.install:
        destination = pathlib.Path(args.model_dir).expanduser() / \
            INSTALL_NAME[args.task]
        destination.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(best, destination)
        print('installed: {}'.format(destination))
        print('verify: python3 scripts/check_models.py --config '
              'ros2_ws/src/jcheng_perception_gpu/config/gpu.yaml --load')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
