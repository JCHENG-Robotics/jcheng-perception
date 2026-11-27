"""
Deterministic synthetic dual-view ore scenes with known instance correspondences.

This is the S4.1 data generator only. It renders ellipsoid "ore" proxies from
two calibrated pinhole cameras with a z-buffer, producing per-view RGB, metric
z-depth and instance label images plus the ground-truth cross-view pairs.

It is **not** evidence of real-ore association: the shapes are ellipsoids, the
surfaces are procedural, and the camera poses are synthetic. Every output is
labelled ``source=synthetic`` / ``annotation_method=procedural_ground_truth``.
"""

import dataclasses
import datetime
import json
import math
import os
from typing import Sequence, Tuple

import cv2
import numpy as np

from .types import sha256_file

GENERATOR_NAME = 'jcheng_perception_scene.synthetic_dual'
GENERATOR_VERSION = '1.0.0'
DUAL_VIEW_DATASET_SCHEMA_VERSION = 'jcheng.ore_dual_view_dataset.v1'

VIEW_IDS = ('ext', 'wrist')

_LIGHT_DIRECTION = np.asarray([-0.45, -0.35, 0.82], dtype=np.float64)
_LIGHT_DIRECTION /= np.linalg.norm(_LIGHT_DIRECTION)

_PALETTE = (
    (118, 126, 132),
    (82, 92, 100),
    (76, 96, 126),
    (92, 108, 136),
    (102, 112, 108),
    (70, 78, 86),
    (126, 132, 122),
    (62, 72, 80),
    (110, 100, 88),
    (88, 118, 112),
)


@dataclasses.dataclass(frozen=True)
class Camera:
    """A pinhole camera in the ``base_link`` frame.

    The camera frame follows the ROS optical convention: +z forward, +x right,
    +y down. ``camera_to_base`` maps camera coordinates to ``base_link``.
    """

    view_id: str
    width: int
    height: int
    fx: float
    fy: float
    cx: float
    cy: float
    camera_to_base: np.ndarray
    fov_deg: float = 55.0

    def __post_init__(self):
        """Validate dimensions and normalise the transform."""
        if self.view_id not in VIEW_IDS:
            raise ValueError('unsupported view_id: %s' % self.view_id)
        if int(self.width) < 32 or int(self.height) < 32:
            raise ValueError('camera dimensions must be at least 32 pixels')
        if self.fx <= 0 or self.fy <= 0:
            raise ValueError('focal lengths must be positive')
        transform = np.asarray(self.camera_to_base, dtype=np.float64)
        if transform.shape != (4, 4):
            raise ValueError('camera_to_base must be 4x4')
        if not np.all(np.isfinite(transform)):
            raise ValueError('camera_to_base must be finite')
        object.__setattr__(self, 'camera_to_base', transform)

    @property
    def base_to_camera(self):
        """Return the inverse transform."""
        return np.linalg.inv(self.camera_to_base)

    @property
    def position_base(self):
        """Return the camera centre in ``base_link``."""
        return self.camera_to_base[:3, 3].copy()

    @property
    def rotation_world_to_camera(self):
        """Return the 3x3 rotation from ``base_link`` to camera."""
        return self.base_to_camera[:3, :3]

    def as_dict(self):
        """Return a JSON-serialisable calibration record."""
        return {
            'view_id': self.view_id,
            'width': int(self.width),
            'height': int(self.height),
            'fx': float(self.fx),
            'fy': float(self.fy),
            'cx': float(self.cx),
            'cy': float(self.cy),
            'fov_deg': float(self.fov_deg),
            'frame_id': 'base_link',
            'camera_to_base': self.camera_to_base.tolist(),
        }


@dataclasses.dataclass(frozen=True)
class Ellipsoid:
    """An ore proxy resting on the z=0 plane.

    ``center_base_m`` is the ellipsoid centre; the vertical semi-axis is the
    distance to the support plane, so the shape is tangent to z=0. ``yaw_rad``
    rotates the horizontal semi-axes only.
    """

    global_id: str
    center_base_m: Tuple[float, float, float]
    radii_m: Tuple[float, float, float]
    yaw_rad: float = 0.0
    color_bgr: Tuple[int, int, int] = (100, 100, 100)

    def __post_init__(self):
        """Validate the shape parameters."""
        if not self.global_id:
            raise ValueError('global_id must be non-empty')
        center = np.asarray(self.center_base_m, dtype=np.float64)
        radii = np.asarray(self.radii_m, dtype=np.float64)
        if center.shape != (3,) or not np.all(np.isfinite(center)):
            raise ValueError('center_base_m must be three finite values')
        if radii.shape != (3,) or not np.all(np.isfinite(radii)):
            raise ValueError('radii_m must be three finite values')
        if np.any(radii <= 0):
            raise ValueError('radii_m must be positive')
        if not math.isfinite(float(self.yaw_rad)):
            raise ValueError('yaw_rad must be finite')
        object.__setattr__(self, 'center_base_m', tuple(
            float(value) for value in center))
        object.__setattr__(self, 'radii_m', tuple(
            float(value) for value in radii))
        object.__setattr__(self, 'color_bgr', tuple(
            int(value) for value in self.color_bgr))

    @property
    def support_height_m(self):
        """Return the centre height above the plane."""
        return float(self.center_base_m[2])

    def as_dict(self):
        """Return a JSON-serialisable geometry record."""
        return {
            'global_id': self.global_id,
            'center_base_m': [float(value) for value in self.center_base_m],
            'radii_m': [float(value) for value in self.radii_m],
            'yaw_rad': float(self.yaw_rad),
            'support_height_m': self.support_height_m,
        }


@dataclasses.dataclass
class ViewRender:
    """Rendered arrays for one camera."""

    image_bgr: np.ndarray
    depth_m: np.ndarray
    label: np.ndarray
    full_silhouette_px: np.ndarray


def look_at_camera(view_id, eye_base_m, target_base_m, width, height,
                   fov_deg=55.0):
    """Build an optical-frame camera looking from ``eye`` at ``target``."""
    eye = np.asarray(eye_base_m, dtype=np.float64)
    target = np.asarray(target_base_m, dtype=np.float64)
    if eye.shape != (3,) or target.shape != (3,):
        raise ValueError('eye and target must be three values')
    forward = target - eye
    norm = np.linalg.norm(forward)
    if norm <= 1e-9:
        raise ValueError('eye and target coincide')
    forward /= norm
    up = np.asarray([0.0, 0.0, 1.0])
    if abs(float(np.dot(forward, up))) > 0.99:
        up = np.asarray([0.0, 1.0, 0.0])
    right = np.cross(forward, up)
    right_norm = np.linalg.norm(right)
    if right_norm <= 1e-9:
        raise ValueError('cannot build a camera basis')
    right /= right_norm
    down = np.cross(forward, right)
    down /= np.linalg.norm(down)
    rotation_world_to_camera = np.vstack([right, down, forward])
    camera_to_base = np.eye(4, dtype=np.float64)
    camera_to_base[:3, :3] = rotation_world_to_camera.T
    camera_to_base[:3, 3] = eye
    width = int(width)
    height = int(height)
    focal = width / (2.0 * math.tan(math.radians(float(fov_deg)) / 2.0))
    return Camera(
        view_id=view_id,
        width=width,
        height=height,
        fx=focal,
        fy=focal,
        cx=(width - 1) / 2.0,
        cy=(height - 1) / 2.0,
        camera_to_base=camera_to_base,
        fov_deg=float(fov_deg),
    )


def default_cameras(width=480, height=360):
    """Return the two fixed, calibrated camera poses used by the generator."""
    return {
        'ext': look_at_camera(
            'ext', (0.52, -0.38, 0.42), (0.0, 0.0, 0.026),
            width, height, fov_deg=55.0),
        'wrist': look_at_camera(
            'wrist', (0.38, 0.34, 0.36), (0.0, 0.0, 0.026),
            width, height, fov_deg=55.0),
    }


def _rays_world(camera):
    """Return the per-pixel ray origin and world directions."""
    rows, columns = np.mgrid[0:camera.height, 0:camera.width]
    directions_camera = np.stack([
        (columns - camera.cx) / camera.fx,
        (rows - camera.cy) / camera.fy,
        np.ones_like(columns, dtype=np.float64),
    ], axis=-1)
    rotation = camera.rotation_world_to_camera
    directions_world = directions_camera.reshape(-1, 3) @ rotation
    return camera.position_base, directions_world.reshape(
        camera.height, camera.width, 3)


def _intersect_ellipsoid(origin, directions, ellipsoid):
    """Return nearest positive t, hit mask and full silhouette mask."""
    center = np.asarray(ellipsoid.center_base_m, dtype=np.float64)
    radii = np.asarray(ellipsoid.radii_m, dtype=np.float64)
    cosine = math.cos(float(ellipsoid.yaw_rad))
    sine = math.sin(float(ellipsoid.yaw_rad))
    local_to_world = np.asarray([
        [cosine, -sine, 0.0],
        [sine, cosine, 0.0],
        [0.0, 0.0, 1.0],
    ])
    offset = (origin - center) @ local_to_world
    local_directions = directions.reshape(-1, 3) @ local_to_world
    offset = offset / radii
    local_directions = local_directions / radii
    a = np.einsum('ij,ij->i', local_directions, local_directions)
    b = 2.0 * np.einsum('ij,j->i', local_directions, offset)
    c = float(np.dot(offset, offset)) - 1.0
    discriminant = b * b - 4.0 * a * c
    valid = (discriminant >= 0.0) & (a > 1e-14)
    near = np.full(a.shape, np.inf)
    far = np.full(a.shape, np.inf)
    if np.any(valid):
        root = np.sqrt(discriminant[valid])
        denominator = 2.0 * a[valid]
        near[valid] = (-b[valid] - root) / denominator
        far[valid] = (-b[valid] + root) / denominator
    hit = valid & (far > 1e-9)
    t = np.where(near > 1e-9, near, far)
    return t, hit, hit


def _background(height, width):
    """Return a deterministic dark belt-like background."""
    rows, columns = np.mgrid[0:height, 0:width]
    base = 30.0 + 6.0 * (rows / max(height - 1, 1))
    vignette = 1.0 - 0.20 * (
        ((columns - width / 2.0) ** 2 + (rows - height / 2.0) ** 2)
        / float(max(width * width + height * height, 1)))
    image = np.empty((height, width, 3), dtype=np.float64)
    image[:, :, 0] = base * 0.92 * vignette
    image[:, :, 1] = base * 0.98 * vignette
    image[:, :, 2] = base * 1.05 * vignette
    return image


def render_view(ellipsoids: Sequence[Ellipsoid], camera: Camera):
    """Render one camera with a z-buffer; returns :class:`ViewRender`."""
    camera = _require_camera(camera)
    origin, directions = _rays_world(camera)
    height, width = camera.height, camera.width
    best_t = np.full((height, width), np.inf, dtype=np.float64)
    label = np.zeros((height, width), dtype=np.uint16)
    full_silhouette = np.zeros(len(ellipsoids), dtype=np.int64)
    for index, ellipsoid in enumerate(ellipsoids):
        t, hit, silhouette = _intersect_ellipsoid(origin, directions, ellipsoid)
        t = t.reshape(height, width)
        hit = hit.reshape(height, width)
        silhouette = silhouette.reshape(height, width)
        full_silhouette[index] = int(np.count_nonzero(silhouette))
        closer = hit & (t < best_t)
        best_t[closer] = t[closer]
        label[closer] = index + 1
    image = _background(height, width)
    valid = label > 0
    if np.any(valid):
        hit_points = origin[None, None, :] + \
            best_t[:, :, None] * directions
        for index, ellipsoid in enumerate(ellipsoids):
            mask = label == index + 1
            if not np.any(mask):
                continue
            center = np.asarray(ellipsoid.center_base_m, dtype=np.float64)
            radii = np.asarray(ellipsoid.radii_m, dtype=np.float64)
            cosine = math.cos(float(ellipsoid.yaw_rad))
            sine = math.sin(float(ellipsoid.yaw_rad))
            local_to_world = np.asarray([
                [cosine, -sine, 0.0],
                [sine, cosine, 0.0],
                [0.0, 0.0, 1.0],
            ])
            points = hit_points[mask]
            local = (points - center) @ local_to_world
            normal_local = local / (radii * radii)
            normal_world = normal_local @ local_to_world.T
            lengths = np.linalg.norm(normal_world, axis=1)
            lengths[lengths <= 1e-12] = 1.0
            normal_world = normal_world / lengths[:, None]
            diffuse = np.clip(normal_world @ _LIGHT_DIRECTION, 0.0, 1.0)
            shade = 0.34 + 0.66 * diffuse
            base_color = np.asarray(ellipsoid.color_bgr, dtype=np.float64)
            rows, columns = np.nonzero(mask)
            # A cheap deterministic surface variation, kept on the visible part.
            texture = 1.0 + 0.06 * np.sin(
                columns * 0.31 + rows * 0.17 + index * 1.7)
            colors = shade[:, None] * base_color[None, :] * texture[:, None]
            image[rows, columns] = np.clip(colors, 0.0, 255.0)
    depth = np.zeros((height, width), dtype=np.float32)
    depth[valid] = best_t[valid].astype(np.float32)
    image_uint8 = np.clip(image, 0.0, 255.0).astype(np.uint8)
    return ViewRender(
        image_bgr=image_uint8,
        depth_m=depth,
        label=label,
        full_silhouette_px=full_silhouette,
    )


def _require_camera(camera):
    if not isinstance(camera, Camera):
        raise TypeError('camera must be a Camera instance')
    return camera


def _scene_rng(seed, index):
    return np.random.default_rng(int(seed) + int(index) * 1000003)


def _generate_ellipsoids(rng, count, scene_radius_m=0.18):
    """Place ``count`` ellipsoids on the z=0 plane inside a disk."""
    ellipsoids = []
    for index in range(int(count)):
        radius = math.sqrt(float(rng.random())) * scene_radius_m
        angle = float(rng.uniform(0.0, 2.0 * math.pi))
        x = radius * math.cos(angle)
        y = radius * math.sin(angle)
        semi_a = float(rng.uniform(0.030, 0.060))
        semi_b = float(rng.uniform(0.028, 0.055))
        semi_c = float(rng.uniform(0.022, 0.045))
        yaw = float(rng.uniform(0.0, 2.0 * math.pi))
        colour = _PALETTE[int(rng.integers(0, len(_PALETTE)))]
        ellipsoids.append(Ellipsoid(
            global_id='g%04d' % (index + 1),
            center_base_m=(x, y, semi_c),
            radii_m=(semi_a, semi_b, semi_c),
            yaw_rad=yaw,
            color_bgr=colour,
        ))
    return ellipsoids


def _visible_record(label, full_silhouette_px, instance_index,
                    scene_id, view_id):
    """Build the per-view instance record for one ellipsoid."""
    mask = label == instance_index + 1
    area = int(np.count_nonzero(mask))
    full_area = int(full_silhouette_px[instance_index])
    if area == 0:
        return None
    rows, columns = np.nonzero(mask)
    return {
        'instance_id': '%s:%s:%04d' % (scene_id, view_id, instance_index + 1),
        'mask_index': instance_index + 1,
        'visible_area_px': area,
        'full_silhouette_px': full_area,
        'occlusion_ratio': round(
            max(0.0, 1.0 - area / float(max(full_area, 1))), 6),
        'bbox_xyxy': [
            int(columns.min()), int(rows.min()),
            int(columns.max()), int(rows.max()),
        ],
        'centroid_px': [
            round(float(columns.mean()), 3),
            round(float(rows.mean()), 3),
        ],
    }


def generate_dual_view_scene(scene_id, seed, index, width, height,
                             instance_count, cameras=None, created_at=None,
                             with_renders=False):
    """Generate one scene and return its records.

    Returns ``(scene_record, pair_record)`` by default, or
    ``(scene_record, pair_record, renders)`` when ``with_renders`` is true.
    """
    cameras = cameras or default_cameras(width, height)
    rng = _scene_rng(seed, index)
    ellipsoids = _generate_ellipsoids(rng, instance_count)
    renders = {
        view_id: render_view(ellipsoids, cameras[view_id])
        for view_id in VIEW_IDS
    }
    views = {}
    for view_id, render in renders.items():
        views[view_id] = {
            'view_id': view_id,
            'width': int(width),
            'height': int(height),
            'timestamp_ns': 0,
            'depth_invalid_value': 0.0,
            'full_silhouette_px': [
                int(value) for value in render.full_silhouette_px],
        }
    instances = []
    pairs = []
    singletons = []
    for index, ellipsoid in enumerate(ellipsoids):
        per_view = {}
        for view_id, render in renders.items():
            record = _visible_record(
                render.label, render.full_silhouette_px, index,
                scene_id, view_id)
            if record is not None:
                per_view[view_id] = record
        instance = ellipsoid.as_dict()
        instance['global_id'] = '%s:%s' % (scene_id, ellipsoid.global_id)
        instance['views'] = per_view
        instances.append(instance)
        if len(per_view) == len(VIEW_IDS):
            pairs.append({
                'global_id': instance['global_id'],
                'members': [
                    {'view_id': view_id, 'instance_id':
                     per_view[view_id]['instance_id']}
                    for view_id in VIEW_IDS
                ],
            })
        elif per_view:
            missing = [
                view_id for view_id in VIEW_IDS if view_id not in per_view]
            for view_id in per_view:
                singletons.append({
                    'view_id': view_id,
                    'instance_id': per_view[view_id]['instance_id'],
                    'global_id': instance['global_id'],
                    'reason': 'not_visible_in_%s' % missing[0],
                })
    scene = {
        'schema_version': DUAL_VIEW_DATASET_SCHEMA_VERSION,
        'scene_id': scene_id,
        'created_at': created_at,
        'timestamp_ns': 0,
        'frame_id': 'base_link',
        'views': views,
        'instances': instances,
        'attributes': {
            'source': 'synthetic',
            'n_instances': len(instances),
            'n_visible_instances': sum(
                1 for instance in instances if instance['views']),
            'n_pairs': len(pairs),
            'n_singletons': len(singletons),
            'empty': len(instances) == 0,
        },
    }
    pair_record = {
        'scene_id': scene_id,
        'pairs': pairs,
        'singletons': singletons,
    }
    if with_renders:
        return scene, pair_record, renders
    return scene, pair_record


def generate_dual_view_dataset(out_dir, scene_count=6, empty_count=1,
                               seed=20261127, width=480, height=360,
                               min_instances=8, max_instances=18,
                               created_at=None):
    """Generate a deterministic dual-view dataset; return the manifest path."""
    scene_count = int(scene_count)
    empty_count = int(empty_count)
    min_instances = int(min_instances)
    max_instances = int(max_instances)
    if scene_count < 0 or empty_count < 0:
        raise ValueError('scene counts must be non-negative')
    if int(width) < 64 or int(height) < 64:
        raise ValueError('image dimensions must be at least 64 pixels')
    if min_instances < 0 or max_instances < min_instances:
        raise ValueError('invalid instance-count range')
    cameras = default_cameras(int(width), int(height))
    for directory in ('images', 'masks', 'depth'):
        os.makedirs(os.path.join(out_dir, directory), exist_ok=True)
    if created_at is None:
        created_at = datetime.datetime.now(
            datetime.timezone.utc).replace(microsecond=0).isoformat()

    scenes = []
    pair_records = []
    total = scene_count + empty_count
    for index in range(total):
        scene_id = 'dualsyn-%d-%03d' % (int(seed), index)
        if index < scene_count:
            rng = _scene_rng(seed, index)
            count = int(rng.integers(min_instances, max_instances + 1))
        else:
            count = 0
        scene, pair_record, renders = generate_dual_view_scene(
            scene_id, seed, index, int(width), int(height), count,
            cameras=cameras, created_at=created_at, with_renders=True)
        for view_id, render in renders.items():
            image_path = os.path.join(
                out_dir, 'images', '%s_%s.png' % (scene_id, view_id))
            mask_path = os.path.join(
                out_dir, 'masks', '%s_%s.png' % (scene_id, view_id))
            depth_path = os.path.join(
                out_dir, 'depth', '%s_%s.npy' % (scene_id, view_id))
            if not cv2.imwrite(image_path, render.image_bgr):
                raise RuntimeError('failed to write %s' % image_path)
            if not cv2.imwrite(mask_path, render.label.astype(np.uint16)):
                raise RuntimeError('failed to write %s' % mask_path)
            np.save(depth_path, render.depth_m)
            scene['views'][view_id].update({
                'image': os.path.join('images', os.path.basename(image_path)),
                'mask': os.path.join('masks', os.path.basename(mask_path)),
                'depth': os.path.join('depth', os.path.basename(depth_path)),
                'image_sha256': sha256_file(image_path),
                'mask_sha256': sha256_file(mask_path),
                'depth_sha256': sha256_file(depth_path),
            })
        scenes.append(scene)
        pair_records.append(pair_record)

    manifest = {
        'schema_version': DUAL_VIEW_DATASET_SCHEMA_VERSION,
        'dataset_id': 'dual_view_%d' % int(seed),
        'source': 'synthetic',
        'annotation_method': 'procedural_ground_truth',
        'license': 'CC0-1.0',
        'citation': None,
        'created_at': created_at,
        'generator': {
            'name': GENERATOR_NAME,
            'version': GENERATOR_VERSION,
            'seed': int(seed),
            'scene_count': scene_count,
            'empty_count': empty_count,
            'width': int(width),
            'height': int(height),
            'min_instances': min_instances,
            'max_instances': max_instances,
        },
        'cameras': {
            view_id: camera.as_dict()
            for view_id, camera in cameras.items()
        },
        'pairs_path': 'pairs.jsonl',
        'scenes': scenes,
    }
    manifest_path = os.path.join(out_dir, 'dataset.json')
    with open(manifest_path, 'w', encoding='utf-8') as handle:
        json.dump(manifest, handle, indent=2, sort_keys=True)
        handle.write('\n')
    with open(os.path.join(out_dir, 'pairs.jsonl'), 'w',
              encoding='utf-8') as handle:
        for record in pair_records:
            handle.write(json.dumps(record, sort_keys=True))
            handle.write('\n')
    _write_dataset_card(out_dir, manifest)
    return manifest_path


def _write_dataset_card(out_dir, manifest):
    path = os.path.join(out_dir, 'DATASET_CARD.md')
    lines = [
        '# Synthetic dual-view ore dataset',
        '',
        '- schema: `%s`' % manifest['schema_version'],
        '- dataset_id: `%s`' % manifest['dataset_id'],
        '- source: **synthetic** (ellipsoid proxies, no real ore)',
        '- annotation: **procedural ground truth**, pixel-accurate by '
        'construction',
        '- license of generated data: **CC0-1.0**',
        '- generator: `%s` %s' % (
            manifest['generator']['name'], manifest['generator']['version']),
        '- scenes: %d (%d empty)' % (
            len(manifest['scenes']), manifest['generator']['empty_count']),
        '- cameras: `ext` and `wrist`, fixed synthetic poses in `base_link`',
        '',
        '## Limits',
        '',
        'This dataset exists to test multi-view association code and its',
        'evaluation protocol. It says nothing about real ore appearance, real',
        'camera calibration, dust, water, specularity or occlusion by the arm.',
        'Do not report synthetic association metrics as real-robot validation.',
        '',
    ]
    with open(path, 'w', encoding='utf-8') as handle:
        handle.write('\n'.join(lines))
