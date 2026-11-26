"""
Deterministic synthetic ore-pile generator with pixel-accurate ground truth.

The generator exists to exercise the offline pipeline and the evaluation code.
It is **not** evidence of real-ore recognition: every produced scene is labelled
``source=synthetic`` and ``annotation_method=procedural_ground_truth``.
"""

import datetime
import json
import math
import os

import cv2
import numpy as np

from .types import DATASET_SCHEMA_VERSION, sha256_file

GENERATOR_NAME = 'jcheng_perception_scene.synthetic'
GENERATOR_VERSION = '1.0.0'

ILLUMINATIONS = ('dark', 'normal', 'bright')
DENSITIES = ('sparse', 'medium', 'dense')

_DENSITY_PLAN = {
    'sparse': (4, 7),
    'medium': (12, 18),
    'dense': (26, 38),
}

_PALETTE = (
    (112, 116, 120),
    (78, 88, 96),
    (72, 92, 122),
    (86, 104, 132),
    (96, 108, 104),
    (66, 74, 82),
    (120, 128, 118),
    (58, 68, 76),
)


def _scene_rng(seed, index):
    """Return a scene-local generator isolated from other scenes."""
    return np.random.default_rng(int(seed) + int(index) * 1000003)


def _belt_background(rng, width, height):
    """Render a dark conveyor-belt-like background."""
    base = rng.uniform(22.0, 36.0)
    img = np.full((height, width, 3), base, dtype=np.float64)
    img += rng.normal(0.0, 4.0, img.shape)
    seam_step = int(max(48, height // 7))
    for y in range(seam_step // 2, height, seam_step):
        thickness = 1 if rng.random() < 0.7 else 2
        img[y:y + thickness, :, :] *= rng.uniform(0.72, 0.86)
    # Rubber grain and a mild vignette.
    yy, xx = np.mgrid[0:height, 0:width]
    radial = ((xx - width / 2.0) ** 2 + (yy - height / 2.0) ** 2)
    scale = max(width * width + height * height, 1.0)
    vignette = 1.0 - 0.18 * radial / scale
    img *= vignette[:, :, None]
    return np.clip(img, 0.0, 255.0)


def _ore_polygon(rng, cx, cy, radius, vertices=12, irregularity=0.26):
    """Return an irregular, non-self-intersecting polygon around a centre."""
    angles = np.linspace(0.0, 2.0 * math.pi, vertices, endpoint=False)
    angles = angles + rng.uniform(-0.10, 0.10, vertices)
    radial = 1.0 + rng.normal(0.0, irregularity, vertices)
    radial = np.clip(radial, 0.58, 1.42)
    points = np.column_stack([
        cx + radius * radial * np.cos(angles),
        cy + radius * radial * np.sin(angles),
    ])
    return np.round(points).astype(np.int32)


def _draw_shadow(img, polygon, rng):
    """Darken the image below and to the right of an ore polygon."""
    height, width = img.shape[:2]
    shift = np.array([
        int(rng.integers(4, 10)),
        int(rng.integers(6, 14)),
    ], dtype=np.int32)
    shadow = np.zeros((height, width), dtype=np.uint8)
    cv2.fillPoly(shadow, [polygon + shift], 255)
    shadow = cv2.GaussianBlur(shadow, (0, 0), 5.0).astype(np.float64) / 255.0
    img *= (1.0 - 0.28 * shadow[:, :, None])


def _texture_ore(img, polygon, rng, base_color):
    """Paint a textured ore surface inside ``polygon`` in place."""
    height, width = img.shape[:2]
    mask = np.zeros((height, width), dtype=np.uint8)
    cv2.fillPoly(mask, [polygon], 255)
    rows, columns = np.nonzero(mask)
    if rows.size == 0:
        return mask.astype(bool)
    x0, x1 = int(columns.min()), int(columns.max())
    y0, y1 = int(rows.min()), int(rows.max())
    roi = img[y0:y1 + 1, x0:x1 + 1]
    local = mask[y0:y1 + 1, x0:x1 + 1].astype(bool)
    local_h, local_w = local.shape
    yy, xx = np.mgrid[0:local_h, 0:local_w]

    noise = rng.normal(0.0, 9.0, (local_h, local_w, 1))
    gradient = (
        (xx / max(local_w, 1) - 0.5) * rng.uniform(-22.0, 22.0)
        + (yy / max(local_h, 1) - 0.5) * rng.uniform(-18.0, 18.0)
    )
    colors = (
        np.asarray(base_color, dtype=np.float64)[None, None, :]
        + noise + gradient[:, :, None]
    )
    roi[local] = np.clip(colors[local], 0.0, 255.0)

    vein_color = np.asarray(base_color, dtype=np.float64) * 0.62
    for _ in range(int(rng.integers(1, 4))):
        first = (
            int(rng.integers(0, max(local_w, 1))),
            int(rng.integers(0, max(local_h, 1))),
        )
        second = (
            int(rng.integers(0, max(local_w, 1))),
            int(rng.integers(0, max(local_h, 1))),
        )
        vein = np.zeros((local_h, local_w), dtype=np.uint8)
        cv2.line(vein, first, second, 255, 1)
        vein = (vein > 0) & local
        roi[vein] = roi[vein] * 0.70 + vein_color[None, :] * 0.30

    # One specular patch per stone, clipped to the stone.
    spot = np.zeros((local_h, local_w), dtype=np.uint8)
    centre = (
        int(rng.integers(local_w // 5, max(local_w * 4 // 5, 2))),
        int(rng.integers(local_h // 5, max(local_h * 4 // 5, 2))),
    )
    radius = max(2, int(min(local_w, local_h) * rng.uniform(0.06, 0.14)))
    cv2.circle(spot, centre, radius, 255, -1)
    spot = cv2.GaussianBlur(spot, (0, 0), radius / 1.6).astype(np.float64)
    spot = (spot / 255.0) * local.astype(np.float64)
    roi += spot[:, :, None] * np.array([26.0, 26.0, 24.0])[None, None, :]
    img[y0:y1 + 1, x0:x1 + 1] = np.clip(roi, 0.0, 255.0)
    return local


def _apply_illumination(img, rng, illumination):
    """Apply a deterministic illumination level and sensor noise."""
    factors = {'dark': 0.55, 'normal': 1.0, 'bright': 1.42}
    gammas = {'dark': 1.18, 'normal': 1.0, 'bright': 0.90}
    img = img * factors[illumination]
    img = np.clip(img, 0.0, 255.0) / 255.0
    img = np.power(img, gammas[illumination]) * 255.0
    sigma = 3.0 if illumination == 'normal' else 5.0
    img += rng.normal(0.0, sigma, img.shape)
    return np.clip(img, 0.0, 255.0).astype(np.uint8)


def _place_centres(rng, count, width, height, radius, scale):
    """Place stones in a few clusters so touching and occlusion occur."""
    clusters = max(1, int(round(math.sqrt(max(count, 1)) / 2.0)))
    centres = []
    cluster_centres = [
        (
            rng.uniform(0.22 * width, 0.78 * width),
            rng.uniform(0.22 * height, 0.78 * height),
        )
        for _ in range(clusters)
    ]
    margin = radius + 4
    for index in range(count):
        base_x, base_y = cluster_centres[index % clusters]
        x = rng.normal(base_x, 0.13 * width)
        y = rng.normal(base_y, 0.13 * height)
        x = float(np.clip(x, margin, width - margin))
        y = float(np.clip(y, margin, height - margin))
        centres.append((x, y))
    return centres


def generate_scene(scene_id, seed, index, width, height, density,
                   illumination):
    """Generate one scene and return ``(image, mask, record)``."""
    rng = _scene_rng(seed, index)
    scale = min(width, height) / 480.0
    if density == 'empty':
        count = 0
    else:
        low, high = _DENSITY_PLAN[density]
        count = int(rng.integers(low, high + 1))

    image = _belt_background(rng, width, height)
    id_buffer = np.zeros((height, width), dtype=np.uint16)
    polygons = []
    full_areas = []
    colours = []

    if count:
        base_radius = (30.0 if density == 'sparse'
                       else 25.0 if density == 'medium' else 21.0) * scale
        radius_low = int(max(10, base_radius * 0.70))
        radius_high = int(max(radius_low + 2, base_radius * 1.35))
        centres = _place_centres(
            rng, count, width, height, radius_high, scale)
        for stone_index in range(count):
            radius = float(rng.integers(radius_low, radius_high + 1))
            polygon = _ore_polygon(rng, centres[stone_index][0],
                                   centres[stone_index][1], radius)
            _draw_shadow(image, polygon, rng)
            full_mask = np.zeros((height, width), dtype=np.uint8)
            cv2.fillPoly(full_mask, [polygon], 255)
            colour = _PALETTE[int(rng.integers(0, len(_PALETTE)))]
            colour = tuple(
                float(np.clip(channel + rng.normal(0.0, 10.0), 20.0, 190.0))
                for channel in colour
            )
            _texture_ore(image, polygon, rng, colour)
            id_buffer[full_mask > 0] = stone_index + 1
            polygons.append(polygon)
            full_areas.append(int(np.count_nonzero(full_mask)))
            colours.append(colour)

    image = _apply_illumination(image, rng, illumination)

    instances = []
    occluded_count = 0
    touching_count = 0
    for stone_index in range(count):
        visible = id_buffer == stone_index + 1
        visible_area = int(np.count_nonzero(visible))
        if visible_area == 0:
            continue
        full_area = max(full_areas[stone_index], visible_area)
        occlusion_ratio = 1.0 - float(visible_area) / float(full_area)
        boundary = visible.astype(np.uint8)
        boundary = boundary - cv2.erode(boundary, np.ones((3, 3), np.uint8))
        boundary_px = int(np.count_nonzero(boundary))
        dilated = cv2.dilate(
            visible.astype(np.uint8), np.ones((5, 5), np.uint8)) > 0
        other = (id_buffer > 0) & (id_buffer != stone_index + 1)
        contact_px = int(np.count_nonzero(dilated & other))
        contact_ratio = contact_px / float(max(boundary_px, 1))
        if occlusion_ratio >= 0.15:
            stratum = 'occluded'
            occluded_count += 1
        elif contact_px >= 8:
            stratum = 'touching'
            touching_count += 1
        else:
            stratum = 'isolated'
        rows, columns = np.nonzero(visible)
        instances.append({
            'instance_id': '%s:%04d' % (scene_id, stone_index + 1),
            'mask_index': stone_index + 1,
            'area_px': visible_area,
            'full_area_px': full_area,
            'occlusion_ratio': round(occlusion_ratio, 6),
            'contact_px': contact_px,
            'contact_ratio': round(contact_ratio, 6),
            'bbox_xyxy': [
                int(columns.min()), int(rows.min()),
                int(columns.max()), int(rows.max()),
            ],
            'stratum': stratum,
            'colour_bgr': [round(float(value), 2) for value in colours[
                stone_index]],
        })

    if occluded_count:
        occlusion_class = 'occluded'
    elif touching_count:
        occlusion_class = 'touching'
    else:
        occlusion_class = 'none'

    record = {
        'scene_id': scene_id,
        'image': 'images/%s.png' % scene_id,
        'mask': 'masks/%s.png' % scene_id,
        'width': int(width),
        'height': int(height),
        'timestamp_ns': 0,
        'frame_id': 'camera_color_optical_frame',
        'attributes': {
            'source': 'synthetic',
            'illumination': illumination,
            'density': density,
            'empty': count == 0,
            'n_instances': len(instances),
            'occluded_instances': occluded_count,
            'touching_instances': touching_count,
            'occlusion_class': occlusion_class,
            'seed': int(seed) + int(index) * 1000003,
        },
        'instances': instances,
    }
    return image, id_buffer, record


def _scene_plan(scene_count, empty_count):
    """Return a balanced (density, illumination) plan plus empty scenes."""
    plan = []
    per_cell = max(1, int(math.ceil(scene_count / 9.0)))
    for illumination in ILLUMINATIONS:
        for density in DENSITIES:
            for _ in range(per_cell):
                if len(plan) >= scene_count:
                    break
                plan.append((density, illumination))
    while len(plan) < scene_count:
        plan.append(('medium', 'normal'))
    for empty_index in range(max(0, int(empty_count))):
        plan.append(('empty', ILLUMINATIONS[empty_index % len(
            ILLUMINATIONS)]))
    return plan


def generate_dataset(out_dir, scene_count=36, empty_count=3, seed=20261126,
                     width=640, height=480, created_at=None):
    """Generate a full synthetic dataset under ``out_dir``.

    The function is deterministic for a fixed ``(scene_count, empty_count,
    seed, width, height)`` tuple. It returns the dataset manifest path.
    """
    scene_count = int(scene_count)
    if scene_count < 0:
        raise ValueError('scene_count must be non-negative')
    if int(width) < 64 or int(height) < 64:
        raise ValueError('image dimensions must be at least 64 pixels')
    os.makedirs(os.path.join(out_dir, 'images'), exist_ok=True)
    os.makedirs(os.path.join(out_dir, 'masks'), exist_ok=True)

    plan = _scene_plan(scene_count, empty_count)
    scenes = []
    for index, (density, illumination) in enumerate(plan):
        scene_id = 'syn-%d-%03d' % (int(seed), index)
        image, mask, record = generate_scene(
            scene_id, seed, index, int(width), int(height), density,
            illumination)
        image_path = os.path.join(out_dir, record['image'])
        mask_path = os.path.join(out_dir, record['mask'])
        if not cv2.imwrite(image_path, image):
            raise RuntimeError('failed to write %s' % image_path)
        if not cv2.imwrite(mask_path, mask.astype(np.uint16)):
            raise RuntimeError('failed to write %s' % mask_path)
        record['image_sha256'] = sha256_file(image_path)
        record['mask_sha256'] = sha256_file(mask_path)
        record['attributes']['image_sha256'] = record['image_sha256']
        record['attributes']['mask_sha256'] = record['mask_sha256']
        scenes.append(record)

    if created_at is None:
        created_at = datetime.datetime.now(
            datetime.timezone.utc).replace(microsecond=0).isoformat()
    manifest = {
        'schema_version': DATASET_SCHEMA_VERSION,
        'dataset_id': 'synthetic_%d' % int(seed),
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
            'empty_count': int(empty_count),
            'width': int(width),
            'height': int(height),
        },
        'category_names': ['ore'],
        'scenes': scenes,
    }
    manifest_path = os.path.join(out_dir, 'dataset.json')
    with open(manifest_path, 'w', encoding='utf-8') as handle:
        json.dump(manifest, handle, indent=2, sort_keys=True)
        handle.write('\n')
    _write_dataset_card(out_dir, manifest)
    return manifest_path


def _write_dataset_card(out_dir, manifest):
    """Write a human-readable provenance card next to the manifest."""
    path = os.path.join(out_dir, 'DATASET_CARD.md')
    lines = [
        '# Synthetic ore scene dataset',
        '',
        '- schema: `%s`' % manifest['schema_version'],
        '- dataset_id: `%s`' % manifest['dataset_id'],
        '- source: **synthetic** (procedurally generated, no real ore)',
        '- annotation: **procedural ground truth**, pixel-accurate by '
        'construction',
        '- license of generated data: **CC0-1.0**',
        '- generator: `%s` %s' % (
            manifest['generator']['name'], manifest['generator']['version']),
        '- scenes: %d (including %d empty)' % (
            len(manifest['scenes']), manifest['generator']['empty_count']),
        '- seed: %d' % manifest['generator']['seed'],
        '',
        '## Limits',
        '',
        'This dataset measures whether the offline pipeline and evaluation code',
        'behave as specified. It says nothing about accuracy on real ore, real',
        'conveyor lighting, dust, water, scale variation or mineral appearance.',
        'Do not report synthetic metrics as real-ore validation.',
        '',
    ]
    with open(path, 'w', encoding='utf-8') as handle:
        handle.write('\n'.join(lines))
