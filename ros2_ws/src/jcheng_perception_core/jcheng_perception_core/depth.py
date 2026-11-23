"""
Robust depth sampling and range-dependent depth uncertainty.

A YOLO box is a 2-D hypothesis. The depth image is the measurement that turns
it into a 3-D point, but depth sensors produce holes, mixed pixels and
range-dependent noise. This module keeps the sampling policy explicit so the
fusion stage can weight the two views by their actual uncertainty.
"""

import math

import numpy as np

from .types import DepthFrame

DEFAULT_MIN_DEPTH_M = 0.05
DEFAULT_MAX_DEPTH_M = 10.0


def sample_depth(depth_frame, u, v, radius=2, min_valid=1,
                 min_depth_m=DEFAULT_MIN_DEPTH_M,
                 max_depth_m=DEFAULT_MAX_DEPTH_M):
    """
    Return ``(depth_m, valid_fraction)`` at a pixel.

    The median of the valid patch suppresses mixed pixels at object edges. A
    missing patch returns ``None``; callers must drop the detection rather than
    invent a depth.
    """
    patch = depth_frame.patch_metres(u, v, radius)
    if patch is None:
        return None
    valid = patch[np.isfinite(patch)]
    valid = valid[(valid >= min_depth_m) & (valid <= max_depth_m)]
    if valid.size < max(1, int(min_valid)):
        return None
    return float(np.median(valid)), float(valid.size) / float(patch.size)


def sample_bbox_depth(depth_frame, bbox_xyxy, radius=2,
                      min_valid=2,
                      min_depth_m=DEFAULT_MIN_DEPTH_M,
                      max_depth_m=DEFAULT_MAX_DEPTH_M):
    """
    Sample depth for a bounding box centre with a grid fallback.

    The centre patch is preferred because a detector box usually centres on the
    object. When the centre is a depth hole (common on shiny or thin objects),
    a 3x3 grid over the inner half of the box supplies a median instead. The
    returned pixel is the sample the depth came from.
    """
    x0, y0, x1, y1 = [float(value) for value in bbox_xyxy]
    if x1 <= x0 or y1 <= y0:
        return None
    centre_u = 0.5 * (x0 + x1)
    centre_v = 0.5 * (y0 + y1)

    direct = sample_depth(
        depth_frame, centre_u, centre_v, radius=radius,
        min_valid=min_valid, min_depth_m=min_depth_m,
        max_depth_m=max_depth_m)
    if direct is not None:
        return {
            'depth_m': direct[0],
            'valid_fraction': direct[1],
            'pixel': (centre_u, centre_v),
            'source': 'centre',
        }

    inner_x0 = x0 + 0.25 * (x1 - x0)
    inner_x1 = x1 - 0.25 * (x1 - x0)
    inner_y0 = y0 + 0.25 * (y1 - y0)
    inner_y1 = y1 - 0.25 * (y1 - y0)
    samples = []
    for fraction_x in (0.25, 0.5, 0.75):
        for fraction_y in (0.25, 0.5, 0.75):
            u = inner_x0 + fraction_x * (inner_x1 - inner_x0)
            v = inner_y0 + fraction_y * (inner_y1 - inner_y0)
            sample = sample_depth(
                depth_frame, u, v, radius=1, min_valid=1,
                min_depth_m=min_depth_m, max_depth_m=max_depth_m)
            if sample is not None:
                samples.append((sample[0], u, v, sample[1]))
    if not samples:
        return None
    values = np.array([sample[0] for sample in samples], dtype=np.float64)
    median = float(np.median(values))
    index = int(np.argmin(np.abs(values - median)))
    return {
        'depth_m': median,
        'valid_fraction': float(np.mean(
            [sample[3] for sample in samples])),
        'pixel': (samples[index][1], samples[index][2]),
        'source': 'grid',
    }


def depth_sigma_m(depth_m, quadratic_m_per_m2, floor_m):
    """
    Return the 1-sigma depth noise in metres.

    Stereo depth error grows with the square of range. The quadratic
    coefficient is calibrated per camera (or estimated from the disparity
    noise); the floor covers quantisation and sensor noise.
    """
    z = max(0.0, float(depth_m))
    quad = float(quadratic_m_per_m2) * z * z
    floor = float(floor_m)
    return math.sqrt(quad * quad + floor * floor)


def lateral_sigma_m(depth_m, focal_px, pixel_sigma_px):
    """Convert a pixel uncertainty to a metric lateral uncertainty."""
    if focal_px <= 0.0:
        raise ValueError('focal length must be positive')
    return abs(float(depth_m)) * float(pixel_sigma_px) / float(focal_px)


def depth_valid_fraction(depth_frame, bbox_xyxy, stride=3,
                         min_depth_m=DEFAULT_MIN_DEPTH_M,
                         max_depth_m=DEFAULT_MAX_DEPTH_M):
    """Return the fraction of valid depth pixels inside a box."""
    x0, y0, x1, y1 = [float(value) for value in bbox_xyxy]
    height, width = depth_frame.data.shape
    x0 = int(max(0, min(width - 1, round(x0))))
    x1 = int(max(x0 + 1, min(width, round(x1))))
    y0 = int(max(0, min(height - 1, round(y0))))
    y1 = int(max(y0 + 1, min(height, round(y1))))
    step = max(1, int(stride))
    patch = depth_frame.data[y0:y1:step, x0:x1:step]
    if patch.size == 0:
        return 0.0
    if depth_frame.metric:
        metres = np.asarray(patch, dtype=np.float64)
    else:
        metres = np.asarray(patch, dtype=np.float64) * float(
            depth_frame.scale_m)
    valid = np.isfinite(metres) & (metres >= min_depth_m) & (
        metres <= max_depth_m)
    return float(np.count_nonzero(valid)) / float(valid.size)


def border_occlusion_ratio(bbox_xyxy, width, height, margin_px=2):
    """Return how many box sides touch the image border (0..1)."""
    x0, y0, x1, y1 = [float(value) for value in bbox_xyxy]
    touches = 0
    if x0 <= margin_px:
        touches += 1
    if y0 <= margin_px:
        touches += 1
    if x1 >= float(width) - margin_px:
        touches += 1
    if y1 >= float(height) - margin_px:
        touches += 1
    return touches / 4.0
