"""Small, dependency-light mask operations shared by scene and metrics."""

import cv2
import numpy as np


def as_bool_mask(mask):
    """Return a boolean 2-D mask, raising ``ValueError`` on bad input."""
    array = np.asarray(mask)
    if array.ndim != 2:
        raise ValueError('mask must be 2-D')
    return array.astype(bool, copy=False)


def bbox_from_mask(mask):
    """Return an inclusive ``(x0, y0, x1, y1)`` bounding box."""
    mask = as_bool_mask(mask)
    rows, columns = np.nonzero(mask)
    if rows.size == 0:
        raise ValueError('empty mask has no bounding box')
    return (
        int(columns.min()), int(rows.min()),
        int(columns.max()), int(rows.max()),
    )


def bbox_intersects(first, second, margin=0):
    """Return True when two inclusive boxes intersect (optionally expanded)."""
    return not (
        first[2] + margin < second[0] or second[2] + margin < first[0] or
        first[3] + margin < second[1] or second[3] + margin < first[1]
    )


def _local_crop(bbox, local_mask, x0, y0, x1, y1):
    """Crop a local mask whose origin is ``bbox[:2]``."""
    offset_x = bbox[0]
    offset_y = bbox[1]
    return local_mask[
        y0 - offset_y:y1 - offset_y + 1,
        x0 - offset_x:x1 - offset_x + 1,
    ]


def iou_local(first_bbox, first_mask, second_bbox, second_mask):
    """IoU for masks stored as ``(bbox, local_mask)`` pairs."""
    if not bbox_intersects(first_bbox, second_bbox):
        return 0.0
    x0 = max(first_bbox[0], second_bbox[0])
    y0 = max(first_bbox[1], second_bbox[1])
    x1 = min(first_bbox[2], second_bbox[2])
    y1 = min(first_bbox[3], second_bbox[3])
    if x1 < x0 or y1 < y0:
        return 0.0
    first_crop = _local_crop(first_bbox, first_mask, x0, y0, x1, y1)
    second_crop = _local_crop(second_bbox, second_mask, x0, y0, x1, y1)
    intersection = int(np.count_nonzero(first_crop & second_crop))
    if intersection == 0:
        return 0.0
    first_area = int(np.count_nonzero(first_mask))
    second_area = int(np.count_nonzero(second_mask))
    union = first_area + second_area - intersection
    if union <= 0:
        return 0.0
    return intersection / float(union)


def mask_iou(first, second):
    """Return the intersection-over-union of two boolean masks."""
    first = as_bool_mask(first)
    second = as_bool_mask(second)
    if first.shape != second.shape:
        raise ValueError('mask shapes differ: %s vs %s' %
                         (first.shape, second.shape))
    intersection = int(np.count_nonzero(first & second))
    union = int(np.count_nonzero(first | second))
    if union == 0:
        return 0.0
    return intersection / float(union)


def overlap_pixels(first, second):
    """Return the number of pixels present in both masks."""
    first = as_bool_mask(first)
    second = as_bool_mask(second)
    if first.shape != second.shape:
        raise ValueError('mask shapes differ: %s vs %s' %
                         (first.shape, second.shape))
    return int(np.count_nonzero(first & second))


def assert_disjoint(masks):
    """Raise ``ValueError`` when any two masks share a pixel."""
    union = None
    for index, mask in enumerate(masks):
        mask = as_bool_mask(mask)
        if union is None:
            union = mask.copy()
            continue
        if np.any(union & mask):
            raise ValueError('masks overlap at index %d' % index)
        union |= mask
    return True


def boundary_pixels(mask):
    """Return the number of boundary pixels of a boolean mask."""
    mask = as_bool_mask(mask)
    if not np.any(mask):
        return 0
    eroded = cv2.erode(mask.astype(np.uint8), np.ones((3, 3), np.uint8))
    return int(np.count_nonzero(mask & (eroded == 0)))


def _place_in_region(bbox, local_mask, region):
    """Place a local mask into an inclusive global ``region`` array."""
    x0 = max(bbox[0], region[0])
    y0 = max(bbox[1], region[1])
    x1 = min(bbox[2], region[2])
    y1 = min(bbox[3], region[3])
    height = region[3] - region[1] + 1
    width = region[2] - region[0] + 1
    if x1 < x0 or y1 < y0:
        return None
    placed = np.zeros((height, width), dtype=bool)
    placed[
        y0 - region[1]:y1 - region[1] + 1,
        x0 - region[0]:x1 - region[0] + 1,
    ] = local_mask[
        y0 - bbox[1]:y1 - bbox[1] + 1,
        x0 - bbox[0]:x1 - bbox[0] + 1,
    ]
    return placed


def boundary_pixels_local(bbox, local_mask):
    """Boundary pixel count for a local mask with a tight bbox."""
    mask = np.asarray(local_mask).astype(bool, copy=False)
    if not np.any(mask):
        return 0
    eroded = cv2.erode(mask.astype(np.uint8), np.ones((3, 3), np.uint8))
    return int(np.count_nonzero(mask & (eroded == 0)))


def contact_pixels_local(first_bbox, first_mask, second_bbox, second_mask,
                         radius=3):
    """Contact count for two local masks, avoiding full-image allocations.

    Only second-mask pixels within ``radius`` of the first mask are counted.
    The first mask is cropped to the second bbox expanded by ``radius``; pixels
    outside that region cannot contribute, so the local dilation is exact.
    """
    radius = max(1, int(radius))
    region = (
        second_bbox[0] - radius, second_bbox[1] - radius,
        second_bbox[2] + radius, second_bbox[3] + radius,
    )
    first_region = _place_in_region(first_bbox, first_mask, region)
    second_region = _place_in_region(second_bbox, second_mask, region)
    if first_region is None or second_region is None:
        return 0
    if not np.any(first_region) or not np.any(second_region):
        return 0
    size = 2 * radius + 1
    dilated = cv2.dilate(
        first_region.astype(np.uint8),
        np.ones((size, size), np.uint8)) > 0
    return int(np.count_nonzero(dilated & second_region))


def contact_pixels(first, second, radius=3):
    """Return how many pixels of ``second`` lie within ``radius`` of ``first``.

    The dilation is applied to ``first`` only, so the result is directional but
    the caller decides how to present it.
    """
    first = as_bool_mask(first)
    second = as_bool_mask(second)
    if first.shape != second.shape:
        raise ValueError('mask shapes differ: %s vs %s' %
                         (first.shape, second.shape))
    size = 2 * int(radius) + 1
    dilated = cv2.dilate(
        first.astype(np.uint8), np.ones((size, size), np.uint8)) > 0
    return int(np.count_nonzero(dilated & second))
