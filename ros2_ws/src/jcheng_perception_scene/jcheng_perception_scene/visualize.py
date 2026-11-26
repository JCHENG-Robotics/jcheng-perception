"""Deterministic mask overlay rendering for offline inspection."""

import cv2
import numpy as np

_PALETTE = (
    (56, 56, 220),
    (60, 180, 75),
    (230, 160, 60),
    (200, 70, 200),
    (60, 200, 220),
    (220, 220, 70),
    (120, 80, 220),
    (80, 220, 150),
)


def _colour(index):
    return _PALETTE[index % len(_PALETTE)]


def overlay_instances(image_bgr, instances):
    """Return a copy of ``image_bgr`` with masks, contours and labels."""
    canvas = image_bgr.copy()
    for index, instance in enumerate(instances):
        x0, y0, x1, y1 = instance.bbox_xyxy
        roi = canvas[y0:y1 + 1, x0:x1 + 1]
        mask = instance.mask[y0:y1 + 1, x0:x1 + 1]
        colour = _colour(index)
        tint = np.zeros_like(roi)
        tint[mask] = colour
        roi[:] = cv2.addWeighted(roi, 0.62, tint, 0.38, 0.0)
        contours, _ = cv2.findContours(
            mask.astype(np.uint8), cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours(roi, contours, -1, colour, 1)
        x = int(instance.visible_centroid_px[0])
        y = int(instance.visible_centroid_px[1])
        label = instance.instance_id.split(':')[-1]
        cv2.putText(
            canvas, '%s %.2f' % (label, instance.score), (x - 8, y),
            cv2.FONT_HERSHEY_SIMPLEX, 0.35, (255, 255, 255), 1,
            cv2.LINE_AA)
    if not instances:
        cv2.putText(
            canvas, '0 instances', (10, 22), cv2.FONT_HERSHEY_SIMPLEX,
            0.55, (255, 255, 255), 1, cv2.LINE_AA)
    return canvas


def render_scene(image_bgr, instances, output_path):
    """Write one overlay image and return the path."""
    overlay = overlay_instances(image_bgr, instances)
    if not cv2.imwrite(output_path, overlay):
        raise RuntimeError('failed to write overlay: %s' % output_path)
    return output_path
