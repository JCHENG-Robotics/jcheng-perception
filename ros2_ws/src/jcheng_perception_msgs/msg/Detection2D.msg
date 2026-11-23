# One 2-D detection in the original image coordinate system.
# The GPU node scales boxes back from the letterboxed network input before
# publishing, so consumers can use them directly with the depth image.

string label
uint32 class_id
float32 score
# x1, y1, x2, y2 in pixels
float32[4] bbox_xyxy
# Flattened x, y, confidence triples. Empty for a box-only model.
float32[] keypoints
uint32 track_hint
