# ADR 0002 — Capture timestamp is the matching key

- Status: accepted
- Date: 2026-11-23

## Context

Detections arrive 30–300 ms after capture. The wrist camera runs at ~12 Hz RGB
and ~8 Hz depth, so the newest depth frame is not necessarily the one belonging
to the detection. Using “latest depth/TF” would silently introduce centimetre
errors while the arm moves, and would make latency look like accuracy.

## Decision

- The Pi 5 driver stamp is authoritative. Every uplink and detection message
  carries it unchanged; the GPU never rewrites `header.stamp`.
- On detection arrival, the edge finds the nearest frame metadata, nearest depth
  frame and the TF sample **at the depth frame's stamp**, within per-camera
  tolerances.
- TF is linearly/slerp interpolated from a 30 Hz pose ring; a gap beyond the
  tolerance is rejected.
- If any match fails, the detection is dropped and counted. There is no
  “fall back to latest”.
- The wrist camera has no static extrinsic fallback; only the rigid D435i does.

## Consequences

Positive:

- Latency changes do not bias geometry.
- Moving wrist camera uses the pose that belongs to the depth measurement.
- Failures are visible (`depth_match_failures`, `tf_match_failures`) instead of
  silent drift.

Negative:

- Buffer memory is retained for `frame/depth/pose_buffer_seconds` (5 s).
- Clock jumps require a node restart.
- The wrist camera loses observations while TF is unavailable.

## Alternatives considered

- GPU returns “age” and edge uses latest: rejected (biases moving cameras).
- Synchronize RGB/depth with `message_filters` inside the edge: rejected for the
  uplink path (adds latency and couples capture to depth rate); depth is looked
  up only after detections return.
- Hardware trigger/sync: desirable, but not available on the current Aurora USB2
  path; the timestamp contract remains valid if it is added later.
