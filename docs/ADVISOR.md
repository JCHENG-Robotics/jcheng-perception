# SAM3 + Qwen advisory detector client

Status: **offline unit-tested + two-frame live HTTP smoke; not deployed in the
perception pipeline.**

`jcheng_perception_gpu` can use the AGX Orin advisory service as a detector
backend (`backend: sam3_advisor`). The service runs SAM 3.0 with the fixed
`T1 + SAM3 score >= 0.8` policy and returns one box after the Qwen judge. The
laptop GPU host sends the captured frame over HTTP and publishes the result as
an ordinary `Detection2DArray`; it does not join a new DDS domain, open a
device, read depth/TF, or command motion.

## Data flow

1. The existing per-camera `CameraWorker` receives a `CompressedImage` and
   drops the oldest frame when its bounded queue is full (`queue_size: 2`).
2. The worker decodes the frame and calls `AdvisorDetector.detect()`.
3. The detector re-encodes the frame as JPEG, base64-encodes it, and POSTs
   `{image_b64, image_format, prompt}` to the Orin endpoint with a timeout.
4. The service returns `gate` = `JUDGE`, `SKIP` or `NO_MASK`.
   - `JUDGE` -> one `Detection` with label `sam3:<prompt>`, the SAM3 score and
     the box, in the original image coordinates.
   - `SKIP` / `NO_MASK` -> an empty list (a successful result, not an error).
5. The worker publishes the array with the original capture timestamp.

The mask RLE returned by the service is **not** published: `Detection2D` has no
mask field, and adding one is a `jcheng-interfaces` contract change.

## Configuration

Copy `config/gpu_advisor.example.yaml` over `config/gpu.yaml` (or merge one
camera entry). A minimal entry:

```yaml
cameras:
  d435i_ext:
    input_topic: /perception/uplink/d435i_ext/image_compressed
    detections_topic: /perception/detections/d435i_ext
    debug_topic: /perception/debug/d435i_ext
    hot_reload: false
    model:
      backend: sam3_advisor
      endpoint: http://192.168.5.31:8090/infer
      prompt: bottle
      timeout_s: 10.0
      jpeg_quality: 90
      min_score: 0.0
```

`hot_reload` is inert for this backend (there is no local model file).
`min_score` is an additional client-side floor; the service already applies
`score >= 0.8`.

## Failure behavior

- Transport timeout, connection refusal, non-JSON, `ok != true`, an unknown
  gate, a missing/short/non-finite/inverted box: `DetectorError`.
- The camera worker counts the frame in `frames_failed`, logs a throttled
  warning, publishes nothing for that frame, and continues. There is no retry
  and no fabricated box.
- A stopped service therefore degrades that camera to zero detections while the
  node and the other camera keep running. There is no automatic fallback to
  YOLO; revert the camera entry or run a shadow YOLO model if both are needed.
- Default timeout is 10 s. Measured advisory latency over the laptop -> Orin
  Wi-Fi path was p50 2.15 s / p95 3.74 s / max 4.03 s, so 10 s leaves headroom
  but a wedged request still occupies the camera worker for that long.
- The service serializes inference behind one lock. With two cameras calling at
  once, one request waits for the other; the frame queue, not the network,
  absorbs the backlog.

## Boundaries

- Advisory only: no `ExecuteMotion`, no lease, no vendor topic, no device
  access, no new DDS domain.
- The Orin is on `192.168.5.x`; the robot is on `10.42.0.x`. The laptop relays;
  a direct Orin -> robot connection is not assumed.
- The prototype was validated with a fake transport in
  `tests/test_detectors.py` (`test_advisor_*`). A live smoke test needs the
  Orin service, a camera stream and a separately authorized hardware session.

## Validation record

- Offline: `tests/test_detectors.py` (`test_advisor_*`) uses a fake transport
  and covers the JUDGE box, SKIP/NO_MASK empties, malformed replies, transport
  errors, `min_score`, config validation and factory construction.
- Live HTTP smoke, 2026-11-26: the real `AdvisorDetector` on the laptop sent
  two recorded 640x480 frames as base64 JPEG to the Orin service (temporarily
  bound to `0.0.0.0`, stopped afterwards):
  - front frame, prompt `green soda bottle`: one box, score **0.961**,
    bbox (28, 151, 117, 326), **3.39 s**.
  - wrist frame, prompt `bottle`: one box, score **0.973**, bbox
    (150, 66, 215, 280), **2.24 s**.
- Not yet validated: camera/DDS integration in the running perception stack,
  two-camera concurrency in the production launch, failure injection against a
  stopped service, and any hardware session. Deployment is **Unverified**.

Research measurements, dataset and service soak results live outside this
repository (workspace `jcheng-research`, report
`reports/20261126-agx-orin-sam3-qwen-deployment.md`). This repository only
records that the client compiles, validates replies and fails closed.
