# ADR 0001 — Edge/GPU split for perception

- Status: accepted
- Date: 2026-11-23

## Context

The ArmPi Ultra carries a Raspberry Pi 5 (8 GB) and two cameras: a USB3 D435i
on the chassis and a USB2 Aurora 930 on the wrist. YOLO inference competes with
camera capture, depth decoding, TF and the motion stack for CPU and memory
bandwidth. Model iteration on the robot would require restarts and could disturb
control.

## Decision

- Pi 5 runs capture, JPEG uplink, depth/TF rings, back-projection, association,
  distance-weighted fusion and Kalman tracking. It runs **no detector**.
- The laptop GPU runs one asynchronous detector per camera with bounded queues,
  hot reload and no access to depth/TF/cameras.
- Images cross the network as JPEG `CompressedImage` with the acquisition stamp;
  detections return with the same stamp.
- Periodic state streams are best-effort and the edge uses a single-threaded
  executor plus dedicated uplink worker threads.

## Consequences

Positive:

- The motion/control path never waits on inference.
- Model iteration is a GPU-side hot reload; the robot node is untouched.
- GPU failure degrades to coasting/lost tracks instead of blocking capture.

Negative:

- Network becomes part of the latency budget.
- Two time bases exist for logs; chrony is recommended (matching itself only
  needs the Pi 5 clock because the GPU preserves the stamp).
- The GPU node must be deployed and monitored separately.

## Alternatives considered

- All inference on Pi 5: rejected (CPU/thermal/jitter; blocks control).
- All perception on the laptop: rejected (USB camera ownership, network
  round-trip for depth/TF, robot loses perception when the link drops).
- Cloud inference: rejected (latency, availability, data policy).
