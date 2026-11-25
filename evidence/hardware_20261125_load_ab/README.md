# 2026-11-25 — Load-reduction A/B, aborted by a Pi wedge

## Baseline (before changes)

- 4 external + 4 wrist ingress stalls per 10 min; NRestarts 17 / 13.
- D435i profiles 640x480x15; ingress `max_uplink_fps` 15 / 12.
- load ~10-12 on four cores, ~13 % idle; realsense 77 %, ingresses 48/35-43 %,
  Aurora 38 %, gateways 42 %, vendor nodes ~28 %.

## Changes tried

1. **D435i 15 -> 6 fps**: the node started but the color stream stayed at
   0.9-4.2 Hz with `Frames didn't arrived within 5 seconds`; reverted to
   15 fps (the wake needed 5 attempts; the camera is fragile). **Not viable.**
2. **Ingress uplink 15/12 -> 4/3** (installed share + source): wrist ingress
   CPU fell from 35-43 % to 23 %, external 48 % -> 43 %; kept.
3. **Stop `armpi-gateway-pose` (8766)**: load fell to ~3 (1-min). Problem:
   `jcheng-ops`' `armpi-ops-health` restarts the gateway after three failed
   probes, so the timer had to be stopped for the A/B. This is a policy
   conflict; jcheng-ops needs a perception-mode exception.
4. **CycloneDDS tuning**: `net.core.rmem_max = rmem_default = 212992` (208 KB).
   A 4 MB `SocketReceiveBufferSize` XML was prepared and tested with a
   throwaway `ros2 topic list` participant. That coincided with load 18.9 and
   SSH banner timeouts at 15:53; the BCM2835 watchdog rebooted the Pi (uptime
   0; first real firing). The stack auto-started after the reboot, but a
   second wedge occurred during the GPU restart/status checks and the Pi was
   unreachable at the end. The tuned XML was **not** written to
   `/home/ubuntu/cyclonedds.xml`.

## What this tells us

- The Pi has almost no load headroom: the production stack plus the
  experiments exceeds it and userland wedges (6.1 GB memory was available, so
  this is not OOM); the hardware watchdog reboots.
- 6 fps is not usable on this camera without a reliable warm-up; 15 fps is the
  stable profile.
- Stopping the motion gateway is effective for load but conflicts with the
  jcheng-ops health timer.
- The reader socket buffer is capped at 208 KB; a 4 MB request is ineffective
  (and the test on the loaded stack is unsafe) until `rmem_max` is raised.
- The hardware watchdog works: it rebooted a wedged board. The vendor boot can
  move the arm; the workspace must be clear.

## Recovery

- Power-cycle the Pi if it stays unreachable; wait for the vendor bringup and
  the perception target (enabled at boot).
- Once the `enx00e04ca373c7` link is up:
  `systemctl --user reset-failed jcheng-perception-gpu && systemctl --user start
  jcheng-perception-gpu`; restart the GPU watchdog if its DDS write errors
  persist. The GPU unit failed with `rclpy error creating node` because the
  CycloneDDS interface had no carrier while the Pi was down, not because of the
  detector code.
- Do not re-run the load experiments until: `rmem_max` is raised and the
  CycloneDDS XML is tested in a quiescent state; jcheng-ops has an exception
  for the motion gateway; the 76.9 % `colord-sane` host daemon and
  `hiwonder-toolbox/find_device.py` are investigated.
- Keep 15 fps; keep the uplink 4/3 change.

## Follow-up: kernel receive buffers and the perception hold

After the power cycle, `net.core.rmem_max`/`rmem_default` were still 208 KB.
They were raised to 4 MB / 1 MB and persisted in
`/etc/sysctl.d/90-cyclonedds-rmem.conf`; the perception target and the 8765
gateway were restarted (the camera drivers were left untouched).

A 4 MB CycloneDDS `SocketReceiveBufferSize` XML was **rejected** by the 0.10.4
build in the vendor container: both ingresses failed with
`rcl node's rmw handle is invalid`, and the external ingress hit its
start-limit before the XML was reverted. Do not add that element on this
version; the kernel defaults are the only working lever here.

Results with the kernel buffers raised, ingress `max_uplink_fps` 4/3 and (from
16:15) the pose-gateway hold:

- 16:12:13-16:15: 0 ext / 0 wrist stalls (rmem + 4/3, pose still active);
  load 13-16.
- 16:15: pose gateway stopped with `/run/armpi-ops/hold-pose`; the deployed
  `armpi-ops-health` logged `skipped restart of armpi-gateway-pose: perception
  hold` and left it down. Load fell from 16.2 to ~10.7.
- 16:12:13-16:16:31: **0 ext / 0 wrist stalls, both ingresses active** (about
  4.5 min, versus the 4/4 stalls per 10 min baseline). This is a promising but
  short window, not a long-run proof.

The external start-limit alert at 16:11 was produced by the rejected XML
experiment, not by a production fault.

Next: keep the kernel buffers, 4/3 and the hold for perception runs; observe a
longer window; the camera-driver writer sockets are still the old 208 KB until
the drivers are restarted (a fragile step; do it in a quiescent window). The
8765 gateway (~35 % CPU) is the remaining optional load. Do not apply the
`SocketReceiveBufferSize` XML on CycloneDDS 0.10.4.
