# 2026-11-25 — CycloneDDS reader-wedge investigation

## Setup

Pi 5 with the split stack (5 systemd units), vendor camera drivers, gateways
8765/8766; the laptop GPU node runs under Jazzy. Pi: CycloneDDS 0.10.4 /
`rmw_cyclonedds_cpp` 1.3.4. Laptop: CycloneDDS 0.10.5 / rmw 2.2.4.

The camera publishers offer **RELIABLE**; the ingress and the probe subscribe
**BEST_EFFORT** (`/camera/color/image_raw`, aligned depth, wrist RGB/depth).

## Probe: a separate participant sees the same reader wedge

`dds_probe.py` was a second, lightweight participant (best-effort, depth 1)
subscribing to the four camera topics without decoding; it printed totals every
5 s (`dds_probe.log`). During the window:

- external color: 20, 32, 59, 66, 76, 91, 113, 131, 144, 149, then **frozen**
  (149) for the remaining 25 s;
- external aligned depth: 38, 66, 78, 107, 121, 140, 142, then **frozen** (142);
- wrist RGB and wrist depth kept increasing (364 -> 451, 306 -> 381).

The two ext readers in that fresh participant stopped while the wrist readers
in the same participant continued. After the event a new `ros2 topic hz`
participant received `camera/color/image_raw` at 4.05 Hz and the aligned depth
at 9.43 Hz; `armpi-rs-depth` was active for 2 h 20 min with `NRestarts=0` and
no camera errors. **The driver/writer is healthy; the stall is reader-level**.

## Host load

`cpu_snapshot.txt` (15:39): load 9.8-12.2, `vmstat` 72 % user / 13 % idle on
four cores. Top CPU: `realsense2_camera_node` 77 %, external ingress 48 %,
Aurora 38 %, wrist ingress 35-43 %, fusion 26 %, gateways 42 % combined,
vendor `ros_robot_controller`/`joint_state_publisher`/`kinematics` ~28 %,
`/home/pi/hiwonder-toolbox/find_device.py` 37.5 %. No apt-get, no throttling
(57-58 C, `get_throttled=0x0`). Memory is not exhausted: MemAvailable 6.1 GB,
SwapFree 2.1 GB; `Committed_AS` 8.0 GB > `CommitLimit` 6.3 GB under the
heuristic mode (`vm.overcommit_memory=0`).

A separate anomaly: 109 `__vm_enough_memory` messages, **all from one exited
PID 5962 (python3)**, last at 15:24. The ext/wrist stalls continue after
15:24, so this is not the current wedge cause; the process should be
identified if it recurs.

## DDS serialization errors

92 `invalid data size` / `string data is not null-terminated` messages in the
Humble ingress logs over 2 h, also in the realsense node and `wrist_tf`; they
come from the Jazzy GPU node's cross-distro traffic. The external wedge
predates the GPU restart (stalls at 14:15-14:36 before the GPU came back at
14:39/14:54), so these errors are a separate issue, not the wedge cause.

## Conclusion

The ext/wrist stalls are reader-level CycloneDDS stalls: a fresh participant
recovers immediately, the camera driver keeps publishing, and a second
participant suffers the same ext reader freeze. They correlate with CPU
saturation (camera drivers + two ingresses + gateways + vendor nodes; 13 %
idle), not with the driver, memory, or the cross-distro GPU. The split ingress
contains the blast radius; the 10/300 restart budget is a symptom-level alarm.

## Recommended experiments / fixes

1. Reduce load: D435i color/depth 15 -> 6 fps (or lower resolution) and ingress
   `max_uplink_fps` 15/8 -> 4/2; expected to lower the realsense/ingress CPU
   substantially.
2. During perception runs, stop the motion gateway 8766
   (`armpi-gateway-pose`): ~22 % CPU and the wrist pointcloud; it is not needed
   for perception (operator approval required).
3. CycloneDDS tuning: add `SocketReceiveBufferSize` / `Internal/Watermarks`
   to `/home/ubuntu/cyclonedds.xml` (and test `lo` for local traffic or
   thread priorities); restart the participants and A/B the stall rate.
4. Camera publishers are RELIABLE while readers are BEST_EFFORT; set
   sensor-data best-effort on the camera publishers if the drivers expose
   `qos_overrides`, reducing writer overhead.
5. Run the GPU node on Humble (same distro) to remove the 92 cross-distro
   serialization errors; this is separate from the wedge.
6. The ingress restart remains the recovery; a driver-level watchdog is not
   indicated (the driver is healthy).

Files: `dds_probe.log`, `cpu_snapshot.txt`.
