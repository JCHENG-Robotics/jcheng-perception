# 2026-11-24 — split per-camera ingress / fusion validation

**Hardware claimed: yes, static only.** Arm stationary; 60 mm AprilTag 36h11
id 0 on the table. Purpose: isolate the camera DDS readers so a wedged Aurora
depth (or D435i colour) reader cannot take down the other view or the fusion
state, and recover the ingress automatically.

## Architecture

`split_edge.launch.py` starts three processes in the Pi container:

- `perception_ingress_ext` — `perception_edge` with `camera_filter:=d435i_ext`.
  It owns that camera's colour/depth/CameraInfo subscriptions, depth/TF
  matching, back-projection, JPEG uplink and publishes ``TargetObservation`` on
  ``/perception/observations/d435i_ext``. `publish_targets`/`publish_status`
  are off.
- `perception_ingress_wrist` — same for `aurora_wrist`.
- `perception_fusion` — subscribes to the two small observation topics, runs
  association/Kalman/fusion, and publishes ``/perception/targets`` and
  ``/perception/status``.

Each ingress has a 4 s link watchdog (`link_stalled` on camera frame/depth
counters). When a reader stops it logs and exits; launch respawns that process
after 2 s, so a fresh DDS participant is created without touching the other
ingress or fusion. In split mode the status counters are observation counts
and per-camera observation rates (the raw camera counters live in the ingress
logs).

## Deployment trap found during validation

`colcon build` skipped reinstalling `tracking.py`/`config.py` because the
extracted source mtime was older than the installed file. The fusion therefore
ran a stale tracker without `velocity_min_dt_s`, and repeated near-duplicate
captures produced velocities of 1e8–1e9 m/s and metre-scale targets. Forcing a
reinstall (`touch` every core/edge source, then rebuild) fixed it. **Always
verify the installed files, not just the source tree**, after deploying an
archive. As a second line of defence the tracker now has
`max_track_speed_mps: 0.5` / `max_target_distance_m: 1.0` (out-of-bounds
tracks are re-anchored; out-of-bounds targets are not published).

## Final run (run 4)

- external 134 + wrist 226 tag observations, both 0.1–60.0 s continuous;
- **cross-view 1.55 mm** (external − wrist = +1.45/−0.53/−0.14 mm);
- fusion: one track, 1691 samples, **max 2.14 mm, p95 1.64 mm, final
  1.96 mm** (recorded-target report: 2.0/1.6/2.0); offline replay 5.0/3.4/0.5;
- edge status 353 frames / 401 observation samples, `gpu_alive=true`,
  active/confirmed tracks 1;
- the **external ingress watchdog fired at 25 s** (`frames=139, depth=110`)
  and was respawned at 28.5 s; the fusion track stayed alive on the wrist view
  and the external observations resumed. The split retained one track across
  the restart.

Earlier split runs 1–3 are retained in the bundle: run 1 exposed the stale-core
explosion, runs 2–3 were the iterative bounds checks.

## Files

- `split_run4.json` — successful recording (observations, targets, status);
- `split_run3.json` — pre-final bounds run;
- `split_ingress_run.log` — split launch log (ingress watchdog/respawn);
- `humble_gpu_run.log` — Humble tag GPU log;
- `replay_split_run4.json`, `recorded_split_run4.json`;
- `summary.json`, `hashes.txt`.

Run it with the TF processes already up:

```bash
ros2 launch jcheng_perception_edge split_edge.launch.py start_tf:=false \
  cameras_config:=<share>/config/cameras.yaml \
  fusion_config:=<share>/config/fusion_dual_hardware.yaml \
  frames_config:=<share>/config/frames.yaml \
  view_bias_config:=<share>/config/view_bias.yaml
```

The single-process `perception_edge` remains available for the legacy mode.
