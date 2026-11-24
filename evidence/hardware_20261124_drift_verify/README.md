# 2026-11-24 — hardware static validation of the per-view bias / drift model

**Hardware claimed: yes, static only.** The arm was not commanded; the scene was
left as found (cup, dining table, kite tag on the table). The point of the run
was to check, on the real Pi 5 + D435i + Aurora + laptop GPU path, that the new
per-view bias model keeps a **static** target from drifting and that the
learned offsets match the recorded cross-view differences.

Deployed commit: `eed1ca9` (`feat(fusion): per-view bias model and static-track
drift fix`), `git archive` → container, `colcon build` on the Pi:
`Summary: 3 packages finished [10.3s]`. Edge parameters:
`fusion_config:=.../fusion_dual_hardware.yaml` (`coalesce_ms: 600`),
`view_bias_config:=.../view_bias.yaml`; wrist/ext TF served by the existing
`wrist_tf`/`ext_tf` processes.

## Environment

| item | value |
|---|---|
| Pi | up 3 h 36 min at the end, load 3.6–4.2, container memory 1.4/6 GB |
| cameras | D435i Bus 02 5000M; external colour 6.95 Hz, aligned depth present; wrist colour 4.4 Hz after the runs |
| TF | wrist optical (0.180, 0.010, 0.192) m; external optical (0.139, 0.027, 0.037) m |
| GPU | laptop RTX 3060, `yolov8s.pt`, restarted 18:16 and loaded; no traceback |
| services | `armpi-rs-depth` active, `armpi-vision` active, `armpi-ext-cam` **stopped** (was in a restart loop; it is `disabled` at boot) |
| edge rates | ~3.3–3.8 fps external / ~2.7–3.7 fps wrist; GPU alive during both runs |

## Runs

Both runs used the same command, bounded with `timeout -s INT`; the laptop
recorded `/perception/observations/*`, `/perception/targets` and
`/perception/status` with `/tmp/record_drift.py`.

| run | edge window | observations (ext/wrist) | frames / detections / matched | wrist depth failures |
|---|---|---:|---:|---:|
| 1 | 90 s | 125 (100 / 25) | 81 / 211 / 125 | 14 |
| 2 | 60 s | 306 (295 / 11) | 357 / 1060 / 306 | **172** |

Run 1's edge-side camera readers stopped delivering at **t ≈ 23.7 s** while the
edge process kept publishing status to the 90 s timeout; the camera drivers
were still publishing after the run (6.95 / 4.4 Hz) and no USB/xhci error
appeared. Run 2 ran the full 60 s (frames 22 → 93 → 154 → 357), so the stop was
not reproduced. This looks like the known cross-distro (Humble edge ↔ Jazzy
GPU) DDS reader wedge, not a camera failure; it needs its own investigation.

## Static drift result (settled metric: median after a 6 s settling window)

Recorded = the live `/perception/targets` samples; replay = the same observations
run offline through the current tracker (`scripts/replay_static_drift.py
--source-bias none`, because the edge already applied `config/view_bias.yaml`
to `/perception/observations`; applying it twice teaches a wrong per-track
offset). The two agree to 0.00 mm (run 1) / 0.22 mm (run 2) on the worst label,
so the offline model is a faithful stand-in for the live edge. The replay also
includes `velocity_min_dt_s = 0.10`, added after these live runs to reject
near-duplicate capture stamps; the live recorded values below are pre-fix and
still inside the 20 mm gate.

| run | label | worst recorded | worst replay | tracks |
|---|---|---:|---:|---|
| 1 | cup | 2.73 mm | 2.69 mm | cup-006 |
| 1 | dining table | 4.63 mm | 4.63 mm | dining_table-002 |
| 1 | kite | 0.16 mm | 0.16 mm | kite-004 |
| 2 | cup | 0.31 mm | 0.44 mm | cup-010 |
| 2 | dining table | 8.13 mm | 7.91 mm | dining_table-002 |
| 2 | kite | 1.61 mm | 1.59 mm | kite-013 |

**Worst required label: 4.63 mm (run 1) and 7.91 mm (run 2) in replay (live 4.63 / 8.13)**, versus the
pre-fix recorded drift of 696 mm (cup) / 448 mm (dining table) in run 7. The
8 mm in run 2 is the dining-table bounding box moving between detections, not
TF drift; the cup (0.31 mm) and kite (1.6 mm) show the floor.

## The one expected jump: first cross-view pair

Run 1's live cup track starts at the external reference
(335.9, −19.5, −29.7) mm, then steps to the wrist reference
(392.8, −31.6, −63.6) mm at t = 12.32 s — a **67.2 mm one-time step** when the
first time-matched wrist sample initialises `Track.view_offset`. This is the
object's measurement-point difference, not drift: the track was external-only
until that pair, and the model then re-references it to the wrist view. After
the step the cup stays within 2.7 mm. A planner must either tolerate this
reference change or the track should not be published until the reference view
has been seen (a policy decision, not implemented here).

## Cross-view offsets measured on hardware

Per-view means over each run (positions are already chain-corrected by
`config/view_bias.yaml`); external minus wrist:

| label | run 1 | run 2 |
|---|---|---|
| cup | (−56.9, +11.6, +34.3) mm | (−56.8, +12.3, +34.1) mm |
| dining table | (+29.2, +21.2, −3.9) mm | (+28.0, +21.9, −4.3) mm |
| kite | (−16.0, −74.1, −5.6) mm | (−14.6, −64.3, −5.8) mm |

The cup value matches the run-7 recorded difference (−66.8, +13.4, +24.9 mm
before the chain correction, i.e. ≈ −57, +11, +34 after it) and the replay's
learned offset. The dining table's larger run-to-run spread is its bbox moving
between detections (see the 8 mm run-2 drift).

## What this does and does not prove

**Proved:** on the real edge, with the production dual-camera config and real
YOLO, a static target no longer shows velocity-driven drift; the per-track
offset learns the measured 57 mm cup cross-view difference; the offline replay
reproduces the live output.

**Not proved / caveats:**

- No motion. The static target is the only case exercised.
- Run 2's wrist depth matching was poor (172/357 failures), so only 11 wrist
  observations and no cross-view pair in the settled window; most run-2 tracks
  are single-view. Run 1 provides the cross-view coverage.
- Absolute positions are still **per-view bbox-centre points**: the cup's
  external and wrist tracks sit 57 mm apart, and a standalone external track is
  not the object centre. Do not quote <5 mm absolute accuracy; a defined
  point/keypoint is still the next step.
- The first run's 16 s of data and the edge-side reader stop mean this is a
  short validation, not a 30-minute stability test.
- The `view_offset` learned in the offline replay of run 1 is not fully
  converged (cup −47 mm after 6 pairs, table +39 mm); the live first-pair step
  uses the full residual by design. This is expected with `view_offset_alpha =
  0.35` and a slow bbox.
- No new OOM occurred in the run window; dmesg contains an older (16:04)
  container memcg OOM and `__vm_enough_memory` messages at 18:16–18:18 from a
  python3 PID that no longer exists. Not root-caused; container memory stayed
  at 23 % of its 6 GB limit.

## Files

- `drift_run1.json`, `drift_run2.json` — raw recorder output (observations,
  targets, status);
- `replay_run1.json`, `replay_run2.json` — offline replay, settle 6 s;
- `recorded_run1.json`, `recorded_run2.json` — live targets, settle 6 s;
- `edge_drift_verify*.log`, `gpu_drift_verify.log`, `record_drift*.log` —
  process logs (the `rcutils` / `invalid data size` lines are the known
  Humble↔Jazzy CycloneDDS serialization noise, not edge errors);
- `summary.json` — machine-readable run summary;
- `hashes.txt` — SHA-256 of this bundle.

Reproduce the analysis:

```bash
python3 scripts/replay_static_drift.py --source evidence/hardware_20261124_drift_verify/drift_run1.json \
    --settle-s 6 --out evidence/hardware_20261124_drift_verify/replay_run1.json
python3 scripts/replay_static_drift.py --source evidence/hardware_20261124_drift_verify/drift_run1.json \
    --recorded-targets --settle-s 6 --out evidence/hardware_20261124_drift_verify/recorded_run1.json
# same for run2
```
