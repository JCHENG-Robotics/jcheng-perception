# 2026-11-24 — cross-view bias model and static-track drift regression

**Hardware claimed: no new robot run.** This is an offline re-analysis of the
observations recorded during the 2026-11-24 dual-view run; the "pre-fix"
baselines are the targets that run actually published. Model description:
[`docs/FUSION.md` §9](../../docs/FUSION.md). Code: `view_bias.py`,
`tracking.py`, `kalman.py`, `config/view_bias.yaml`.

## Why this analysis

Run 7 fused both views on hardware, but a **static** cup drifts hundreds of
millimetres on the published `FusedTarget` (recorded: cup-005 696 mm,
dining_table-035 448 mm). The fusion weights and `source_cameras` were correct,
so the bug was in the tracking model, not in the data path. The recorded
observations are rich enough to reproduce and fix it offline.

## Data and method

Source: `evidence/hardware_20261124_dualview/dual_verify7.json` —
223 observations (both views, real YOLO, capture stamp preserved) and 1018
published target samples, 36 s, arm stationary.

- Replay rebuilds each `TargetObservation` from the recorded
  `camera/label/position/depth/score/valid_depth_fraction/t/stamp`. The
  measurement covariance is rebuilt from `config/cameras.yaml` because the log
  does not carry it; everything else is recorded data.
- Detections are coalesced by receive time (`coalesce_ms: 600`, the run-7
  hardware config) and targets are published at 30 Hz.
- Three measurements are compared:
  - **recorded** — target samples the pre-fix system actually published
    (`--recorded-targets`);
  - **legacy replay** — the pre-fix tracker logic re-run on the observations
    (`--mode legacy`; no bias model, full-gain cross-view updates, 1 m/s
    velocity prior, 1 s extrapolation);
  - **fixed replay** — current tracker (`--mode fixed`).
- Metric: after a 1 s settling window, the maximum distance of a published
  position from that track's settled median. `required` labels are
  cup / dining table / chair / person; `kite` is excluded (below).

## What the data shows (the drift mechanism)

Per-view positions are stable across the whole run:

| label | view | samples | per-axis σ | range |
|---|---|---:|---|---|
| cup | external | 35 | 0.2 / 0.1 / 0.1 mm | 0.9 / 0.5 / 0.6 mm |
| cup | wrist | 31 | 0.4 / 1.1 / 0.5 mm | 2.0 / 3.2 / 2.1 mm |
| dining table | external | 35 | 1.5 / 0.1 / 0.4 mm | 6.2 / 0.5 / 1.7 mm |
| dining table | wrist | 31 | 0.5 / 1.0 / 0.5 mm | 2.2 / 3.8 / 2.1 mm |

But the two views do **not** measure the same point:

| label | external − wrist (x, y, z) |
|---|---|
| cup | (−66.8, +13.4, +24.9) mm |
| dining table | (+11.3, +23.3, −13.6) mm |

The external bbox centre lands on the side of the cup, the wrist bbox centre on
the rim/top; the external chain additionally carries its known ~10 mm bias.
With single-view updates alternating between two views 65 mm apart, the old
constant-velocity filter turned the alternation into velocity: replay spikes
were 7–14 m/s. `targets()` then computed `p + v·dt` with
`dt = now − last_state` clamped at 1.0 s, so a 0.1 m/s velocity error became a
0.1 m position error, and larger spikes became the observed 0.5 m excursions.

## Model applied

1. **Chain-level per-view correction** (`config/view_bias.yaml`, applied in
   `PerceptionPipeline._build_observations`): external correction
   (+9.9, −1.8, +9.4) mm with 6 mm 1σ, from the run-3 tag cross-view and the
   table-plane check. Covariance is inflated by that σ; the wrist is the
   reference view. Only the relative offset is identifiable.
2. **Per-track residual view offset** (`Track.view_offset`): the remaining
   external-minus-wrist difference is learned online from time-matched pairs,
   including cross-batch pairing against the track's stored per-camera sample
   (the views arrive asynchronously). First pair initialises, later pairs
   follow with `view_offset_alpha = 0.35`. Single-view external updates carry
   the remaining offset uncertainty; a fresh pair does not.
3. **Velocity only from same-view differences**: the position update uses
   `velocity_gain = 0`, and velocity is a separate measurement built from two
   successive samples of the same camera (`update_velocity`, `H = [0|I]`). The
   per-view offset cancels in that difference; a view change never does.
4. **Extrapolation discipline**: initial velocity σ 0.25 m/s (was 1.0) and
   `max_extrapolation_s = 0.35` in both association prediction and publishing.

## Result

Worst displacement per required label:

| label | recorded (pre-fix hardware) | legacy replay | fixed replay |
|---|---:|---:|---:|
| cup | 696.4 mm | 477.8 mm | **4.1 mm** |
| dining table | 447.5 mm | 454.3 mm | **4.9 mm** |
| chair | 23.2 mm | 23.3 mm | **6.4 mm** |
| person | 10.5 mm | 10.5 mm | **4.4 mm** |
| **required worst** | **696.4 mm** | **477.8 mm** | **6.4 mm** |

The drift also fragmented tracks: the recorded run produced 5 cup and 4
dining-table tracks; the fixed replay settles to one of each.

The learned per-track offsets match the offline per-label statistics, which is
the strongest check that the model is estimating the intended quantity (values
after the global chain correction):

| track | learned `view_offset` | hits | max disp |
|---|---|---:|---:|
| cup-005 | (−57.2, +10.9, +34.0) mm | 52 | 4.1 mm |
| dining_table-002 | (+22.1, +21.1, −5.0) mm | 54 | 4.9 mm |

`kite` is not part of the gate: the wrist view's kite observations have
σ = 32.6/30.7 mm and the two views sit 81 mm apart in y, i.e. the label is
switching between two physical objects / regions. That is an association and
dataset issue, not a bias the tracker can fix.

## Limitations

- **Absolute accuracy is still wrist-referenced.** Only relative per-view
  offsets are observable; the fused position is the wrist view's bbox-centre
  point corrected to a stable static estimate. A defined point (AprilTag or a
  grasp keypoint) is still needed to quote <5 mm absolute accuracy.
- The 6 mm external chain correction is **provisional**; it must be re-derived
  from the deployed hand-eye and the adopted (not provisional) external
  extrinsic.
- The per-track offset assumes a static/slow target. Motion between the two
  captures is partly absorbed into the offset; fast motion needs the defined
  point/keypoint path and a re-tuned `view_offset_alpha`.
- This is offline evidence. The code has not been deployed to the Pi 5 or
  re-run on hardware; no motion commands were issued.
- `max_extrapolation_s = 0.35` bounds the published lag compensation. At the
  current ~1–2 fps detection rate the target holds its position between
  updates rather than extrapolating a noisy velocity; verify this against the
  planner's tolerance before motion integration.

## Files

- `replay_fixed.json` — current tracker on the recorded observations;
- `replay_legacy.json` — pre-fix tracker baseline;
- `recorded_targets.json` — metrics of the targets run 7 actually published;
- `hashes.txt` — SHA-256 of the three reports, `summary.json` and this README.

Reproduce:

```bash
python3 scripts/replay_static_drift.py \
  --out evidence/analysis_20261124_drift_bias/replay_fixed.json
python3 scripts/replay_static_drift.py --mode legacy \
  --out evidence/analysis_20261124_drift_bias/replay_legacy.json
python3 scripts/replay_static_drift.py --recorded-targets \
  --out evidence/analysis_20261124_drift_bias/recorded_targets.json
```
