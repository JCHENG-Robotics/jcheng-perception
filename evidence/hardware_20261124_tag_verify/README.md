# 2026-11-24 — defined reference point (AprilTag) validation

**Hardware claimed: yes, static only.** Arm not commanded; the 60 mm AprilTag
36h11 id 0 on the table was used as the common physical point. Goal: replace
the object-dependent bbox centre with a point that both views measure
identically, so the cross-view residual and the first-pair reference step are
millimetres instead of the cup's 57-67 mm.

Deployed edge: `eed1ca9` (no edge code changes for this mode). GPU: new `aruco`
backend, `config/gpu_tag.yaml`, labels `reference_tag_<id>`, centre keypoint
first, four corners at lower confidence. The existing edge keypoint depth path
back-projects the centre; the bbox is only a fallback.

## Runs (three attempts, 2026-11-24)

| run | result | reason |
|---|---|---|
| 1 | **0 observations** | edge did not reach `edge ready` inside the 60 s `timeout` (Pi load ~9.6 during an SD-I/O burst); the SIGINT landed in the Python import |
| 2 | **wrist only** (30 tag observations) | laptop GPU external uplink reader wedged: `frames_received` 2 external vs 246 wrist |
| 3 | **both views** (311 observations: 120 external / 191 wrist, 40 s) | edge started ready-gated, then the GPU node was restarted while the edge was publishing; external reader recovered (21/41 frames at the check) |

Run 3 is the validation data. The successful recipe: start the edge, wait for
`edge ready`, restart the GPU node, then record. The plain "GPU first, then
edge" order reproduced the external-reader wedge.

## Defined-point result

Per-view tag centre means over run 3 (already chain-corrected by the edge):

| view | mean (mm) | σ (mm) | observations |
|---|---|---:|---:|
| external | (330.5, −14.3, −82.8) | 0.1 / 0.0 / 0.1 | 120 |
| wrist | (330.3, −12.3, −79.6) | 0.4 / 0.9 / 0.6 | 191 |
| **external − wrist** | **(+0.2, −2.0, −3.2)** | | **3.77 mm** |

The replay learns `Track.view_offset = (+0.03, −1.93, −3.31) mm`, matching the
measured difference. That is the defined-point residual after the chain
correction, and it is what replaces the cup's 57-67 mm.

Independent fixed-point check (`fixed_point_check.py`, 20 s per view, 36/30
samples, `tf=exact`, reprojection 0.21 / 0.75 px), 5 mm gate:

| comparison | raw | after `config/view_bias.yaml` (+9.9, −1.8, +9.4 mm on external) |
|---|---:|---:|
| depth (wrist − external) | 15.92 mm FAIL | **3.67 mm PASS** |
| RGB PnP (wrist − external) | 12.79 mm FAIL | **3.07 mm PASS** |

The raw value is the chain bias; the corrected value is the defined-point
agreement. Per-view depth-vs-PnP bias remains: external 4.2 mm, wrist 7.4 mm
(the tag sheet is curled and the wrist sees it more obliquely). The absolute
tag centre in `base_link` is wrist PnP (335.4, −12.7, −85.7) mm; corrected
external PnP is (334.7, −14.7, −83.5) mm. The table reference is −82.5 mm, so
the tag centre sits ~3 mm above it; the printed sheet is not flat.

## Fused-track behaviour

- No 67 mm first-pair step: the replay track starts at t≈0.9 s on the external
  point (330.6, −14.3, −82.9) mm and moves to the wrist reference by only the
  3.8 mm residual; the live recorded track starts at (329.7, −13.1, −78.2) mm,
  within 3.8 mm of both view means.
- Live recorded fused track (edge `eed1ca9`, before the min-dt fix below):
  max 11.4 mm, p95 6.2 mm, final 3.7 mm.
- Offline replay (`--source-bias none`, which is required because the edge
  already applied `view_bias.yaml`) with `velocity_min_dt_s = 0.10`:
  max **8.3 mm**, p95 5.4 mm, final 3.6 mm.
- The remaining few mm are tracker transients, not the reference switch.
  The tag run exposed near-duplicate capture stamps (47/191 wrist observations
  had dt ≤ 0); a millimetre difference over 0.04 s becomes ~25 mm/s and a
  10 mm extrapolation. `velocity_min_dt_s = 0.10` rejects such baselines; it
  was added **after** the live runs and is covered by tests/replay, not yet by
  a new hardware run.

## Ops residue

The ready-gated wrapper's `kill -INT` went to the `ros2 run` parent, not the node
child; two nodes stayed up until they were found ~8-10 min later by PID. After
`SIGKILL` they remain **zombies** because the container's PID 1 is
`tail -f /dev/null` and never reaps; two `[perception_edge] <defunct>` entries
persist until the container is restarted (they consume no CPU). Next time kill
the actual `lib/jcheng_perception_edge/perception_edge` PID (or
`pkill -INT -f "[l]ib/jcheng_perception_edge/perception_edge"`) and check
`ps` afterwards. Pi load after stopping the edges was still ~7-10: the D435i
driver (~84 %), the 8765 gateway (~36 %) and Aurora (~30 %) dominate; the Pi
was responsive, no D-state or USB error.

## Caveats

- One static pose, one tag, no motion.
- The 3.67/3.07 mm cross-view figure is a consistency check against the same
  chain correction stored in `view_bias.yaml`, not an independent absolute
  truth; the per-view depth-vs-PnP bias (4.2 / 7.4 mm) and the tag curl still
  bound absolute accuracy.
- The tag sheet curls up to ~7 mm; flatten it before quoting absolute accuracy.
- The wrist depth stream was poor in run 3 (`aurora_wrist:depth_failures=200`)
  but enough tag observations matched; external depth is the stable one.
- The GPU external uplink reader wedge is a recurring ops issue; the reliable
  order here was edge first, then GPU restart. This needs a real fix (matching
  distros / reader recovery), not just a restart.
- The edge-side `velocity_min_dt_s` fix is not deployed to the Pi yet; the
  container still runs `eed1ca9`. A future bounded run should confirm the
  live tag track drops to the replay's ~8 mm or better.

## Files

- `tag_run3.json` — successful cross-view recording (observations, targets,
  status);
- `tag_run2.json` — wrist-only attempt (external reader wedge);
- `tag_wrist_fixed.csv`, `tag_ext_fixed.csv` — fixed-point check samples;
- `tag_report_raw.json`, `tag_report_corrected.json` — 5 mm gate before/after
  the chain correction;
- `replay_tag_run3.json`, `recorded_tag_run3.json` — fused drift (replay with
  `--source-bias none`, live recorded);
- `edge_tag_verify*.log`, `gpu_tag_verify*.log`, `record_tag*.log` — process
  logs;
- `ext_frame.png`, `wrist_frame.png` — the frames that show the tag in both
  views;
- `summary.json`, `hashes.txt`.

Reproduce the reports:

```bash
python3 scripts/fixed_point_report.py \
  --wrist evidence/hardware_20261124_tag_verify/tag_wrist_fixed.csv \
  --ext evidence/hardware_20261124_tag_verify/tag_ext_fixed.csv \
  --ext-offset 0.0099 -0.0018 0.0094 --limit 0.005 \
  --out evidence/hardware_20261124_tag_verify/tag_report_corrected.json
python3 scripts/replay_static_drift.py \
  --source evidence/hardware_20261124_tag_verify/tag_run3.json \
  --source-bias none --settle-s 6 --require reference_tag_0 --limit-mm 15 \
  --out evidence/hardware_20261124_tag_verify/replay_tag_run3.json
```
