# Ore scene understanding S1 evidence (2026-11-26)

**Scope**: offline only. No robot connection, no DDS, no depth, no motion.
`hardware_claimed: false`; `real_ore_validation: false`.

## What was run

1. Synthetic test set: seed `20261127`, 36 non-empty + 3 empty scenes,
   640×480, 600 visible GT instances.
2. Classical watershed baseline (`classical_watershed.v1.0.0`) with
   `config/ore_scene.yaml`.
3. Public Mendeley `40_70mm` (CC BY 4.0):
   - tuning: `40_0`, `40_1`;
   - held-out test: `40_2`–`40_9`;
   - 2,592×1,944; dataset-provided COCO polygons.
4. `selftest`: empty scene, truncated image, overlapping masks.
5. `python3 -m pytest tests -q`: 93 passed.

Exact commands are in `docs/ORE_SCENE_S1.md` §4 and in `summary.json`.

## Results

### Synthetic (provenance: synthetic, procedural ground truth)

| Metric | Value |
|---|---|
| scenes / GT / predicted | 39 / 600 / 346 |
| mask AP50 | **0.3682** |
| mask AP50-95 | **0.2114** |
| precision @ IoU 0.5, score 0.5 | 0.7513 |
| recall @ IoU 0.5, score 0.5 | 0.2467 |
| instance count MAE | 6.51 (relative 0.4233) |
| latency P50 / P95 | 14.4 / 22.3 ms |
| failed / empty scenes | 0 / 3 (0 predictions) |

### Public `40_70mm` (provenance: public, dataset-provided polygons)

| Split | Images | GT | Predicted | AP50 | AP50-95 | Count MAE | Latency P50/P95 |
|---|---|---|---|---|---|---|---|
| tuning | 2 | 2,346 | 890 | 0.0430 | 0.0119 | 728.0 | 6.56 / 7.53 s |
| held-out test | 8 | 9,208 | 2,860 | **0.0429** | **0.0122** | 793.5 | 4.68 / 8.53 s |

The public result is a **domain-gap baseline, not an acceptance result**. The
classical baseline under-segments dense real piles by roughly 2.5–3×.

## Files

| File | Content |
|---|---|
| `summary.json` | machine-readable bundle summary, hashes, environment, commands |
| `synthetic_metrics.json` | full synthetic evaluation |
| `synthetic_run_summary.json` | run counts/environment |
| `synthetic_dataset_manifest.json` | synthetic ground truth manifest |
| `synthetic_DATASET_CARD.md` | synthetic provenance card |
| `public_tune_metrics.json` | public tuning metrics |
| `public_test_metrics.json` | public held-out metrics |
| `public_test_run_summary.json` | public run counts/environment |
| `selftest.json` | empty/bad-input/overlap checks |
| `ore_scene.yaml`, `ore_scene_public_40_70mm.yaml` | effective configs |
| `overlays/*.jpg` | downscaled qualitative overlays (not metric inputs) |
| `sha256sums.txt` | SHA-256 of bundle files |

## Reproduce

```bash
# from jcheng-perception/
python3 scripts/ore_scene.py generate-synthetic \
  --out <data>/synthetic --scenes 36 --empty-scenes 3 \
  --seed 20261127 --width 640 --height 480
python3 scripts/ore_scene.py run --dataset <data>/synthetic \
  --out <run>/run_synthetic --config config/ore_scene.yaml

python3 scripts/ore_scene.py import-coco \
  --coco <mendeley>/annotations/instances_default.json \
  --images <mendeley>/images/default \
  --out <data>/public_test --dataset-id mendeley_40_70mm_test \
  --license CC-BY-4.0 --image-offset 2 --image-limit 8
python3 scripts/ore_scene.py run --dataset <data>/public_test \
  --out <run>/run_public_test \
  --config config/ore_scene_public_40_70mm.yaml --no-overlays
```

The Mendeley images/annotations and the synthetic images are **outside the
repository**; only manifests/metrics/overlays are committed. Reproducing the
public result requires downloading the CC BY 4.0 dataset (DOI
`10.17632/v9grvgsrh9.1`) and extracting `40_70mm.zip`.

## Limits

- Synthetic metrics prove pipeline mechanics, not real-ore accuracy.
- Public annotations' human-review status could not be verified; they may
  include SAM/Mask R-CNN assistance. Overlapping polygons are flattened into a
  visible label map.
- No depth, TF, DDS, device or motion path is exercised.
- Scores are heuristic and uncalibrated; operating-point P/R are not guarantees.
- No GPU was used for S1.
