# SAM 2.1 ore S1.5 MVE evidence (2026-11-26)

**Scope**: offline only. `hardware_claimed: false`, `real_ore_validation: false`.
No DDS, device, depth, motion or runtime change. Full masks are outside git.

## Model and host

| Item | Value |
|---|---|
| Model | SAM 2.1 Hiera Tiny, Apache-2.0 |
| Checkpoint | `sam2.1_hiera_tiny.pt`, 156,008,466 B |
| SHA-256 | `7402e0d864fa82708a20fbd15bc84245c2f26dff0eb43a4b5b93452deb34be69` |
| Host | AGX Orin 64 GB, `192.168.5.31` |
| Workspace | `/home/ti5robot/jcheng-s1.5` (separate from `~/shelf_sam3_orin`) |
| Python / torch | 3.10.19 / 2.8.0 |
| Peak GPU | ~2.5 GB per 1008 tile; ~5.7 GB for a 5 MP image at 1008 |
| Coexistence | Qwen3-VL llama-server stayed alive; SAM3 environment untouched |
| Known issue | SAM2 `_C` extension missing; post-processing skipped (masks still full size) |

## Data

Mendeley `benchmark for rock blasting`, `40_70mm` subset, CC BY 4.0,
DOI `10.17632/v9grvgsrh9.1`. Held-out: `40_10, 40_11, 40_13, 40_14,
40_15, 40_16, 40_17, 40_18` — 8 images, 9,208 visible GT instances.
Annotations' human-review/SAM-assist status is unverified; overlapping polygons
are flattened in the internal label maps.

## Results

| Strategy | AP50 | AP50-95 | P | R | Pred/GT | Count MAE | Latency P50/P95 |
|---|---|---|---|---|---|---|---|
| Fixed 1008 tiles | 0.508 | 0.328 | 0.705 | 0.591 | 7723/9208 | 443.4 | 43.2/46.1 s |
| **1008 + 504 fallback (<500 preds)** | **0.637** | **0.387** | 0.617 | 0.775 | 11564/9208 | 335.5 | 45.6/287.8 s |
| Classical watershed (S1) | 0.043 | 0.012 | 0.275 | 0.028 | 2860/9208 | 793.5 | 4.7/8.5 s |

Failure-image recovery with 504 tiles:

| Image | 1008 AP50 | 504 AP50 |
|---|---|---|
| `40_10` | 0.141 | 0.567 |
| `40_14` | 0.320 | 0.477 |
| `40_15` | 0.089 | 0.549 |

Synthetic subset (sparse + dense): SAM2 AP50 0.879 / AP50-95 0.793 vs
classical 0.389 / 0.213.

Independent image split (selected before inference; not used in tuning or the
8-image test): `40_2, 40_19, 40_20, 40_21`, 4,585 GT. The `<500` policy was
frozen before this run.

| Strategy | AP50 | AP50-95 | P | R | Pred/GT | Count MAE |
|---|---|---|---|---|---|---|
| Fixed 1008 | 0.568 | 0.359 | 0.726 | 0.637 | 4026/4585 | 282.3 |
| **Frozen 1008 + 504 fallback** | **0.6375** | **0.3909** | 0.649 | 0.745 | 5262/4585 | 246.3 |

Only `40_2` triggered the fallback: 0.245 → **0.546**. Same public dataset and
annotation source; this is an independent image split, not an independent
domain.

Independent scene type — `Rocks UAV`, 2 images at 5472×3648 (803 GT), fixed
1008 tiles, no tuning and no 504 fallback:

| Image | AP50 | AP50-95 | P | R | Pred/GT | Count MAE |
|---|---|---|---|---|---|---|
| `DJI_0157` | 0.216 | 0.123 | 0.127 | 0.540 | 1119/263 | 856 |
| `DJI_0158` | 0.341 | 0.185 | 0.202 | 0.637 | 1704/540 | 1164 |
| **Aggregate** | **0.301** | **0.164** | 0.172 | 0.605 | 2823/803 | 1010 |

**Negative result:** the 40–70 mm tile/area/merge policy does not transfer to
aerial UAV scenes; it over-segments ~3.5× GT. Domain-specific scale/tiling or a
trained model is required.

## Files

| File | Content |
|---|---|
| `metrics_1008_8image.json` | fixed-1008 evaluation on 8 held-out images |
| `per_image_1008_8image.json` | per-image fixed-1008 metrics |
| `metrics_504_40_10_15.json` | 504 evaluation for `40_10`/`40_15` |
| `per_image_504_40_10_15.json` | per-image 504 metrics |
| `metrics_504_40_14.json` | 504 evaluation for `40_14` |
| `metrics_hybrid500_8image.json` | adaptive hybrid evaluation |
| `metrics_independent_1008.json` | fixed-1008 on the independent 4-image split |
| `metrics_independent_504_40_2.json` | 504 fallback for `40_2` |
| `metrics_independent_hybrid.json` | frozen-policy hybrid on the independent split |
| `per_image_independent_hybrid.json` | per-image independent hybrid metrics |
| `summary_1008.json` | method, host, weight hash, limitations |
| `summary_hybrid500.json` | adaptive policy, results, limitations |
| `summary_independent_hybrid.json` | independent-split policy/results/limits |
| `metrics_uav_1008.json` | Rocks UAV fixed-1008 evaluation (negative result) |
| `per_image_uav_1008.json` | per-image UAV metrics |
| `summary_uav_1008.json` | UAV scene-type method/results/limits |

## Reproduce

Remote inference:

```bash
cd ~/jcheng-s1.5
~/jcheng-s1.5/sam2-venv/bin/python jobs/run_sam2_auto.py \
  --images-dir <tiles> --out <out> \
  --checkpoint ~/jcheng-s1.5/weights/sam2.1_hiera_tiny.pt \
  --points-per-side 32
```

Local merge/evaluation uses the same `jcheng_perception_scene` mask-AP code as
S1; full label maps live in
`/home/jerichocheng/jcheng/perception_datasets/ore_s1_20261126/`.

## Limits

- Threshold 500 and tile/merge parameters were chosen on the earlier 8 images.
  A separate 4-image split (selected before inference) reproduced AP50 0.6375,
  but it is the same public dataset/annotation source, not an independent
  domain.
- 504 fallback over-segments (11,564 predictions / 9,208 GT on the 8-image set;
  5,262 / 4,585 on the independent set) and costs ~2.5–4.8 min/image.
- Only 14 of 49 public images across all splits; no real-ore data.
- The 2-image Rocks UAV check (different scene type) failed to transfer:
  AP50 0.301, precision 0.172, ~3.5× over-segmentation. This is a negative
  result, not a capability claim.
- No depth, 3D, multi-view, grasp or runtime integration.
- `_C` post-processing is skipped; verify before production.
