# Synthetic ore scene dataset

- schema: `jcheng.ore_dataset.v1`
- dataset_id: `synthetic_20261127`
- source: **synthetic** (procedurally generated, no real ore)
- annotation: **procedural ground truth**, pixel-accurate by construction
- license of generated data: **CC0-1.0**
- generator: `jcheng_perception_scene.synthetic` 1.0.0
- scenes: 39 (including 3 empty)
- seed: 20261127

## Limits

This dataset measures whether the offline pipeline and evaluation code
behave as specified. It says nothing about accuracy on real ore, real
conveyor lighting, dust, water, scale variation or mineral appearance.
Do not report synthetic metrics as real-ore validation.
