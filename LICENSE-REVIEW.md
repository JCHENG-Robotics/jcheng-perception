# License review — pending

This repository does **not** ship a `LICENSE` file. Copyright is reserved;
JCHENG-Robotics license review is in progress, consistent with the other
organization repositories.

## What is in this repository

- Original Python and ROS 2 source written for this project.
- Configuration values and measured extrinsics from `jcheng-manipulation`
  (internal project evidence).
- No model weights, no vendor binaries, no third-party source copies.

## Third-party runtime dependencies

| Dependency | Use | License note |
|---|---|---|
| ROS 2 / rclpy / tf2 / cv_bridge | runtime | Apache-2.0 |
| OpenCV | image codec, mock detector | Apache-2.0 |
| NumPy | numerics | BSD-3-Clause |
| PyYAML | configuration | MIT |
| Ultralytics | optional GPU detector backend | **AGPL-3.0 by default**; commercial licensing may be required |
| OpenVINO | optional IR detector backend | Apache-2.0 |
| ONNX Runtime | optional ONNX detector backend | MIT |

The optional runtimes are installed in a local venv, not vendored into this
repository. Model weights remain outside the repository.

## Offline ore scene understanding (S1, 2026-11-26)

`jcheng_perception_scene` adds **no new runtime dependency**: it uses the
already-declared NumPy (BSD-3-Clause), PyYAML (MIT) and OpenCV (Apache-2.0,
`opencv-python-headless` in CI). It ships **no model weights** and performs no
network, DDS or device access. The synthetic data it generates is CC0-1.0.

The public evaluation data is **not** committed. `Mendeley — benchmark for rock
blasting` (`40_70mm`, DOI `10.17632/v9grvgsrh9.1`) is **CC BY 4.0**; any
redistribution or publication must attribute Ronkin & Reshetnikov and keep the
CC BY 4.0 notice. Its annotations' human-review/SAM-assistance status is not
independently verified; the report labels it as a public domain-gap baseline,
not a gold standard.

Candidate deep backends were reviewed but **not added**:

| Candidate | Code license | Weight license | Decision |
|---|---|---|---|
| SAM 2 / SAM 2.1 | Apache-2.0 | Apache-2.0 | S1.5 candidate; 6 GB not yet measured |
| MobileSAM | Apache-2.0 | Apache-2.0 | S1.5 alternative |
| YOLO-seg (Ultralytics) | AGPL-3.0 | AGPL-3.0 | existing optional runtime; not a default |
| Mask R-CNN (Detectron2) | Apache-2.0 | CC BY-SA 3.0 | research only; share-alike weights |
| Mask2Former | MIT | CC BY-NC 4.0 | research only; non-commercial weights |
| FastSAM | AGPL-3.0 | AGPL-3.0 | not used |
| EdgeSAM | S-Lab License 1.0 (non-commercial) | non-commercial | not used |
| GraspNet-1Billion baseline | academic/non-profit non-commercial | same | not used |
| AnyGrasp | closed binary + machine-bound license | proprietary | not used |
| Contact-GraspNet | `License.pdf`, terms unverified | unverified | not used; ≥8 GB inference |

Any future weight must be recorded here with origin, SHA-256, license and export
format before use. No weight file may enter the repository tree.

## Model weights

Weights are deliberately not committed. The `gpu.yaml` paths point outside the
repository. Anyone adding a weight file must record its origin, license and
export format in this file before committing it. Ultralytics `YOLO` model
weights are subject to the Ultralytics license, not this repository's pending
license.

## Vendor and baseline material

The `jcheng-manipulation` repository is a frozen reference baseline. This
repository reuses its published frame names and calibration numbers only; it
does not copy vendor code.
