# ADR 0003 — Distance-weighted two-view information fusion

- Status: accepted
- Date: 2026-11-23

## Context

The external D435i gives coarse, wide-area localization and drop-point recheck.
The wrist Aurora gives precise close-range measurements and grasp keypoints.
Their accuracy depends on range (stereo depth error grows with z²), detector
confidence, keypoint support and occlusion. A fixed 50/50 average would let a
far, weak view degrade a close, strong one.

## Decision

- Each observation carries a 3×3 position covariance derived from pixel noise
  and a per-camera depth-noise model.
- The view factor is `prior(phase, role) · quality / sigma_eff²`; quality includes
  score, keypoint support, occlusion and a range penalty.
- Factors are normalised by their maximum and fused by inverse information:
  `R_fused = (Σ factor_i C_i⁻¹)⁻¹`, `p_fused = Σ w_i p_i`.
- Phase priors decide which view dominates when both are equally precise:
  external for COARSE/DROP_RECHECK, wrist for FINE.
- Tracks use a constant-velocity Kalman filter; duplicate per-camera tracks are
  merged (6 cm/3σ disjoint, strict 2 cm overlapping).

## Consequences

Positive:

- Near, confident, unoccluded views dominate automatically.
- Single-view updates do not get artificially sharpened (factor normalisation).
- Weights are published (`weight_external`, `weight_wrist`) and can be logged.

Negative:

- The depth-noise model needs per-camera calibration; defaults are initial values.
- Phase priors are a policy choice that must be reviewed for new tasks.
- Two views are treated as independent; correlated depth errors (common lighting,
  calibration bias) can make the fused covariance optimistic.
- Orientation remains ambiguous for symmetric objects; covariance marks roll/pitch
  unobserved but yaw can still be 90°/180° wrong.

## Alternatives considered

- Fixed weights: rejected (range/occlusion dependent).
- Sequential Kalman updates only: acceptable, but makes the published view
  weights less interpretable; a short coalescing window is used to fuse both
  views when they belong to the same capture round.
- Learned fusion: deferred until a labelled hardware dataset exists.
