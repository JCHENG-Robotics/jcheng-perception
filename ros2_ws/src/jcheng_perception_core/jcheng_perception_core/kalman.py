"""
Constant-velocity Kalman filter in three dimensions.

The state is ``[x, y, z, vx, vy, vz]`` in the target frame (nominally
``base_link``). Depth measurements provide position only; the filter supplies
velocity and a smooth prediction between the asynchronous detection streams.
"""

import numpy as np


class ConstantVelocityKalman:
    """A 6-state constant-velocity Kalman filter with Joseph updates."""

    def __init__(self, position, covariance3, velocity=None,
                 accel_noise=0.35, measurement_floor_m=0.0008,
                 initial_velocity_sigma=0.25):
        """Create a filter from a position and a 3x3 covariance."""
        position = np.asarray(position, dtype=np.float64).reshape(3)
        covariance3 = np.asarray(
            covariance3, dtype=np.float64).reshape(3, 3)
        if velocity is None:
            velocity = np.zeros(3, dtype=np.float64)
        self.state = np.concatenate([
            position, np.asarray(velocity, dtype=np.float64).reshape(3)
        ])
        self.covariance = np.zeros((6, 6), dtype=np.float64)
        self.covariance[:3, :3] = covariance3
        # A 1 m/s velocity prior (the original default) lets one biased
        # cross-view sample masquerade as metres per second of motion. The
        # measured arm/table speeds are below 0.15 m/s, so 0.25 m/s is already
        # deliberately permissive.
        self.covariance[3:, 3:] = np.eye(3) * (
            max(1e-6, float(initial_velocity_sigma)) ** 2)
        self.accel_noise = max(1e-6, float(accel_noise))
        self.measurement_floor_m = max(1e-9, float(measurement_floor_m))
        self.last_stamp_ns = 0

    @property
    def position(self):
        """Return a copy of the position state."""
        return self.state[:3].copy()

    @property
    def velocity(self):
        """Return a copy of the velocity state."""
        return self.state[3:].copy()

    @property
    def position_covariance(self):
        """Return a copy of the 3x3 position covariance."""
        return self.covariance[:3, :3].copy()

    def predict(self, dt_s):
        """Advance the state by ``dt_s`` seconds."""
        dt = max(0.0, float(dt_s))
        if dt <= 0.0:
            return
        transition = np.eye(6, dtype=np.float64)
        transition[0, 3] = dt
        transition[1, 4] = dt
        transition[2, 5] = dt
        q = self.accel_noise
        process = np.zeros((6, 6), dtype=np.float64)
        block = np.array([
            [dt ** 3 / 3.0, dt ** 2 / 2.0],
            [dt ** 2 / 2.0, dt],
        ], dtype=np.float64)
        for axis in range(3):
            process[axis, axis] = q * block[0, 0]
            process[axis, axis + 3] = q * block[0, 1]
            process[axis + 3, axis] = q * block[1, 0]
            process[axis + 3, axis + 3] = q * block[1, 1]
        process += np.eye(6) * 1e-12
        self.state = transition @ self.state
        self.covariance = (
            transition @ self.covariance @ transition.T + process)
        self._symmetrise()
        self.last_stamp_ns += int(dt * 1e9)

    def update(self, measurement, covariance, velocity_gain=1.0):
        """Update with a 3-D position measurement and its 3x3 covariance.

        ``velocity_gain`` scales the velocity rows of the Kalman gain. A value
        below one is used when the update comes from a different view than the
        previous one: the views measure different physical points, so their
        difference is a per-view bias, not evidence of motion. Scaling a valid
        gain keeps the Joseph covariance update consistent.
        """
        value = np.asarray(measurement, dtype=np.float64).reshape(3)
        noise = np.asarray(covariance, dtype=np.float64).reshape(3, 3)
        noise = 0.5 * (noise + noise.T)
        noise = noise + np.eye(3) * self.measurement_floor_m ** 2
        observation = np.zeros((3, 6), dtype=np.float64)
        observation[:, :3] = np.eye(3)
        innovation = value - observation @ self.state
        innovation_covariance = (
            observation @ self.covariance @ observation.T + noise)
        try:
            inverse = np.linalg.inv(innovation_covariance)
        except np.linalg.LinAlgError:
            inverse = np.linalg.pinv(innovation_covariance)
        gain = self.covariance @ observation.T @ inverse
        gain = np.asarray(gain, dtype=np.float64).copy()
        gain[3:, :] *= float(np.clip(velocity_gain, 0.0, 1.0))
        self.state = self.state + gain @ innovation
        identity = np.eye(6, dtype=np.float64)
        joseph = identity - gain @ observation
        self.covariance = (
            joseph @ self.covariance @ joseph.T + gain @ noise @ gain.T)
        self._symmetrise()
        try:
            mahalanobis = float(innovation.T @ inverse @ innovation)
        except (ValueError, np.linalg.LinAlgError):
            mahalanobis = float('inf')
        return innovation, mahalanobis

    def gating_distance(self, measurement, covariance):
        """Return the squared Mahalanobis distance of a measurement."""
        value = np.asarray(measurement, dtype=np.float64).reshape(3)
        noise = np.asarray(covariance, dtype=np.float64).reshape(3, 3)
        observation = np.zeros((3, 6), dtype=np.float64)
        observation[:, :3] = np.eye(3)
        innovation = value - observation @ self.state
        total = observation @ self.covariance @ observation.T + noise
        try:
            inverse = np.linalg.inv(total)
        except np.linalg.LinAlgError:
            inverse = np.linalg.pinv(total)
        return float(innovation.T @ inverse @ innovation)

    def update_velocity(self, measurement, covariance):
        """Update the velocity state with a velocity-only measurement.

        Position and velocity are correlated in the covariance, so this is a
        full 6-state Joseph update with ``H = [0 | I]``; the position state is
        only changed through the cross-covariance.
        """
        value = np.asarray(measurement, dtype=np.float64).reshape(3)
        noise = np.asarray(covariance, dtype=np.float64).reshape(3, 3)
        noise = 0.5 * (noise + noise.T)
        noise = noise + np.eye(3) * 1e-12
        observation = np.zeros((3, 6), dtype=np.float64)
        observation[:, 3:] = np.eye(3)
        innovation = value - observation @ self.state
        innovation_covariance = (
            observation @ self.covariance @ observation.T + noise)
        try:
            inverse = np.linalg.inv(innovation_covariance)
        except np.linalg.LinAlgError:
            inverse = np.linalg.pinv(innovation_covariance)
        gain = self.covariance @ observation.T @ inverse
        self.state = self.state + gain @ innovation
        identity = np.eye(6, dtype=np.float64)
        joseph = identity - gain @ observation
        self.covariance = (
            joseph @ self.covariance @ joseph.T + gain @ noise @ gain.T)
        self._symmetrise()
        return innovation

    def _symmetrise(self):
        """Keep the covariance numerically symmetric."""
        self.covariance = 0.5 * (self.covariance + self.covariance.T)
