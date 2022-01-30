from murl.particles import ParticleCloud
from murl.sensors import CONE_WIDTH

import numpy as np
from scipy.stats import norm

class Cloud(ParticleCloud):
    def calculate_weight(self, angle_dists, _robot_guesses, _robot_scans):
        self._particles[:,3] = 0.0
        mask = self._world.is_valid(self._particles)
        self._particles[~mask,3] = float('-inf')
        count = np.ma.array(self._particles[:,3], mask=~mask).count()

        for angle, dist in angle_dists:
            angles = angle + self._rng.uniform(-CONE_WIDTH/2, CONE_WIDTH/2,
                                               (self.RAY_SAMPLES, count))
            true_dists = self._world.get_dist(self._particles[mask], angles)
            if np.isfinite(dist):
                self._particles[mask,3] += norm.logpdf(dist, true_dists, 0.8)
            else:
                self._particles[mask,3] += norm.logsf(3, true_dists, 0.8)
