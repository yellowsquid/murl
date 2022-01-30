from abc import ABC, abstractmethod

import numpy as np
import rospy

from geometry_msgs.msg import Point32
from sensor_msgs.msg import ChannelFloat32, PointCloud

class ParticleCloud(ABC):
    PARTICLE_COUNT = 10000
    RESAMPLE_PROB = 2**-10
    RAY_SAMPLES = 3
    ALPHAS = [0.01, 0.00, 0.01, 0.00]

    def __init__(self, world, robot, rng):
        self._world = world
        self._particles = np.array([[0.0] * 4] * self.PARTICLE_COUNT)
        self._publisher = rospy.Publisher('/robot{}/particles'.format(robot), PointCloud, queue_size=1)
        self._frame = 'robot{}/odom'.format(robot)
        self._robot = robot
        self._rng = rng


    def resample_all(self):
        self.resample(np.array([True] * self.PARTICLE_COUNT))


    def resample(self, mask):
        tmp = self._particles[mask,:]
        self._world.sample(tmp)
        self._particles[mask,:] = tmp


    def move(self, delta):
        rot1 = np.arctan2(delta[1], delta[0])

        # Assume large rot1 means reversing
        if rot1 < -np.pi:
            rot1 += np.pi
        elif rot1 > np.pi:
            rot1 -= np.pi

        trans = np.sqrt(delta[0]**2 + delta[1]**2)
        rot2 = delta[2] - rot1

        rot1_var = self.ALPHAS[0] * rot1**2 + self.ALPHAS[1] * trans**2
        trans_var = self.ALPHAS[2] * trans**2 + self.ALPHAS[3] * (rot1**2 + rot2**2)
        rot2_var = self.ALPHAS[0] * rot2**2 + self.ALPHAS[1] * trans**2

        rot1 += self._rng.normal(scale = np.sqrt(rot1_var), size=self.PARTICLE_COUNT)
        trans += self._rng.normal(scale = np.sqrt(trans_var), size=self.PARTICLE_COUNT)
        rot2 += self._rng.normal(scale = np.sqrt(rot2_var), size=self.PARTICLE_COUNT)

        self._particles[:,0] += trans * np.cos(rot1 + self.yaws)
        self._particles[:,1] += trans * np.sin(rot1 + self.yaws)
        self._particles[:,2] += rot1 + rot2

        mask = self._rng.random(self.PARTICLE_COUNT) < self.RESAMPLE_PROB
        self.resample(mask)


    @abstractmethod
    def calculate_weight(self, angle_dists, robot_guesses, robot_scans):
        raise NotImplementedError()


    def average(self):
        biggest = max(self._particles[:,3])
        mask = self._particles[:,3] > biggest - 5
        masked = self._particles[mask]

        if len(masked) < 20 or not np.isfinite(biggest):
            print('Massive collapse for robot', self._robot)
            self._particles[:,3] = 0.
            self.resample_all()
            return

        weights = np.exp(masked[:,3] - max(masked[:,3]))
        self._particles = self._rng.choice(masked, size=self.PARTICLE_COUNT, p=weights/np.sum(weights))
        self._particles[:,3] = 0.0


    def guess_position(self):
        weights = np.exp(self._particles[:,3])
        weights /= np.sum(weights)

        mean = np.average(self._particles[:,:2], weights=weights, axis=0)
        cov = np.cov(self._particles[:,:2], rowvar=False, ddof=0, aweights=weights)

        # x and y being too correlated is suspicious
        if abs(cov[0,0]*cov[1,1] - cov[0,1] * cov[1,0]) < 1e-15:
            print('Singular covariance matrix for robot', self._robot)
            # pretend to uncorrelate the data slightly
            cov[0,0] += 0.1
            cov[1,1] += 0.1

        return mean, cov


    def publish(self, frame_id):
        msg = PointCloud()
        msg.header.seq = frame_id
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self._frame
        intensity = ChannelFloat32()
        intensity.name = 'intensity'
        msg.channels.append(intensity)
        msg.points = [Point32(x = particle[0], y = particle[1], z = 0.05) for particle in self._particles]
        intensity.values = list(np.exp(self._particles[:,3]))
        self._publisher.publish(msg)


    @property
    def particles(self):
        return self._particles


    @property
    def yaws(self):
        return self._particles[:,2]


    @property
    def log_weights(self):
        return self._particles[:,3]


    @property
    def weights(self):
        return np.exp(self.log_weights)
