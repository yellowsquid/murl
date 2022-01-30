import numpy as np
import rospy

from gazebo_msgs.msg import ModelStates
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion

SCAN_ANGLES = [0., np.pi / 4., -np.pi / 4., np.pi / 2., -np.pi / 2.]
CONE_WIDTH = np.pi / 180. * 3.1 # 3.1 degrees cone of view (3 rays).

class SimpleLaser:
    def __init__(self, robot):
        rospy.Subscriber('/robot{}/scan'.format(robot), LaserScan, self.callback)
        self._angles = SCAN_ANGLES
        self._width = CONE_WIDTH
        self._measurements = [float('inf')] * len(self._angles)
        self._indices = None

    def callback(self, msg):
        # Helper for angles.
        def _within(x, a, b):
            pi2 = np.pi * 2.
            x %= pi2
            a %= pi2
            b %= pi2
            if a < b:
                return a <= x and x <= b
            return a <= x or x <= b;

        # Compute indices the first time.
        if self._indices is None:
            self._indices = [[] for _ in range(len(self._angles))]
            for i, d in enumerate(msg.ranges):
                angle = msg.angle_min + i * msg.angle_increment
                for j, center_angle in enumerate(self._angles):
                    if _within(angle, center_angle - self._width / 2., center_angle + self._width / 2.):
                        self._indices[j].append(i)

        ranges = np.array(msg.ranges)
        for i, idx in enumerate(self._indices):
            # We do not take the minimum range of the cone but the 10-th percentile for robustness.
            self._measurements[i] = np.percentile(ranges[idx], 10)

    @property
    def ready(self):
        return not np.isnan(self._measurements[0])

    @property
    def measurements(self):
        return self._measurements


class Motion:
    def __init__(self, robot):
        self._previous_time = None
        self._delta_pose = np.array([0., 0., 0.], dtype=np.float32)
        rospy.Subscriber('/robot{}/odom'.format(robot), Odometry, self.callback)

    def callback(self, msg):
        u = msg.twist.twist.linear.x
        w = msg.twist.twist.angular.z
        if self._previous_time is None:
            self._previous_time = msg.header.stamp
        current_time = msg.header.stamp
        dt = (current_time - self._previous_time).to_sec()
        self._delta_pose[0] += u * dt
        self._delta_pose[1] += 0.
        self._delta_pose[2] += w * dt
        self._previous_time = current_time

    @property
    def ready(self):
        return True

    @property
    def delta_pose(self):
        ret = self._delta_pose.copy()
        self._delta_pose[:] = 0
        return ret


class SimpleLaser:
    def __init__(self, robot):
        rospy.Subscriber('/robot{}/scan'.format(robot), LaserScan, self.callback)
        self._angles = SCAN_ANGLES
        self._width = CONE_WIDTH
        self._measurements = [float('inf')] * len(self._angles)
        self._indices = None

    def callback(self, msg):
        # Helper for angles.
        def _within(x, a, b):
            pi2 = np.pi * 2.
            x %= pi2
            a %= pi2
            b %= pi2
            if a < b:
                return a <= x and x <= b
            return a <= x or x <= b;

        # Compute indices the first time.
        if self._indices is None:
            self._indices = [[] for _ in range(len(self._angles))]
            for i, d in enumerate(msg.ranges):
                angle = msg.angle_min + i * msg.angle_increment
                for j, center_angle in enumerate(self._angles):
                    if _within(angle, center_angle - self._width / 2., center_angle + self._width / 2.):
                        self._indices[j].append(i)

        ranges = np.array(msg.ranges)
        for i, idx in enumerate(self._indices):
            # We do not take the minimum range of the cone but the 10-th percentile for robustness.
            self._measurements[i] = np.percentile(ranges[idx], 10)

    @property
    def ready(self):
        return not np.isnan(self._measurements[0])

    @property
    def measurements(self):
        return self._measurements


class Motion(object):
    def __init__(self, robot):
        self._previous_time = None
        self._delta_pose = np.array([0., 0., 0.], dtype=np.float32)
        rospy.Subscriber('/robot{}/odom'.format(robot), Odometry, self.callback)

    def callback(self, msg):
        u = msg.twist.twist.linear.x
        w = msg.twist.twist.angular.z
        if self._previous_time is None:
            self._previous_time = msg.header.stamp
        current_time = msg.header.stamp
        dt = (current_time - self._previous_time).to_sec()
        self._delta_pose[0] += u * dt
        self._delta_pose[1] += 0.
        self._delta_pose[2] += w * dt
        self._previous_time = current_time

    @property
    def ready(self):
        return True

    @property
    def delta_pose(self):
        ret = self._delta_pose.copy()
        self._delta_pose[:] = 0
        return ret


class GroundtruthPose:
    def __init__(self, robot):
        rospy.Subscriber('/gazebo/model_states', ModelStates, self.callback)
        self._pose = np.array([np.nan, np.nan, np.nan], dtype=np.float32)
        self._name = 'Robot{}'.format(robot)

    def callback(self, msg):
        idx = [i for i, n in enumerate(msg.name) if n == self._name]
        if not idx:
            raise ValueError('Specified name "{}" does not exist.'.format(self._name))
        idx = idx[0]
        self._pose[0] = msg.pose[idx].position.x
        self._pose[1] = msg.pose[idx].position.y
        _, _, yaw = euler_from_quaternion([
                msg.pose[idx].orientation.x,
                msg.pose[idx].orientation.y,
                msg.pose[idx].orientation.z,
                msg.pose[idx].orientation.w])
        self._pose[2] = yaw

    @property
    def ready(self):
        return not np.isnan(self._pose[0])

    @property
    def pose(self):
        return self._pose
