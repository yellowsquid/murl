#!/usr/bin/env python3
import argparse
import copy
import importlib
import time

import numpy as np
import rospy

from geometry_msgs.msg import Twist
from numpy.random import default_rng
from std_msgs.msg import Header

import murl.sensors as sensors
from murl.world.simple import SimpleWorld


RNG = default_rng()


class Robot:
    SENSORS_TO_WHEELS = np.array([[0.2, 0.2, -0.2, 0.1, -0.1],
                                  [0.2, -0.2, 0.2, -0.1, 0.1]])
    WHEELS_TO_UW = np.array([[.5, .5], [.5, -.5]])
    SENSORS_TO_UW = np.matmul(WHEELS_TO_UW, SENSORS_TO_WHEELS)

    def __init__(self, world, robot, cloud_module):
        self._robot = robot
        self._publisher = rospy.Publisher('/robot{}/cmd_vel'.format(robot), Twist, queue_size=5)
        self._laser = sensors.SimpleLaser(robot)
        self._motion = sensors.Motion(robot)
        self._groundtruth = sensors.GroundtruthPose(robot)
        self._particles = cloud_module.Cloud(world, robot, RNG)
        self._particles.resample_all()


    def move(self):
        f, fl, fr, l, r = self._laser.measurements
        inputs = np.clip(np.array([f, fl, fr, l, r]), 0, 3.0)
        uw = np.matmul(self.SENSORS_TO_UW, inputs)

        u = uw[0]
        w = uw[1]

        if f < 0.3:
            u = -0.01
            w = 0.5
        elif fl < 0.5:
            u = -0.01
            w = -0.5
        elif fr < 0.5:
            u = -0.01
            w = 0.5
        elif l < 0.5:
            w = -0.5
        elif r < 0.5:
            w = 0.5

        msg = Twist()
        msg.linear.x = u
        msg.angular.z = w
        self._publisher.publish(msg)


    @property
    def ready(self):
        return self._laser.ready and self._motion.ready and self._groundtruth.ready


    @property
    def delta_pose(self):
        return self._motion.delta_pose


    @property
    def pose(self):
        return self._groundtruth.pose


    @property
    def scan(self):
        return self._laser._measurements


    @property
    def particles(self):
        return self._particles


def run(args):
    rospy.init_node('localisation')

    # Update control every 100 ms.
    rate_limiter = rospy.Rate(100)

    world = SimpleWorld(RNG)
    cloud_module = importlib.import_module(args.cloud)
    robots = [Robot(world, i, cloud_module) for i in range(args.robots)]
    robot_guesses = [None for i in range(args.robots)]
    robot_scans = [[None for i in range(args.robots)] for i in range(args.robots)]

    for i, robot in enumerate(robots):
        robot_guesses[i] = robot.particles.guess_position()
    print('Ready!')

    pose_history = []
    with open('/tmp/gazebo_exercise.txt', 'w'):
        pass

    frame_id = 0
    while not rospy.is_shutdown():
        # Make sure all measurements are ready.
        if not all(map(lambda robot: robot.ready, robots)):
            rate_limiter.sleep()
            continue

        # Update robot positions
        for robot in robots:
            robot.move()


        # Recalculate relative robot positions
        for i in range(args.robots):
            for j in range(args.robots):
                if i == j:
                    continue
                dist = robots[j].pose[:2] - robots[i].pose[:2]
                angle = np.arctan2(dist[1], dist[0]) - robots[i].pose[2]
                robot_scans[i][j] = np.sqrt(dist[0]**2 + dist[1]**2), angle

        for i, robot in enumerate(robots):
            delta = robot.delta_pose
            angle_dists = zip(sensors.SCAN_ANGLES, robot.scan)
            robot.particles.move(delta)
            guesses = robot_guesses[:i] + robot_guesses[i+1:]
            scans = robot_scans[i][:i] + robot_scans[i][i+1:]
            robot.particles.calculate_weight(angle_dists, guesses, scans)


        # Resample particles and update guesses
        for i, robot in enumerate(robots):
            if frame_id % 10:
                robot.particles.publish(frame_id)
            robot.particles.average()
            robot_guesses[i] = robot.particles.guess_position()

        # Log positions
        pose_history.append([(robot.pose[:2], guess[0]) for robot, guess in zip(robots, robot_guesses)])
        if len(pose_history) % 10:
            def entry_to_string(per_robot):
                return ','.join(','.join(str(v) for v in robot) + ',' + ','.join(str(v) for v in guess) for robot, guess in per_robot)

            with open('/tmp/data.csv', 'a') as fp:
                fp.write('\n'.join(entry_to_string(entry) for entry in pose_history) + '\n')

            pose_history = []

        # Loop again
        rate_limiter.sleep()
        frame_id += 1


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Runs a particle filter')
    parser.add_argument('--robots', action='store', type=int, help='Number of robots.')
    parser.add_argument('--cloud', action='store', type=str, help='Particle cloud module.')
    args, unknown = parser.parse_known_args()
    try:
        # cProfile.run('run(args)', '/tmp/profile-stats')
        run(args)
    except rospy.ROSInterruptException:
        pass
