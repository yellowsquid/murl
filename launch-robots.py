#!/usr/bin/env python3

import argparse
import math
from shutil import which
import subprocess
import sys
from typing import Tuple

ORIGIN = [0.0, 0.0]
DISTANCE = 1.5

TEMPLATE = """
  <group ns="robot{n}">
    <param name="tf_prefix" value="robot{n}" />
    <include file="$(find murl)/launch/one-robot.launch">
      <arg name="robot_name" value="Robot{n}"/>
      <arg name="x_pos" value="{x}"/>
      <arg name="y_pos" value="{y}"/>
      <arg name="z_pos" value="{z}"/>
    </include>
  </group>
"""

MAGIC_STRING = "<!-- INCLUDE ROBOTS -->"

def get_pose(i : int, n : int) -> Tuple[float, float, float]:
    angle = 2 * math.pi * i / n
    x = ORIGIN[0] + DISTANCE * math.cos(angle)
    y = ORIGIN[1] + DISTANCE * math.sin(angle)
    z = 0.0
    return (x , y , z)


def make_robot_groups(n : int) -> str:
    output = ""
    for i in range(n):
        x , y , z = get_pose(i, n)
        output += TEMPLATE.format(
            n = i,
            x = round(x, 3),
            y = round(y, 3),
            z = round(z, 3))
    return output


def splice_robots(text : str, n : int) -> str:
    return text.replace(MAGIC_STRING, make_robot_groups(n))


def main(filename : str, n : int):
    out = None
    with open(filename, 'r') as fd:
        out = splice_robots(fd.read(), n)

    subprocess.run([which('roslaunch'), '-'], input=out, text=True)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Launches gazebo')
    parser.add_argument('--template', action='store', type=str, help='Template launch file.')
    parser.add_argument('--robots', action='store', type=int, help='Number of robots.')
    args, unknown = parser.parse_known_args()
    main(args.template, args.robots)
