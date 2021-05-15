#!/usr/bin/env python3

# Copyright 2021 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import yaml
import sys
import subprocess
import argparse
import collections
from pathlib import Path


def main(argv=sys.argv):
  parser = argparse.ArgumentParser()
  parser.add_argument("--building_path", required=True, type=str,
                      help='Path to building.yaml file')
  parser.add_argument("--nav_graph_path", required=True, type=str,
                      help='Path to nav_graph file')
  parser.add_argument("--waypoint_name", required=True, type=str,
                      help='Name of waypoint location to spawn robot')
  parser.add_argument("--sdf_path", required=True, type=str,
                      help='Path to robot SDF')
  parser.add_argument("--robot_name", required=True, type=str,
                      help='Name of robot to spawn')

  parsed_args = parser.parse_args(argv[1:])

  subprocess.Popen(['ros2', 'run', 'rmf_gym_tools', 'get_waypoint_location',
                    '--building_path', parsed_args.building_path,
                    '--nav_graph_path', parsed_args.nav_graph_path,
                    '--waypoint_name', parsed_args.waypoint_name,
                    '--output_path', f"/tmp/get_waypoints/{parsed_args.waypoint_name}"]).communicate()

  with open(f"/tmp/get_waypoints/{parsed_args.waypoint_name}/{parsed_args.waypoint_name}.yml") as config_file:
    graph_yaml = yaml.load(config_file, Loader=yaml.FullLoader)

  subprocess.Popen(['ros2', 'run', 'rmf_gym_tools', 'spawn_robot',
                    '--config_path', f"/tmp/get_waypoints/{parsed_args.waypoint_name}/{parsed_args.waypoint_name}.yml",
                    '--sdf_path', f"{parsed_args.sdf_path}",
                    '--robot_name', f"{parsed_args.robot_name}"]).communicate()


if __name__ == '__main__':
  main(sys.argv)
