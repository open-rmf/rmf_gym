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
import psutil
import sys
import time
import subprocess
import argparse
import collections
from pathlib import Path
import rclpy
from rclpy.node import Node


def main(argv=sys.argv):
  parser = argparse.ArgumentParser()
  parser.add_argument("--config_path", required=True, type=str,
                      help='Path to config file')
  parser.add_argument("--sdf_path", required=True, type=str,
                      help='Path to robot SDF')
  parser.add_argument("--robot_name", required=True, type=str,
                      help='Name of robot to spawn')

  parsed_args = parser.parse_args(argv[1:])

  with open(parsed_args.config_path) as config_file:
    graph_yaml = yaml.load(config_file, Loader=yaml.FullLoader)

  while True:
    out, err = subprocess.Popen(
        ['gz', 'topic'], stdout=subprocess.PIPE, stderr=subprocess.PIPE).communicate()
    if "not running" in str(err):
      time.sleep(2)
      print("Gazebo not running. Retrying..")
      continue
    else:
      break

  subprocess.Popen(
      ['gz', 'model',  '-f', parsed_args.sdf_path, '-m', parsed_args.robot_name,
       '-x', str(graph_yaml['x']), '-y', str(graph_yaml['y']), '-z',
       str(graph_yaml['z'])]).communicate()


if __name__ == '__main__':
  main(sys.argv)
  rclpy.shutdown()
