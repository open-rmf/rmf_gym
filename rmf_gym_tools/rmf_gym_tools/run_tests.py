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
import os
import fnmatch
import argparse
import collections
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter


class GymTestNodeParams:
  def __init__(self, config_file, test_launch_path,
               use_ignition, gazebo_version,
               use_sim_time, failover_mode,
               no_simulation, headless):
    self.config_file = config_file
    self.test_launch_path = test_launch_path
    self.use_ignition = use_ignition
    self.gazebo_version = gazebo_version
    self.use_sim_time = use_sim_time
    self.failover_mode = failover_mode
    self.no_smulation = no_simulation
    self.headless = headless


class GymTestNode(Node):
  def __init__(self):
    super().__init__('gym_testing_node')
    self.declare_parameter('config_file')
    self.declare_parameter('test_launch_path')
    self.declare_parameter('use_ignition')
    self.declare_parameter('gazebo_version')
    self.declare_parameter('failover_mode')
    self.declare_parameter('no_simulation')
    self.declare_parameter('headless')

    config_file_path = self.get_parameter(
        'config_file').get_parameter_value().string_value
    with open(config_file_path) as config_file:
      config_yaml = yaml.load(config_file, Loader=yaml.FullLoader)

    self.params_config = GymTestNodeParams(
        config_yaml,
        self.get_parameter(
            'test_launch_path').get_parameter_value().string_value,
        self.get_parameter('use_ignition').get_parameter_value().bool_value,
        self.get_parameter(
            'gazebo_version').get_parameter_value().integer_value,
        self.get_parameter('use_sim_time').get_parameter_value().bool_value,
        self.get_parameter('failover_mode').get_parameter_value().bool_value,
        self.get_parameter('no_simulation').get_parameter_value().bool_value,
        self.get_parameter('headless').get_parameter_value().bool_value
    )

    ready = self.check_params_config()

    if not ready:
      sys.exit(1)
    else:
      self.get_logger().info("All test files found")

    self.run_tests()

  def run_tests(self):
    for world in self.params_config.config_file['worlds']:
      tests = self.params_config.config_file['worlds'][world]
      for fixture_name, test_list in tests.items():
        if test_list[0] == 'all':
          test_list = fnmatch.filter(os.listdir(f"{self.params_config.test_launch_path}/{world}/tests"), "test_*.launch.xml")
          test_list = [x.replace('.launch.xml', '') for x in test_list]

        for test_name in test_list:
          self.get_logger().info(f"\n\nTesting World: {world}\nFixture: {fixture_name}\nTest: {test_name}")
          self.get_logger().info(f"Done.\n\n")

  def check_params_config(self):
    success = True
    # Check if all necessary files are present
    for world in self.params_config.config_file['worlds']:
      tests = self.params_config.config_file['worlds'][world]
      for fixture_name, test_list in tests.items():
        # Check if fixture exists
        try:
          open(f"{self.params_config.test_launch_path}/{world}/tests/{fixture_name}.launch.xml")
        except FileNotFoundError as e:
          self.get_logger().error(f"Missing Fixture: {fixture_name}")
          success = False

        for test_name in test_list:
          if test_name != "all":
            try:
              open(f"{self.params_config.test_launch_path}/{world}/tests/{test_name}.launch.xml")
            except FileNotFoundError as e:
              self.get_logger().error(f"Missing Test: {test_name}")
              success = False

      return success

def main(argv=sys.argv):
  rclpy.init()
  test_node = GymTestNode()


if __name__ == '__main__':
  main(sys.argv)
