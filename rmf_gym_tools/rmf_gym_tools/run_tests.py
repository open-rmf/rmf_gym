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
import time
import sys
import signal
import os
import fnmatch
import argparse
import collections
import subprocess
from pathlib import Path
import rclpy
from rclpy.node import Node
from rmf_task_msgs.srv import GetTaskList
from rclpy.parameter import Parameter


class GymTestNodeParams:
  def __init__(self, config_file, test_launch_path,
               use_ignition, gazebo_version,
               use_sim_time, failover_mode,
               no_simulation, headless,
               debug, get_tasks_topic):
    self.config_file = config_file
    self.test_launch_path = test_launch_path
    self.use_ignition = use_ignition
    self.gazebo_version = gazebo_version
    self.use_sim_time = use_sim_time
    self.failover_mode = failover_mode
    self.no_smulation = no_simulation
    self.headless = headless
    self.debug = debug
    self.get_tasks_topic = get_tasks_topic


class GymTestNode(Node):
  def __init__(self):
    self.rmf_process = None
    self.test_process = None

    super().__init__('gym_testing_node')

    self.declare_parameter('config_file')
    self.declare_parameter('test_launch_path')
    self.declare_parameter('use_ignition')
    self.declare_parameter('gazebo_version')
    self.declare_parameter('failover_mode')
    self.declare_parameter('no_simulation')
    self.declare_parameter('headless')
    self.declare_parameter('debug')
    self.declare_parameter('get_tasks_topic')

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
        self.get_parameter('headless').get_parameter_value().bool_value,
        self.get_parameter('debug').get_parameter_value().bool_value,
        self.get_parameter(
            'get_tasks_topic').get_parameter_value().string_value
    )

    self.get_task_list_srv = self.create_client(
        GetTaskList, self.params_config.get_tasks_topic)

    self.output_pipe = subprocess.PIPE
    if self.params_config.debug:
      self.output_pipe = None

    if not self._check_params_config():
      sys.exit(1)

  def _check_params_config(self):
    success = True
    # Check if all necessary files are present
    for world in self.params_config.config_file['worlds']:
      tests = self.params_config.config_file['worlds'][world]
      for fixture_name, test_list in tests.items():
        # Check if fixture exists
        try:
          open(
              f"{self.params_config.test_launch_path}/{world}/tests/{fixture_name}.launch.xml")
        except FileNotFoundError as e:
          self.get_logger().error(f"Missing Fixture: {fixture_name}")
          success = False

        for test_name in test_list:
          if test_name != "all":
            try:
              open(
                  f"{self.params_config.test_launch_path}/{world}/tests/{test_name}.launch.xml")
            except FileNotFoundError as e:
              self.get_logger().error(f"Missing Test: {test_name}")
              success = False

      return success

  def _get_task_srv_is_available(self):
    if not self.get_task_list_srv.wait_for_service(timeout_sec=3.0):
      self.get_logger().error('Task getting service is not available')
      return False
    else:
      return True

  def run_tests(self):
    self.get_logger().info("Running Tests.")

    for world in self.params_config.config_file['worlds']:
      tests = self.params_config.config_file['worlds'][world]
      for fixture_name, test_list in tests.items():
        if test_list[0] == 'all':
          test_list = fnmatch.filter(os.listdir(
              f"{self.params_config.test_launch_path}/{world}/tests"), "test_*.launch.xml")

          for test_name in test_list:
            if not self.run_test(world, fixture_name, test_name):
              self.get_logger().error(
                  f"FAILED: {world} {fixture_name} {test_name}")
            else:
              self.get_logger().error(
                  f"COMPLETED: {world} {fixture_name} {test_name}")
            reset(self)
    reset(self)

  def run_test(self, world, fixture_name, test_name):
    self.get_logger().info(
        f"\n\nTesting World: {world}\nFixture: {fixture_name}\nTest: {test_name}")

    try:
      self.get_logger().info(f"Launching {world} World..")
      self.rmf_process = subprocess.Popen(
          ['ros2', 'launch', 'rmf_gym_worlds',
           f"{world}.launch.xml", f"headless:={self.params_config.headless}"],
          stdout=self.output_pipe, stderr=self.output_pipe)
    except Exception as e:
      self.get_logger().error(
          f"ERROR: Encountered an error launching {world} World: {e}")
      return False

    self.get_logger().info("Checking the RMF Backend is up")
    max_retries = 5
    retries = 0
    while not self._get_task_srv_is_available():
      if retries < max_retries:
        self.get_logger().info("Waiting for backend to be ready..")
        retries += 1
        time.sleep(2)
      else:
        self.get_logger().error("ERROR: RMF Backend is not available")
        return False

    try:
      self.get_logger().info("Spawning Fixtures")
      subprocess.Popen(
          ['ros2', 'launch', 'rmf_gym_worlds',
           f"{fixture_name}.launch.xml"],
          stdout=self.output_pipe,
          stderr=self.output_pipe).communicate(timeout=10)
    except subprocess.TimeoutExpired:
      # Sometimes, gz just hangs. We should just ignore it
      pass
    except Exception as e:
      self.get_logger().error(
          f"ERROR: Encountered an error spawning {fixture_name} fixture: {e}")
      return False

    if not self.params_config.headless:
      subprocess.Popen(['wmctrl', '-a', 'rviz']).communicate()

    try:
      self.get_logger().info(f"Running {test_name} Test..")
      self.test_process = subprocess.Popen(
          ['ros2', 'launch', 'rmf_gym_worlds',
           f"{test_name}",
           f"use_sim_time:={self.params_config.use_sim_time}"]).communicate()
    except Exception as e:
      self.get_logger().error(
          f"ERROR: Encountered an error running {test_name} test: {e}")
      return False

    return True

def reset(node):
  node.get_logger().info("Terminating Test Instance..")
  try:
    if node.rmf_process is not None:
      for child in psutil.Process(node.rmf_process.pid).children(recursive=True):
        child.terminate()
        child.wait()
      node.rmf_process.terminate()
      node.rmf_process.wait()
  except psutil.NoSuchProcess:
    pass

  subprocess.Popen(
      ['ros2', 'daemon', 'stop'], stdout=node.output_pipe,
      stderr=node.output_pipe).communicate()
  subprocess.Popen(
      ['pkill', '-f', 'gz'], stdout=node.output_pipe,
      stderr=node.output_pipe).communicate()
  subprocess.Popen(
      ['pkill', '-f', 'gzclient'], stdout=node.output_pipe,
      stderr=node.output_pipe).communicate()
  subprocess.Popen(
      ['pkill', '-f', 'gzserver'], stdout=node.output_pipe,
      stderr=node.output_pipe).communicate()

  node.get_logger().info("Termination Done.")
  node.destroy_node()

  if os.getenv('RMW_IMPLEMENTATION', 'rmw_fastrtps_cpp') == 'rmw_fastrtps_cpp':
    subprocess.Popen(
        ['bash', 'fastdds', 'shm', 'clean'], stdout=node.output_pipe,
        stderr=node.output_pipe, cwd=f"/opt/ros/{os.getenv('ROS_DISTRO')}/bin").communicate()

  time.sleep(4)
  node.__init__()

def main(argv=sys.argv):
  rclpy.init()
  try:
    test_node = GymTestNode()
  except Exception as e:
    print(f"Something wrong happened with node initialization. {e}")

  try:
    test_node.run_tests()
  except psutil.NoSuchProcess:
    pass
  except KeyboardInterrupt:
    pass
  except Exception as e:
    test_node.get_logger().error(e.__str__())
  finally:
    reset(test_node)
    test_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
  main(sys.argv)
