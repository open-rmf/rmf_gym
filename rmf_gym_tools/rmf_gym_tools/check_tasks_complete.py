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

import sys
import time
import argparse
import random
import yaml
import math

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rmf_task_msgs.srv import GetTaskList
from rmf_task_msgs.msg import TaskType, Loop


class AllTasksCompleteCheckConfig:
  def __init__(self, get_task_topic,
               task_check_period, task_check_timeout):
    self.get_task_topic = get_task_topic
    self.task_check_period = task_check_period
    self.task_check_timeout = task_check_timeout


class AllTasksCompleteCheck:
  def __init__(self, argv=sys.argv):

    # Get all input parameters
    parser = argparse.ArgumentParser()
    parser.add_argument("--get_task_topic", type=str, default="get_tasks",
                        help='Topic to get tasks. default: get_tasks')
    parser.add_argument("--task_check_period", type=float, default=5.0,
                        help='Seconds between checks for task completion. default: 5.0')
    parser.add_argument("--task_check_timeout", type=float, default=300.0,
                        help='Seconds before deciding task is failed. default: 300.0')

    # initialization with given parameters
    self.args = parser.parse_args(argv[1:])

    self.config = AllTasksCompleteCheckConfig(
        self.args.get_task_topic,
        self.args.task_check_period,
        self.args.task_check_timeout,
    )

    # ROS2 Plumbing
    self.node = rclpy.create_node('all_tasks_complete_checker_node')
    self.get_task_list_srv = self.node.create_client(
        GetTaskList, self.config.get_task_topic)
    self.node.get_logger().info(
        f'All Task Complete Checker Configuration: {self.config.__dict__}')

  def _get_task_srv_is_available(self):
    if not self.get_task_srv.wait_for_service(timeout_sec=3.0):
      self.node.get_logger().error('Task getting service is not available')
      return False
    else:
      return True

  def main(self):
    time_elapsed = 0
    while True:
      # Check that at least one task is issued
      if not self._get_task_srv_is_available:
        time.sleep(1)
        continue
        
      if time_elapsed > 10:
        raise Exception(f"A task was never received.")
      
      req_msg = GetTaskList.Request()
      future = self.get_task_list_srv.call_async(req_msg)
      rclpy.spin_until_future_complete(
          self.node, future, timeout_sec=1.0)
      response = future.result()
      
      if response is None:
        continue
        
      if response.active_tasks:
        break
      else:
        self.node.get_logger().info(f"Waiting for a task to be issued. ({time_elapsed} sec elapsed)")
        self.node.get_logger().info(f"Active Tasks: {response.activate_tasks}")
        self.node.get_logger().info(f"Terminated Tasks: {response.terminated_tasks}")
        time.sleep(1)
        time_elapsed += 1
    
    time_elapsed = 0
    while True:
      time.sleep(self.config.task_check_period)
      time_elapsed += self.config.task_check_period

      # Check if services are available
      if not self._get_task_srv_is_available:
        # Failed to get service, retry loop
        time.sleep(1)
        continue

      # If timeout, then raise exception
      if time_elapsed > self.config.task_check_timeout:
        raise Exception(
            f" The failed to complete within the timeout duration of {self.config.task_check_timeout}")

      req_msg = GetTaskList.Request()
      future = self.get_task_list_srv.call_async(req_msg)
      rclpy.spin_until_future_complete(
          self.node, future, timeout_sec=1.0)
      response = future.result()

      if response is None:
        raise Exception('/get_task srv call failed')

      elif not response.success:
        raise Exception('Something wrong happened getting tasks')

      if response.active_tasks:
        self.node.get_logger().info(
            f'waiting for tasks {[x.task_id for x in response.active_tasks]} to complete. ({time_elapsed} sec elapsed)')
        self.node.get_logger().info(f"Active Tasks: {response.activate_tasks}")
        self.node.get_logger().info(f"Terminated Tasks: {response.terminated_tasks}")
      else:
        self.node.get_logger().info(
            f'Tasks completed.')
        return

###############################################################################


def main(argv=sys.argv):
  rclpy.init(args=sys.argv)
  task_checker = AllTasksCompleteCheck(sys.argv)
  try:
    task_checker.main()
    shutdown(0)
  except Exception as e:
    task_checker.node.get_logger().error(e.__str__())
    shutdown(1)


def shutdown(return_code):
  rclpy.shutdown()
  sys.exit(return_code)


if __name__ == '__main__':
  main(sys.argv)
