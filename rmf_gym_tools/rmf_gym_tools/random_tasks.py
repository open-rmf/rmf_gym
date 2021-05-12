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
import os
import subprocess
import uuid
import time
import argparse
import random
import yaml
import math

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rmf_task_msgs.srv import SubmitTask, GetTaskList
from rmf_task_msgs.msg import TaskType, Loop


class TaskGeneratorConfig:
  def __init__(
          self, task_type, task_config_file,
          task_priority, task_count, use_sim_time,
          submit_task_topic, get_task_topic,
          task_check_period, task_check_timeout,
          log_label):
    self.task_type = task_type
    self.task_config_file = task_config_file
    self.task_priority = task_priority
    self.task_count = task_count
    self.use_sim_time = use_sim_time
    self.submit_task_topic = submit_task_topic
    self.get_task_topic = get_task_topic
    self.task_check_period = task_check_period
    self.task_check_timeout = task_check_timeout
    self.log_label = log_label


class TaskGenerator:
  def __init__(self, argv=sys.argv):

    # Get all input parameters
    parser = argparse.ArgumentParser()
    parser.add_argument("--use_sim_time", required=True, type=bool, 
                        help='Use sim time')
    parser.add_argument("--task_type", required=True, type=str,
                        help='Type of Task we are issuing')
    parser.add_argument("--task_config_path", required=True, type=str,
                        help='path to YAML config file for sepcified task')
    parser.add_argument("--task_priority", type=int, default=0,
                        help='Priority for publishing tasks, default: 0')
    parser.add_argument("--task_count", type=int, default=None,
                        help='Number of tasks to issue, default: None ( infinite )')
    parser.add_argument("--submit_task_topic", type=str, default="submit_task",
                        help='Topic to submit tasks. default: submit_task')
    parser.add_argument("--get_task_topic", type=str, default="get_tasks",
                        help='Topic to get tasks. default: get_tasks')
    parser.add_argument("--task_check_period", type=float, default=5.0,
                        help='Seconds between checks for task completion. default: 5.0')
    parser.add_argument("--task_check_timeout", type=float, default=300.0,
                        help='Seconds before deciding task is failed. default: 300.0')
    parser.add_argument("--log_label", type=str, default="log",
                        help='Label to identify logs from this instance. default: "log"')

    # initialization with given parameters
    self.args = parser.parse_args(argv[1:])

    with open(self.args.task_config_path) as config_file:
      task_config_file = yaml.load(config_file, Loader=yaml.FullLoader)

    if selfs.args.use_sim_time.lower() == "false" or self.args.use_sim_time == "0":
      use_sim_time = False
    else:
      use_sim_time = True

    self.config = TaskGeneratorConfig(
        self.args.task_type,
        task_config_file,
        self.args.task_priority,
        math.inf if self.args.task_count is None else self.args.task_count,
        use_sim_time,
        self.args.submit_task_topic,
        self.args.get_task_topic,
        self.args.task_check_period,
        self.args.task_check_timeout,
        self.args.log_label
    )

    # ROS2 Plumbing
    self.node = rclpy.create_node('task_generator_node')
    self.node.set_parameters(
        [Parameter("use_sim_time", Parameter.Type.BOOL, self.config.use_sim_time)])
    self.submit_task_srv = self.node.create_client(
        SubmitTask, self.config.submit_task_topic)
    self.get_task_list_srv = self.node.create_client(
        GetTaskList, self.config.get_task_topic)
    self.node.get_logger().info(
        f'Task Generator Configuration: {self.config.__dict__}')

    # Book Keeping
    self.task_count_left = self.config.task_count

  def _submit_task_srv_is_available(self):
    if not self.submit_task_srv.wait_for_service(timeout_sec=3.0):
      self.node.get_logger().error('Dispatcher Node is not available')
      return False
    else:
      return True

  def _generate_loop_task(self):
    random_points = random.sample(self.config.task_config_file, 1)
    start_wp = random_points[0]
    finish_wp = random_points[0]
    self.node.get_logger().info(f"Generated Random Loop Task to {finish_wp}")

    req_msg = SubmitTask.Request()
    req_msg.description.task_type.type = TaskType.TYPE_LOOP

    loop = Loop()
    loop.num_loops = 1
    loop.start_name = start_wp
    loop.finish_name = finish_wp
    req_msg.description.loop = loop

    ros_start_time = self.node.get_clock().now().to_msg()
    req_msg.description.start_time = ros_start_time
    req_msg.description.priority.value = self.config.task_priority
    return req_msg

  def _generate_task_req_msg(self):
    if self.config.task_type.lower() == 'loop':
      return self._generate_loop_task()
    else:
      raise Exception(f"Task type {self.config.task_type} not supported.")

  def _submit_task_req_msg(self, req_msg):
    future = self.submit_task_srv.call_async(req_msg)
    rclpy.spin_until_future_complete(
        self.node, future, timeout_sec=1.0)
    response = future.result()

    if response is None:
      raise Exception('/submit_task srv call failed')
    elif not response.success:
      raise Exception('Dispatcher node failed to accept task')
    else:
      self.node.get_logger().info(
          'Request was successfully submitted '
          f'and assigned task_id: [{response.task_id}]')
    return response.task_id

  def _await_task_completion(self, task_to_track):
    time_elapsed = 0

    while True:
      if time_elapsed > self.config.task_check_timeout:
        raise Exception(
            f"Task failed to complete within timeout duration of {self.config.task_check_timeout}")

      # Get current task list
      req_msg = GetTaskList.Request()
      future = self.get_task_list_srv.call_async(req_msg)
      rclpy.spin_until_future_complete(
          self.node, future, timeout_sec=1.0)
      response = future.result()

      if response is None:
        raise Exception('/get_task srv call failed')

      elif not response.success:
        raise Exception('Something wrong happened getting tasks')

      if any(task_to_track == x.task_profile.task_id for x in response.active_tasks):
        self.node.get_logger().info(
            f'waiting for task {task_to_track} to complete. ({time_elapsed} sec elapsed)')
      else:
        self.node.get_logger().info(
            f'Task {task_to_track} completed.')
        return

      time.sleep(self.config.task_check_period)
      time_elapsed += self.config.task_check_period

  def _handle_task_start(self, task_to_track):
    log_process = []

    subprocess.Popen(['mkdir', '-p',  f'{self.config.log_label}'])
    bag_process = subprocess.Popen(
        ['ros2', 'bag', 'record', '-a', '-o',
            f'{self.config.log_label}/{task_to_track}.bag'],
        shell=False, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    log_process.append(bag_process)

    return log_process

  def _handle_task_failure(self, task_to_track, log_process):
    [process.kill() for process in log_process]
    [process.wait() for process in log_process]
    subprocess.call(["stty", "echo"])
    # Implement your own alert methods here
    subprocess.Popen(['mv', f'{self.config.log_label}/{task_to_track}.bag',
                     f'{self.config.log_label}/{task_to_track}-failed.bag']).communicate()
    shutdown(1)

  def _handle_task_success(self, task_to_track, log_process):
    [process.kill() for process in log_process]
    [process.wait() for process in log_process]
    subprocess.call(["stty", "echo"])

    # Remove bags to save space
    subprocess.Popen(
        ['rm', '-r', f'{self.config.log_label}/{task_to_track}.bag'])

  def main(self):
    while self.task_count_left > 0:

      if not self._submit_task_srv_is_available:
        # Failed to get service, retry loop
        continue

      rclpy.spin_once(self.node, timeout_sec=1.0)
      req_msg = self._generate_task_req_msg()
      try:
        task_to_track = self._submit_task_req_msg(req_msg)
      except Exception as e:
        # Failed to submit task, retry loop
        self.node.get_logger().error(e.__str__())
        continue

      try:
        log_process = self._handle_task_start(task_to_track)
        self._await_task_completion(task_to_track)

      except Exception as e:
        # Task Failure, collect data!
        self.node.get_logger().error(e.__str__())
        self._handle_task_failure(task_to_track, log_process)

      self.task_count_left -= 1
      self._handle_task_success(task_to_track, log_process)
      time.sleep(0.5)

    self.node.get_logger().info(f"Finished {iterations} requests")

###############################################################################


def main(argv=sys.argv):
  rclpy.init(args=sys.argv)
  task_generator = TaskGenerator(sys.argv)
  task_generator.main()
  shutdown(0)


def shutdown(return_code):
  rclpy.shutdown()
  sys.exit(return_code)


if __name__ == '__main__':
  main(sys.argv)
