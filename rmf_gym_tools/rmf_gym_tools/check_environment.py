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

import argparse
import sys
import os

def main(argv=sys.argv):
  # Check that the RMF environment is properly set up
  parser = argparse.ArgumentParser()
  parser.add_argument("--ros_version", type=str, required=True,
                      help='ROS version you are on')
  parser.add_argument("--ros_domain_id", type=str, required=True,
                      help='Domain ID you are on')
  parser.add_argument("--dds_version", type=str, required=True,
                      help='DDS you are using.')

  args = parser.parse_args(argv[1:])
  success = True
  try:
    distro = os.environ['ROS_DISTRO']
    if distro != args.ros_version:
      print(f"Wrong ROS version: Required {args.ros_version}, found {distro}.")
      success = False
  except KeyError:
    print(f"It seems you have not sourced your environment. source /opt/ros/{args.ros_version}/setup.bash")
    success = False

  domain_id = os.environ.get('ROS_DOMAIN_ID', 0)
  if domain_id != args.ros_domain_id:
    print(f"Wrong ROS Domain ID: Required {args.ros_domain_id}, found {domain_id}.")
    success = False

  dds_version = os.environ.get('RMW_IMPLEMENTATION', 'rmw_fastrtps_cpp')
  if dds_version != args.dds_version:
    print(f"Wrong ROS DDS Version: Required {args.dds_version}, found {dds_version}.")

  try:
    os.environ['AMENT_PREFIX_PATH']
  except KeyError:
    print(f"It seems you have not sourced your environment. source /opt/ros/{args.ros_version}/setup.bash")
    success = False

  if success:
    print("Success")
    sys.exit(0)
  else:
    sys.exit(1)
