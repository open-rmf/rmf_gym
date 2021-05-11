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
                      help='Name of the waypoint you want to get the (x,y) coordinates of')
  parser.add_argument("--output_path", required=True, type=str,
                      help='Path to output folder')

  parsed_args = parser.parse_args(argv[1:])
  with open(parsed_args.building_path) as building_config_file:
    building_yaml = yaml.load(building_config_file, Loader=yaml.FullLoader)
  with open(parsed_args.nav_graph_path) as nav_config_file:
    nav_graph_yaml = yaml.load(nav_config_file, Loader=yaml.FullLoader)

  for level in building_yaml['levels']:
    level_vertices = nav_graph_yaml['levels'][level]['vertices']
    for vertex in level_vertices:
      if vertex[2]['name'] == parsed_args.waypoint_name:
        waypoint = {'x': vertex[0], 'y': vertex[1], 'z': building_yaml['levels'][level]['elevation']}

  Path(parsed_args.output_path).mkdir(parents=True, exist_ok=True)
  with open(f'{parsed_args.output_path}/{parsed_args.waypoint_name}.yml', 'w') as output_file:
    yaml.dump(waypoint, output_file)

if __name__ == '__main__':
  main(sys.argv)
