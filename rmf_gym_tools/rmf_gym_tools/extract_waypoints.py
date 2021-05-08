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

def main(argv=sys.argv):
  parser = argparse.ArgumentParser()
  parser.add_argument("--building_path", required=True, type=str,
                      help='Path to building.yaml file')

  parsed_args = parser.parse_args(argv[1:])
  with open(parsed_args.building_path) as config_file:
    building_yaml = yaml.load(config_file, Loader=yaml.FullLoader)

  # dict mapping graph indices to waypoints
  waypoints = collections.defaultdict(set)

  for level in building_yaml['levels']:
    level_lanes = building_yaml['levels'][level]['lanes']
    level_vertices = building_yaml['levels'][level]['vertices']

    for lane in level_lanes:
      graph_idx = lane[2]['graph_idx'][1]
      source_vertex = level_vertices[lane[0]][3]
      dest_vertex = level_vertices[lane[1]][3]

      waypoints[graph_idx].add(source_vertex)
      waypoints[graph_idx].add(dest_vertex)

      waypoints[f'{graph_idx}_{level}'].add(source_vertex)
      waypoints[f'{graph_idx}_{level}'].add(dest_vertex)

  for graph_index, waypoint_set in waypoints.items():
    try:
      waypoint_set.remove("")
    except KeyError:
      pass

    with open(f'{graph_index}.yml', 'w') as output_file:
      yaml.dump(list(waypoint_set), output_file)

if __name__ == '__main__':
  main(sys.argv)
