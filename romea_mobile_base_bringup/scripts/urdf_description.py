#!/usr/bin/env python3
from romea_mobile_base_bringup import urdf_description
from romea_common_bringup import robot_urdf_prefix
import sys

if __name__ == "__main__":

  argv = sys.argv

  parameters = {}
  for argument in argv[1:]:
       name, value = argument.split(':')
       parameters[name] = value

  mode=parameters["mode"]
  prefix = robot_urdf_prefix(parameters["robot_namespace"])
  meta_description_filename=parameters["meta_description_filename"]
  print(urdf_description(prefix,mode,meta_description_filename))
