#!/usr/bin/env python3
import romea_mobile_base_bringup
import sys

if __name__ == "__main__":

  argv = sys.argv

  parameters = {}
  for argument in argv[1:]:
       name, value = argument.split(':')
       parameters[name] = value


  mode=parameters["mode"]
  if not parameters["robot_namespace"] :
    prefix="";
  else:
    prefix=parameters["robot_namespace"]+"_";

  meta_description_filename=parameters["meta_description_filename"]

  print(romea_mobile_base_bringup.urdf_description(prefix,mode,meta_description_filename))
