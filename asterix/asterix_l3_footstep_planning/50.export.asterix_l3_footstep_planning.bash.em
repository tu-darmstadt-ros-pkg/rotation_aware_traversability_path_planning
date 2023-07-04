#!/bin/bash

if [ "$(type -t add_rosrs_setup_env)" == "function" ]; then
  add_rosrs_setup_env HECTOR_USE_L3_PLANNER "true=1,false=0" "Additionally start the L3 path planner with rotation-aware traversability-based planning."
fi
