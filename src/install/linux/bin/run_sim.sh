#!/bin/bash

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/ros2_setup.sh"

./aimrt_main --cfg_file_path=./cfg/x1_cfg_sim.yaml
# gdb --args ./aimrt_main --cfg_file_path=./cfg/x1_cfg_sim.yaml
