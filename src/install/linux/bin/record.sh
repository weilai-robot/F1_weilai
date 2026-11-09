#!/bin/bash
###
 # @Author: richie.li
 # @Date: 2024-11-18 14:41:20
 # @LastEditors: richie.li
 # @LastEditTime: 2024-11-18 17:48:16
### 

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/ros2_setup.sh"

current_time=$(date "+%Y-%m-%d_%H:%M:%S")

ros2 launch ros2_utils record.launch.py dir:=./log/ros_bags/$current_time
