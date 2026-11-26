#!/bin/bash

# ROS2 环境自动探测与 source（X1 实时优化机为 Humble，适配 apt/conda 安装，
# 探测优先级详见同目录 ros2_source.sh；Jazzy 时期写死的路径已废弃）
_script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${_script_dir}/ros2_source.sh"

if [ -f ./build/install/share/ros2_plugin_proto/local_setup.bash ]; then
    source ./build/install/share/ros2_plugin_proto/local_setup.bash
elif [ -f ./install/share/ros2_plugin_proto/local_setup.bash ]; then
    source ./install/share/ros2_plugin_proto/local_setup.bash
elif [ -f ../share/ros2_plugin_proto/local_setup.bash ]; then
    source ../share/ros2_plugin_proto/local_setup.bash
fi

# Add project libraries to LD_LIBRARY_PATH
if [ -d ./install/lib ]; then
    export LD_LIBRARY_PATH=./install/lib:${LD_LIBRARY_PATH}
elif [ -d ./build/install/lib ]; then
    export LD_LIBRARY_PATH=./build/install/lib:${LD_LIBRARY_PATH}
fi
