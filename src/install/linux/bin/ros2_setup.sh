#!/bin/bash

# Source ROS2 Jazzy environment
if [ -f /opt/ros/jazzy/setup.bash ]; then
    source /opt/ros/jazzy/setup.bash
else
    echo "Warning: ROS2 Jazzy not found at /opt/ros/jazzy/setup.bash"
    echo "Please install ROS2 Jazzy: https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debians.html"
fi

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
