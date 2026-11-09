#!/bin/bash

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/ros2_setup.sh"

# 防御：清掉之前可能由 root 跑残留的 FastDDS 共享内存段（否则 DDS 数据发不出）
sudo rm -f /dev/shm/fastrtps_* 2>/dev/null
# log 目录若被 root 占了，改回当前用户属主
sudo chown -R "$(id -u):$(id -g)" "${SCRIPT_DIR}/log" 2>/dev/null

sudo setcap cap_net_raw=ep ./aimrt_main
./aimrt_main --cfg_file_path=./cfg/x1_cfg.yaml
