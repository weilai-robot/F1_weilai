#!/bin/bash
# run.sh — 薄 wrapper, 委托 run_main.sh（带真机通信验证配置 x1_cfg.yaml）
# 原 FastDDS shm 清理/log 属主防御已并入 run_main.sh 开头；颈部/setcap/local_setup
# 逻辑随 X1 实时优化机启动套件（run_main.sh + checks/）统一维护，只此一份。
# 用法: bash run.sh [run_main.sh 的任意参数，如 --check-only]
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "${SCRIPT_DIR}"
exec bash ./run_main.sh --cfg ./cfg/x1_cfg.yaml "$@"
