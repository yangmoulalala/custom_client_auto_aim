#!/bin/bash
set -e

# 固定以仓库根目录为工作目录，避免从其他目录调用时相对路径失效。
cd -- "$(dirname -- "${BASH_SOURCE[0]}")"

# GPU 推理不能在用户态库与内核驱动不一致时安全启动。驱动更新后通常需要重启。
if ! nvidia_smi_output="$(nvidia-smi 2>&1)"; then
    echo "NVIDIA driver is not ready; GPU inference cannot start." >&2
    echo "${nvidia_smi_output}" >&2
    if [[ -e /var/run/reboot-required ]]; then
        echo "A system reboot is required. Reboot, verify nvidia-smi, then run ./start.sh again." >&2
    fi
    exit 1
fi

# 加载调用环境选定的 ROS 2 发行版。
source "/opt/ros/$ROS_DISTRO/setup.bash"

# 先将自瞄和适配器构建到仓库根目录的统一产物目录，任一步失败都不启动节点。
cmake -S auto_aim -B build/auto_aim -DCMAKE_BUILD_TYPE=Release
cmake --build build/auto_aim -j "$(nproc)"

colcon build --base-paths custom_client_adapter --build-base build/custom_client_adapter --symlink-install
source install/setup.bash

# 构建全部成功后再同时启动适配器与自瞄，防止只启动半条控制链路。
ros2 launch custom_client_adapter custom_client_adapter.launch.py &
adapter_pid=$!

# 自瞄端固定启用调试输出，仅识别可选的 --show，其他脚本参数不转发。
custom_client_args=(auto_aim/configs/custom_client.yaml --debug)
for arg; do
    if [[ "${arg}" == "--show" ]]; then
        custom_client_args+=(--show)
        break
    fi
done
./build/auto_aim/custom_client "${custom_client_args[@]}" &
custom_client_pid=$!

# Ctrl+C 或任一子进程退出时都终止并回收两端，避免留下单独运行的节点。
cleanup() {
    trap - EXIT INT TERM
    kill "${adapter_pid}" "${custom_client_pid}" 2>/dev/null || true
    wait "${adapter_pid}" "${custom_client_pid}" 2>/dev/null || true
}

trap cleanup EXIT INT TERM
wait -n "${adapter_pid}" "${custom_client_pid}"
