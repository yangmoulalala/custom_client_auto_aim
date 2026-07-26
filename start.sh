#!/bin/bash
set -eo pipefail

# 无论从哪里执行脚本，都先切换到仓库根目录。
repo_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
cd "${repo_dir}"

# client_id 可作为第一个参数传入，否则循环等待用户输入正整数。
client_id="${1:-}"
if (( $# > 0 )); then
    shift
fi

while [[ ! "${client_id}" =~ ^[1-9][0-9]*$ ]]; do
    [[ -z "${client_id}" ]] || echo "Error: MQTT client ID must be a positive integer." >&2
    if ! read -r -p "Enter MQTT client ID (positive integer): " client_id; then
        exit 1
    fi
done

# 默认使用 Jazzy，也可在执行脚本前通过 ROS_DISTRO 指定其他版本。
ros_distro="${ROS_DISTRO:-jazzy}"
source "/opt/ros/${ros_distro}/setup.bash"

# 先完成自瞄和适配器的编译，任一步失败都不会启动节点。
cmake -S auto_aim -B build/auto_aim -DCMAKE_BUILD_TYPE=Release
cmake --build build/auto_aim -j "$(nproc)"

colcon --log-base log build \
    --base-paths custom_client_adapter \
    --build-base build/custom_client_adapter \
    --install-base install \
    --symlink-install --packages-select custom_client_adapter
source install/setup.bash

# 两端都编译成功后，再同时启动适配器和自瞄。
ros2 launch custom_client_adapter custom_client_adapter.launch.py \
    "client_id:=${client_id}" "$@" &
adapter_pid=$!

./build/auto_aim/auv_client auto_aim/configs/AUVClient.yaml --debug &
auto_aim_pid=$!

# Ctrl+C 或任一进程退出时，同时清理两个进程。
cleanup() {
    trap - EXIT INT TERM
    kill "${adapter_pid}" "${auto_aim_pid}" 2>/dev/null || true
    wait "${adapter_pid}" "${auto_aim_pid}" 2>/dev/null || true
}

trap cleanup EXIT INT TERM
wait -n "${adapter_pid}" "${auto_aim_pid}"
