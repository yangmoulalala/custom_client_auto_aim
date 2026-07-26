# custom_client_auto_aim

RoboMaster 自定义客户端与自瞄的 ROS 2 集成项目。自瞄算法来自同济大学 SuperPower 战队
[`sp_vision_25`](https://github.com/TongjiSuperPower/sp_vision_25)，算法原理和原始硬件支持请查看
上游仓库。

| 目录 | 作用 |
| --- | --- |
| `auto_aim/` | 图像识别、跟踪、瞄准和火控 |
| `custom_client_adapter/` | UDP 视频解码与 MQTT 0x0310/0x0311 桥接 |

```text
UDP video -> rm_video -> /rm_video/image_processed --------+
MQTT 0310 -> rm_mqtt  -> /rm_mqtt/imu ---------------------+-> auv_client
                         /rm_mqtt/self_is_red --------------+       |
                                                                    +-> /auto_aim/debug
                                                                    v
MQTT 0311 <- rm_mqtt  <- /auto_aim/result <-------------------------+
```

## 安装依赖

适用于 x86_64 Ubuntu 24.04。请先安装 ROS 2，并设置 `ROS_DISTRO`。

安装构建工具、自瞄和适配器依赖：

```bash
sudo apt update
sudo apt install -y \
  build-essential cmake git curl gnupg pkg-config \
  libopencv-dev libfmt-dev libeigen3-dev libspdlog-dev \
  libyaml-cpp-dev nlohmann-json3-dev libceres-dev libusb-1.0-0-dev \
  libavcodec-dev libavutil-dev libswscale-dev libjpeg-dev
```

安装 MQTT Python 依赖：

```bash
sudo apt install -y python3-paho-mqtt python3-protobuf
```

安装测试依赖：

```bash
sudo apt install -y python3-pytest
```

安装 [OpenVINO 2024.6](https://docs.openvino.ai/2024/get-started/install-openvino/install-openvino-apt.html)：

```bash
curl -fsSL \
  https://apt.repos.intel.com/intel-gpg-keys/GPG-PUB-KEY-INTEL-SW-PRODUCTS.PUB \
  | gpg --dearmor \
  | sudo tee /usr/share/keyrings/intel-openvino.gpg >/dev/null
echo "deb [signed-by=/usr/share/keyrings/intel-openvino.gpg] https://apt.repos.intel.com/openvino/2024 ubuntu24 main" \
  | sudo tee /etc/apt/sources.list.d/intel-openvino-2024.list
sudo apt update
sudo apt install -y openvino-2024.6.0
```

默认 UDP 视频链路无需额外安装相机 SDK。

## 构建与启动

加载 ROS 2 环境：

```bash
source "/opt/ros/${ROS_DISTRO}/setup.bash"
```

按需修改配置：

| 配置 | 内容 |
| --- | --- |
| `custom_client_adapter/custom_client_adapter/config/rm_video.yaml` | 视频端口、编码、ROI、旋转、亮度和时间补偿 |
| `custom_client_adapter/custom_client_adapter/config/rm_mqtt.yaml` | MQTT broker、话题、限频和开火许可 |
| `auto_aim/configs/AUVClient.yaml` | 模型、同步、相机内参、手眼标定和开火许可 |

构建并启动 `rm_video`、`rm_mqtt` 和 `auv_client --debug`：

```bash
./start.sh 3  # 3 为正整数 MQTT client_id
```

调试图像发布到 `/auto_aim/debug`，不创建本地窗口。不传 `client_id` 时脚本会交互读取；按
`Ctrl+C` 停止全部进程。

## 组件调试

自瞄：

```bash
source "/opt/ros/${ROS_DISTRO}/setup.bash"
cmake -S auto_aim -B build/auto_aim -DCMAKE_BUILD_TYPE=Release
cmake --build build/auto_aim -j "$(nproc)"
./build/auto_aim/auv_client auto_aim/configs/AUVClient.yaml --debug
```

调试图像发布到 `/auto_aim/debug`，不创建本地窗口。

适配器：

```bash
source "/opt/ros/${ROS_DISTRO}/setup.bash"
colcon --log-base log build --base-paths custom_client_adapter \
  --build-base build/custom_client_adapter --install-base install \
  --symlink-install --packages-select custom_client_adapter
source install/setup.bash
ros2 launch custom_client_adapter custom_client_adapter.launch.py client_id:=3
```

## 联调验证

```bash
ros2 node list
ros2 topic hz /rm_video/image_processed
ros2 topic hz /rm_mqtt/imu
ros2 topic echo --qos-reliability best_effort --once /rm_mqtt/self_is_red
ros2 topic echo --qos-reliability best_effort --once /auto_aim/result
ros2 topic hz /auto_aim/debug
```

使用 [ROSboard](https://github.com/dheera/rosboard) 网页调试话题（本项目不包含该工具）：

```bash
git clone https://github.com/dheera/rosboard.git
python3 -m pip install tornado simplejpeg --break-system-packages
./rosboard/run
```

启动后，在浏览器中访问 `http://<机器人IP>:8888/`。

- `rm_video` 关注 `fps/udp/dropped/error`。
- `rm_mqtt` 关注 `rx/rx_error/tx/tx_error` 和 `MQTT disconnected`。
- `auv_client --debug` 每秒输出输入、识别、跟踪和控制统计；收到阵营前只输出安全结果。

## 测试

```bash
ctest --test-dir build/auto_aim --output-on-failure

colcon --log-base log test --build-base build/custom_client_adapter \
  --install-base install --packages-select custom_client_adapter
colcon test-result --test-result-base build/custom_client_adapter --verbose
```

相关文档：

- [适配器运行、参数与协议](custom_client_adapter/README.md)
- [AUV Client 接口、同步、标定与安全约定](docs/auv_client_ros_manual.md)
- [项目协作约定](AGENTS.md)
