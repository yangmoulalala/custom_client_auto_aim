# custom_client_auto_aim

RoboMaster 自定义客户端与自瞄的 ROS 2 集成项目。自瞄算法来自同济大学 SuperPower 战队
[`sp_vision_25`](https://github.com/TongjiSuperPower/sp_vision_25)，算法原理和原始硬件支持请查看
上游仓库。

| 目录 | 作用 |
| --- | --- |
| `auto_aim/` | 图像识别、跟踪、瞄准和火控 |
| `custom_client_adapter/` | UDP 视频解码与 MQTT 0x0310/0x0311 桥接 |
| `rosboard/` | 可选的 ROS 2 话题 Web 可视化 |

```text
UDP video -> rm_video -> /rm_video/image_processed --------+
MQTT 0310 -> rm_mqtt  -> /rm_mqtt/imu ---------------------+-> auv_client
                         /rm_mqtt/self_is_red --------------+       |
                                                                    v
MQTT 0311 <- rm_mqtt  <- /auto_aim/result <-------------------------+
```

## 快速开始

环境：Ubuntu 24.04、ROS 2 Jazzy、OpenVINO 2024.6。适配器依赖 FFmpeg、libjpeg、Paho MQTT 和
Protobuf；自瞄依赖 OpenCV、Eigen、fmt、spdlog、yaml-cpp 等。首次构建前按各子项目报错补齐
系统依赖和相机 SDK。

运行前检查：

| 配置 | 内容 |
| --- | --- |
| `custom_client_adapter/custom_client_adapter/config/rm_video.yaml` | 视频端口、编码、ROI、旋转、亮度和时间补偿 |
| `custom_client_adapter/custom_client_adapter/config/rm_mqtt.yaml` | MQTT broker、话题、限频和开火许可 |
| `auto_aim/configs/AUVClient.yaml` | 模型、同步、相机内参、手眼标定和开火许可 |

一键构建并启动 `rm_video`、`rm_mqtt` 和 `auv_client --debug`：

```bash
./start.sh 3  # 3 为正整数 MQTT client_id
```

不传 `client_id` 时脚本会交互读取；按 `Ctrl+C` 停止全部进程。

## 分组件调试

自瞄：

```bash
source /opt/ros/jazzy/setup.bash
cmake -S auto_aim -B build/auto_aim -DCMAKE_BUILD_TYPE=Release
cmake --build build/auto_aim -j "$(nproc)"
./build/auto_aim/auv_client auto_aim/configs/AUVClient.yaml --debug --show
```

`--show` 打开检测、EKF 和瞄准结果窗口，按 `q` 退出。无桌面环境时去掉该参数。

适配器：

```bash
source /opt/ros/jazzy/setup.bash
colcon --log-base log build --base-paths custom_client_adapter \
  --build-base build/custom_client_adapter --install-base install \
  --symlink-install --packages-select custom_client_adapter
source install/setup.bash
ros2 launch custom_client_adapter custom_client_adapter.launch.py client_id:=3
```

ROSboard：

```bash
source /opt/ros/jazzy/setup.bash
cd rosboard && ./run
```

浏览器访问 `http://<机器人IP>:8888/`。

## 联调检查

```bash
ros2 node list
ros2 topic hz /rm_video/image_processed
ros2 topic hz /rm_mqtt/imu
ros2 topic echo --qos-reliability best_effort --once /rm_mqtt/self_is_red
ros2 topic echo --qos-reliability best_effort --once /auto_aim/result
```

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

详细说明：

- [适配器运行、参数与协议](custom_client_adapter/README.md)
- [AUV Client 接口、同步、标定与安全约定](docs/auv_client_ros_manual.md)
- [项目协作约定](AGENTS.md)
