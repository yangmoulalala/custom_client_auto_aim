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

安装测试和标定图像转换工具：

```bash
sudo apt install -y python3-pytest \
  "ros-${ROS_DISTRO}-image-transport" \
  "ros-${ROS_DISTRO}-compressed-image-transport"
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
| `auto_aim/configs/calibration.yaml` | 标定板、标定图像/IMU 话题和手眼标定的中间参数 |

构建并启动 `rm_video`、`rm_mqtt` 和 `auv_client --debug`：

```bash
./start.sh 3  # 3 为正整数 MQTT client_id
./start.sh 3 --show  # 同时打开本地自瞄可视化窗口
```

`/auto_aim/debug` 发布器始终启用；存在订阅者时按处理速度生成并发布调试图像。
传入 `--show` 时同时创建本地窗口；窗口显示不受话题订阅状态影响。
不传 `client_id` 时脚本会交互读取；按 `Ctrl+C` 停止全部进程。

## 组件调试

自瞄：

```bash
source "/opt/ros/${ROS_DISTRO}/setup.bash"
cmake -S auto_aim -B build/auto_aim -DCMAKE_BUILD_TYPE=Release
cmake --build build/auto_aim -j "$(nproc)"
./build/auto_aim/auv_client auto_aim/configs/AUVClient.yaml --debug
./build/auto_aim/auv_client auto_aim/configs/AUVClient.yaml --debug --show
```

`--show` 打开本地调试窗口，按 `q` 或 `Esc` 退出；调试图像话题不依赖 `--show`，并在存在订阅者时按需发布。

适配器：

```bash
source "/opt/ros/${ROS_DISTRO}/setup.bash"
colcon --log-base log build --base-paths custom_client_adapter \
  --build-base build/custom_client_adapter --install-base install \
  --symlink-install --packages-select custom_client_adapter
source install/setup.bash
ros2 launch custom_client_adapter custom_client_adapter.launch.py client_id:=3
```

## 相机标定与自瞄调参

### 工具和文件

当前集成链路的运行参数集中在 `auto_aim/configs/AUVClient.yaml`。配置由程序启动时读取，不是
ROS 参数，修改后需要重启 `auv_client`。各工具和文件的职责如下：

| 工具或文件 | 用途 |
| --- | --- |
| `ros_calibrate_camera` | 使用棋盘格和原始 `sensor_msgs/Image` 标定内参、畸变及标定分辨率 |
| `ros_calibrate_handeye` | 将原始图像与 IMU 姿态配对，标定相机到云台的旋转和平移 |
| `image_transport republish` | 将 `/rm_video/image_processed` 的 `CompressedImage` 临时解码成标定工具需要的原始图像 |
| `auv_client --debug --show` | 查看检测框、EKF 预测、瞄准点、跟踪状态、处理耗时和输出角度 |
| ROSboard、`ros2 topic hz/echo/info` | 远程查看调试图，检查帧率、阵营、时间同步、QoS 和控制 JSON |
| `auto_aim/configs/calibration.yaml` | 修改棋盘格内角点数、格边长、临时原始图话题、IMU 话题和手眼输入内参 |
| `auto_aim/configs/AUVClient.yaml` | 修改识别、跟踪、预测、弹道、火控参数，并写入最终内参与手眼结果 |
| `custom_client_adapter/custom_client_adapter/config/rm_video.yaml` | 固定标定和运行共同使用的 ROI、旋转、亮度与图像时间戳补偿 |
| `custom_client_adapter/custom_client_adapter/config/rm_mqtt.yaml` | 修改 IMU 时间戳补偿及下游总开火许可 `control.allow_fire` |

模型或 `yolov5_color_order` 改动后，可运行以下测试，检查模型能加载且当前蓝/红输出映射没有
颠倒：

```bash
ctest --test-dir build/auto_aim -R yolov5_model_test --output-on-failure
```

完整坐标系、时间戳和安全语义见 [AUV Client 接口手册](docs/auv_client_ros_manual.md)。

### 标定步骤

标定前先确定 `rm_video.yaml` 中的 ROI 和旋转。内参必须对应自瞄实际收到的
`/rm_video/image_processed` 画面；标定后再改变裁剪、旋转、宽高比或相机安装位置，需要重新标定。
曝光、对焦和镜头光圈也应固定。当前 `calibration.yaml` 配置的是 `10 x 6` 个棋盘格内部角点，
即 `11 x 7` 个黑白格，单格边长 `75 mm`；使用其他标定板时按实物修改。

标定程序只接受原始 `sensor_msgs/Image`，而视频节点发布 JPEG `CompressedImage`。启动适配器后，
在单独终端运行以下转换，并保持该进程直到两项标定结束：

```bash
source "/opt/ros/${ROS_DISTRO}/setup.bash"
ros2 run image_transport republish --ros-args \
  -p in_transport:=compressed \
  -p out_transport:=raw \
  -p qos_overrides./rm_video/image_processed.subscription.reliability:=best_effort \
  -r in/compressed:=/rm_video/image_processed \
  -r out:=/calibration/image_raw
```

在另一个终端确认解码后的原始图持续发布：

```bash
ros2 topic hz /calibration/image_raw
```

先标内参：

```bash
./build/auto_aim/ros_calibrate_camera auto_aim/configs/calibration.yaml \
  --output-folder=auto_aim/assets/ros_camera_calibration
```

画面显示 `board detected` 时按 `s` 收一帧，覆盖画面中心、四角、远近和不同倾角，至少 10 帧；
按 `q` 求解。将 `calibration_result.yaml` 中的 `calibration_image_width`、
`calibration_image_height`、`camera_matrix`、`distort_coeffs` 同时写入 `calibration.yaml` 和
`AUVClient.yaml`。不能只追求低重投影误差，样本对整个视场的覆盖同样重要。

再标手眼：

```bash
./build/auto_aim/ros_calibrate_handeye auto_aim/configs/calibration.yaml \
  --output-folder=auto_aim/assets/ros_handeye_calibration
```

标定板在整个过程中必须固定。先根据机械安装填写 `R_gimbal2imubody`，确认预览中的
yaw/pitch/roll 方向和正负号正确；再让云台取得至少 10 个旋转差异明显的姿态，每个有效姿态按
`s`，最后按 `q`。将 `handeye_result.yaml` 中的 `R_gimbal2imubody`、`R_camera2gimbal` 和
`t_camera2gimbal` 写入 `AUVClient.yaml`。平移单位已经由工具转换为米。

### 2026 前哨站预测

前哨站三块装甲按 `120 degree` 间隔、`0.275 m` 半径旋转，相邻装甲中心高度差为
`0.102 m`。Tracker 以首次观测装甲为高度基准，根据后续各装甲 ID 的高度观测从六种可能排列中
选择残差最小的一种；三块装甲尚未全部出现时，Aimer 只瞄准当前观测 ID，不预测未知高度装甲。

前哨站保留原有 `70/30 degree` 进入/离开选板窗口和收敛后 `|angular_velocity| > 2 rad/s`
时吸附到 `+/-2.51 rad/s` 的逻辑。静止或方向窗口没有候选时，只在正面夹角不超过 `30 degree`
的已观测装甲中回退选择；无候选时撤销控制和开火。

### 调参顺序和建议

1. **先关闭开火。** 将 `AUVClient.yaml` 的 `auto_fire` 和 `rm_mqtt.yaml` 的
   `control.allow_fire` 都设为 `false`，先只验证瞄准控制；最终恢复时也要逐层确认。
2. **先保证输入可靠。** 固定分辨率、ROI、旋转、曝光和模型，检查图像/IMU 帧率、阵营以及
   `auv_client --debug` 中的 `stale`、`unmatched_imu`。不要通过增大同步容差、等待时间或队列来
   掩盖时钟和性能问题。
3. **先调识别，再调跟踪。** 用调试图检查蓝红映射、类别和四角点；从 `min_confidence` 入手，
   必要时再启用 ROI 或传统角点修正。识别稳定后再改 `min_detect_count` 和丢失帧计数。Tracker
   参数按帧计数，帧率变化会直接改变实际确认/保持时间。
4. **静态目标先校几何和弹道。** 写入实测 `bullet_speed`，将偏置从小量开始调整，在近、中、远
   多个距离验证。固定方向误差才用 `yaw_offset`/`pitch_offset`；误差随距离增长时应优先检查
   内参、外参、单位和弹速，不能用单一偏置硬补。
5. **动态目标再调预测。** 分别测试平移、正反方向旋转和不同转速，一次只改一个 delay。
   `high_speed_delay_time`/`low_speed_delay_time` 增大表示把目标预测得更靠后；过大会越过目标。
   Aimer 使用角速度绝对值与 `decision_speed` 比较，正、反转采用相同的高低速切换阈值，但仍应
   分别验证方向、选板和延迟补偿。
6. **最后开放开火。** 先用较小的 `first_tolerance`/`second_tolerance` 验证近远距离切换，再逐步
   放宽。`shoot=true` 还要求控制有效、瞄准点有效、指令没有突变且云台进入 yaw 容差；最终只有
   `auto_fire=true` 与 `control.allow_fire=true` 同时满足，开火命令才会下发。

每轮只改一类参数，记录配置版本、距离、弹速、转向、命中偏差和调试日志。`comming_angle` 是代码
当前使用的历史拼写，不能改成 `coming_angle`；`min_spin_speed` 当前未被 `auv_client` 读取，具体
限制已写在 YAML 注释中。

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

相关文档：

- [适配器运行、参数与协议](custom_client_adapter/README.md)
- [AUV Client 接口、同步、标定与安全约定](docs/auv_client_ros_manual.md)
- [项目协作约定](AGENTS.md)
