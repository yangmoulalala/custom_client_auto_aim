# custom_client_auto_aim

RoboMaster 自定义客户端与自瞄的 ROS 2 集成项目。项目把裁判系统提供的视频、IMU 和阵营信息转换为
标准 ROS 2 话题，交给自瞄完成识别、跟踪、解算和火控，再将控制结果送回裁判系统。自瞄算法来自
同济大学 SuperPower 战队的
[`sp_vision_25`](https://github.com/TongjiSuperPower/sp_vision_25)。

## 1. 了解系统链路

仓库包含两个可独立构建的项目，以及一个集成入口：

| 目录或程序 | 作用 |
| --- | --- |
| `custom_client_adapter/` | 接收并解码 UDP 视频，桥接裁判系统 MQTT 0x0310/0x0311 数据 |
| `auto_aim/` | 完成装甲识别、位姿解算、跟踪、预测、瞄准和开火决策 |
| `custom_client` | 订阅适配器输入并发布自瞄控制结果 |

```text
UDP video -> rm_video -> /rm_video/image_processed --------+
MQTT 0310 -> rm_mqtt  -> /rm_mqtt/imu ---------------------+-> custom_client
                         /rm_mqtt/self_is_red --------------+       |
                                                                    +-> /auto_aim/debug
                                                                    v
MQTT 0311 <- rm_mqtt  <- /auto_aim/result <-------------------------+
```

## 2. 准备运行环境

当前环境为 x86_64 Ubuntu 24.04 和 ROS 2 Jazzy。先按照
[ROS 2 Jazzy 官方文档](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)完成 Desktop 安装。

### NVIDIA 软件源和驱动

GPU 推理需要 NVIDIA 驱动。先添加后续 CUDA 安装使用的 NVIDIA 官方软件源，再统一更新一次软件包
索引并安装 Ubuntu 推荐的驱动：

```bash
curl -fL \
  https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2404/x86_64/cuda-keyring_1.1-1_all.deb \
  -o /tmp/cuda-keyring.deb
sudo dpkg -i /tmp/cuda-keyring.deb
rm /tmp/cuda-keyring.deb
sudo apt update
sudo apt install -y ubuntu-drivers-common
sudo ubuntu-drivers install
sudo reboot
```

重启并重新登录后，先运行 `nvidia-smi`，确认驱动可以正常访问 GPU，再继续安装项目依赖。如果提示
`Driver/library version mismatch`，说明仍在运行更新前加载的内核驱动，不能启动 GPU 推理。启用
Secure Boot 时，请在重启过程中按照系统提示完成 MOK 密钥确认。驱动版本选择、指定版本安装和卸载
方法见
[Ubuntu NVIDIA 驱动文档](https://documentation.ubuntu.com/server/how-to/graphics/install-nvidia-drivers/)。

### 项目依赖

重启后安装默认构建、运行、标定、离线分析和维护所需的 APT 软件包。Python 依赖集中在同一组：

```bash
sudo apt install -y \
  build-essential cmake curl git pkg-config \
  libopencv-dev libfmt-dev libeigen3-dev libspdlog-dev \
  libyaml-cpp-dev nlohmann-json3-dev \
  libavcodec-dev libavutil-dev libswscale-dev libjpeg-dev \
  python3-colcon-common-extensions python3-paho-mqtt python3-protobuf \
  python3-pytest python3-numpy python3-opencv python3-matplotlib \
  python3-pyqt5 python3-tornado \
  ros-jazzy-ament-cmake ros-jazzy-ament-cmake-python \
  ros-jazzy-ament-cmake-pytest \
  ros-jazzy-rosbag2-py ros-jazzy-rosbag2-storage-mcap
```

活动链路不使用 OpenVINO、Ceres、libusb 或工业相机 SDK，无需安装这些依赖。

### CUDA、cuDNN 和 ONNX Runtime

使用前面添加的软件源安装 CUDA Toolkit 12.8 和 cuDNN 9：

```bash
sudo apt install -y cuda-toolkit-12-8 cudnn9-cuda-12
```

这两个软件包不安装 NVIDIA 驱动，因此驱动步骤不能省略。随后安装 ONNX Runtime 1.28.0 GPU 版本：

```bash
curl -fL \
  https://github.com/microsoft/onnxruntime/releases/download/v1.28.0/onnxruntime-linux-x64-gpu_cuda12-1.28.0.tgz \
  -o /tmp/onnxruntime-1.28.0.tgz
sudo mkdir -p /opt/onnxruntime
sudo tar -xzf /tmp/onnxruntime-1.28.0.tgz --strip-components=1 \
  -C /opt/onnxruntime
echo "/opt/onnxruntime/lib" | sudo tee /etc/ld.so.conf.d/onnxruntime.conf
sudo ldconfig
rm /tmp/onnxruntime-1.28.0.tgz
```

## 3. 完成首次配置

运行参数分布在四个 YAML 文件中。适配器参数的用途、单位、范围和实时性影响直接写在文件的中文
注释中，README 不重复维护逐字段参数表。

| 配置文件 | 首次使用时确认的内容 |
| --- | --- |
| `custom_client_adapter/custom_client_adapter/config/rm_video.yaml` | UDP 端口、编码格式、ROI、旋转、亮度和图像时间补偿 |
| `custom_client_adapter/custom_client_adapter/config/rm_mqtt.yaml` | MQTT 服务器、IMU 时间补偿、控制限频和下游开火许可 |
| `auto_aim/configs/custom_client.yaml` | 模型设备、图像/IMU 同步、弹速、内外参、跟踪预测和自瞄开火许可 |
| `auto_aim/configs/calibration.yaml` | 棋盘格规格、标定话题、内参和手眼标定输入 |

首次接触机器人时按以下顺序配置：

1. 在 `rm_video.yaml` 中确认视频端口、编码、ROI 和旋转，使
   `/rm_video/image_processed` 成为自瞄最终使用的画面。完成标定后不要再改变这些几何设置。
2. 在 `rm_mqtt.yaml` 中确认裁判系统地址和 IMU 时间补偿，并将 `control.allow_fire` 设为 `false`。
3. 在 `custom_client.yaml` 中将 `auto_fire` 设为 `false`，确认输入话题和推理设备。未完成标定前只
   验证数据链路和调试画面，不进行实弹控制。
4. 按实物修改 `calibration.yaml` 中的棋盘格规格，后续将标定结果同步写入该文件和
   `custom_client.yaml`。

修改配置后需要重启对应节点。话题名属于两端接口，除非同步修改所有发布方和订阅方，否则保持
默认值。

## 4. 启动并验收完整链路

从仓库根目录运行：

```bash
./start.sh
./start.sh --show
```

脚本先构建自瞄和适配器，再启动 `rm_video`、`rm_mqtt` 和 `custom_client --debug`。MQTT client ID
由适配器自动探测；脚本只识别可选的 `--show`，其他参数不会传给节点。按 `Ctrl+C` 停止整条链路。

`--show` 只额外打开本地调试窗口，按 `q` 或 `Esc` 可关闭。不加 `--show` 时仍可通过
`/auto_aim/debug` 在 ROS 中查看调试图像。

自瞄创建推理会话后使用全黑图像执行三次预热。预热的结果会被丢弃，其作用是提前完成 ONNX
Runtime 以及 GPU 模式下的 CUDA 上下文和首次执行内核初始化，避免第一张真实图像承担额外延迟。

### 检查 ROS 话题

完整链路使用以下话题；实时话题均为 best-effort、keep last 1：

| 话题 | 类型 | 应看到的内容 |
| --- | --- | --- |
| `/rm_video/image_raw` | `sensor_msgs/msg/CompressedImage` | 未裁剪、未旋转的完整 JPEG 画面 |
| `/rm_video/image_processed` | `sensor_msgs/msg/CompressedImage` | 经过 ROI、旋转和亮度处理的自瞄输入 |
| `/rm_mqtt/imu` | `sensor_msgs/msg/Imu` | 裁判系统提供的姿态四元数 |
| `/rm_mqtt/self_is_red` | `std_msgs/msg/Bool` | 己方阵营，`true` 表示红方 |
| `/auto_aim/result` | `std_msgs/msg/String` | 自瞄控制结果 JSON |
| `/auto_aim/debug` | `sensor_msgs/msg/CompressedImage` | 检测、装甲板距离、跟踪、瞄准点和处理状态 |

```bash
ros2 topic list
ros2 topic hz /rm_video/image_processed
ros2 topic hz /rm_mqtt/imu
ros2 topic echo --qos-reliability best_effort --once /rm_mqtt/self_is_red
ros2 topic echo --qos-reliability best_effort --once /auto_aim/result
ros2 topic hz /auto_aim/debug
```

首次验收至少应确认处理后图像和 IMU 持续更新、阵营正确、调试图方向与实际一致。收到阵营前、输入
超时或数据无效时，`/auto_aim/result` 应保持安全结果，不能沿用旧的开火命令。

### 阅读运行日志

| 组件 | 正常运行时关注 | 异常时先检查 |
| --- | --- | --- |
| `rm_video` | `fps`、`udp` 持续更新 | `dropped`、`error` 是否增长，以及紧邻的 warning |
| `rm_mqtt` | 已连接的 client ID，`rx`、`tx` 持续更新 | `rx_error`、`tx_error` 和断线原因 |
| `custom_client` | `input ok`、识别、跟踪和控制统计 | `timeout`、`stale`、`unmatched_imu`、阵营和安全输出 |

不要通过增大 ROS 队列深度掩盖消费速度或时间同步问题。运行日志以及 colcon 的构建日志统一位于
仓库根目录的 `log/`。

### 使用 ROSboard

使用 [ROSboard](https://github.com/dheera/rosboard) 在网页中查看 ROS 话题。仓库根目录尚无
`rosboard/` 时先执行：

```bash
git clone https://github.com/dheera/rosboard.git
```

启动 ROSboard：

```bash
./rosboard/run
```

浏览器访问 `http://<机器人IP>:8888/`，重点查看 processed/debug 图像、IMU、阵营和控制结果。

## 5. 完成相机和手眼标定

内参必须对应 `/rm_video/image_processed` 的实际分辨率、ROI 和旋转。改变裁剪、旋转、宽高比、
镜头或相机安装位置后需要重新标定；曝光、对焦和镜头光圈也应固定。

当前 `calibration.yaml` 配置为 `10 x 7` 个棋盘格内部角点，即 `11 x 8` 个黑白格，单格边长
`50 mm`。使用其他标定板时先按实物修改。

### 准备标定图像

标定程序直接订阅 `/rm_video/image_processed` 的 JPEG `sensor_msgs/msg/CompressedImage`。先保持
适配器运行，不需要额外的图像转换进程：

```bash
ros2 launch custom_client_adapter custom_client_adapter.launch.py
```

```bash
ros2 topic hz /rm_video/image_processed
```

### 标定相机内参

```bash
./build/auto_aim/ros_calibrate_camera auto_aim/configs/calibration.yaml \
  --output-folder=auto_aim/assets/ros_camera_calibration
```

画面显示 `board detected` 时按 `s` 采集一帧。样本应覆盖中心、四角、远近和不同倾角，至少采集
10 帧；按 `q` 求解。将输出中的 `calibration_image_width`、`calibration_image_height`、
`camera_matrix` 和 `distort_coeffs` 同时写入 `calibration.yaml` 与 `custom_client.yaml`。除重投影
误差外，还要检查样本是否充分覆盖整个视场。

### 标定相机到云台

```bash
./build/auto_aim/ros_calibrate_handeye auto_aim/configs/calibration.yaml \
  --output-folder=auto_aim/assets/ros_handeye_calibration
```

标定板在整个过程中保持固定。先根据机械安装填写 `R_gimbal2imubody`，确认预览中的
yaw/pitch/roll 方向和正负号正确；再让云台取得至少 10 个旋转差异明显的姿态，每个有效姿态按
`s`，最后按 `q`。将输出中的 `R_gimbal2imubody`、`R_camera2gimbal` 和 `t_camera2gimbal` 写入
`custom_client.yaml`；平移单位为米。

手眼工具与 `custom_client` 使用相同的时间戳匹配方式：遍历最多 200 条 IMU 缓存，选择与图像
`header.stamp` 绝对时间差最小且不超过 `sync_tolerance_ms` 的样本。暂时没有合格样本时最多等待
`sync_wait_ms`，超时丢弃该图像；工具不会插值、外推或改写消息时间戳。固定时差应通过适配器的
`timestamp_offset_sec` 校正，不能靠放大同步容差掩盖。

坐标系、时间戳和安全语义的完整定义见
[Custom Client 接口手册](docs/auv_client_ros_manual.md)。

## 6. 验证自瞄并逐步调参

当前赛制仅英雄使用大装甲；哨兵、工程、3/4/5 号步兵、前哨站和基地均按小装甲解算。基地继续
使用三装甲布局，不再保留平衡步兵或两装甲底盘分支。

2026 前哨站的三块装甲按 `120 degree` 间隔、`0.275 m` 半径旋转，相邻装甲中心高度差为
`0.102 m`。Tracker 根据不同装甲 ID 的高度观测从六种排列中选择残差最小的一种；三块装甲尚未
全部出现时只瞄准已观测装甲，不预测未知高度。选板沿用 `70/30 degree` 进入/离开窗口，收敛后
`|angular_velocity| > 2 rad/s` 时吸附到 `+/-2.51 rad/s`；无满足角度约束的候选时撤销控制和开火。

按以下顺序联调，每轮只改一类参数并记录配置版本、距离、弹速、转向、命中偏差和日志：

1. **保持禁止开火。** 确认 `auto_fire=false` 且 `control.allow_fire=false`，先验证云台控制方向。
2. **确认输入可靠。** 固定分辨率、ROI、旋转、曝光和模型，消除 `stale`、`unmatched_imu` 等输入
   问题，不用更深队列或更大同步容差掩盖根因。
3. **先调识别，再调跟踪。** 用 debug 图检查蓝红映射、类别和四角点，从 `min_confidence` 开始；
   识别稳定后再修改确认和丢失帧数。Tracker 参数按帧计数，帧率会改变实际确认与保持时间。
4. **用静态目标校几何和弹道。** 写入实测 `bullet_speed`，在近、中、远距离验证。固定方向误差才
   使用 `yaw_offset`/`pitch_offset`；随距离增长的误差优先检查内参、外参、单位和弹速。
5. **用动态目标校预测。** 分别测试平移、正反旋转和不同转速，一次只改一个 delay。delay 增大
   表示预测得更靠前，过大会越过目标。
6. **最后开放开火。** 从较小的开火容差开始；确认控制、瞄准点、输入和云台误差均有效后，再依次
   开启 `auto_fire` 与 `control.allow_fire`。

字段的单位、范围和当前实现限制以 YAML 中文注释为准。

## 7. 拆分组件排查问题

完整链路异常时，先根据话题和日志确定问题属于适配器输入还是自瞄处理，再单独构建和运行对应
组件。

### 单独构建和运行自瞄

```bash
cmake -S auto_aim -B build/auto_aim -DCMAKE_BUILD_TYPE=Release
cmake --build build/auto_aim -j "$(nproc)"
./build/auto_aim/custom_client auto_aim/configs/custom_client.yaml --debug
./build/auto_aim/custom_client auto_aim/configs/custom_client.yaml --debug --show
```

### 单独构建和运行适配器

```bash
colcon --log-base log build --base-paths custom_client_adapter \
  --build-base build/custom_client_adapter --install-base install \
  --symlink-install --packages-select custom_client_adapter
source install/setup.bash
ros2 launch custom_client_adapter custom_client_adapter.launch.py
```

launch 同时启动 `rm_video` 和 `rm_mqtt`。只运行一个节点时使用：

```bash
ros2 run custom_client_adapter rm_video_node --ros-args \
  --params-file custom_client_adapter/custom_client_adapter/config/rm_video.yaml
ros2 run custom_client_adapter rm_mqtt_node --ros-args \
  --params-file custom_client_adapter/custom_client_adapter/config/rm_mqtt.yaml
```

### 离线分析图像和 IMU 延迟

在线节点只负责转发实时数据。需要分析同步延迟时，先录制固定话题：

```bash
python3 -s custom_client_adapter/tools/record_imu_camera_bag.py
```

bag 写入 `log/imu_camera_YYYYMMDD_HHMMSS/`。按 `Ctrl+C` 后，工具完成 MCAP 写入并核对实际消息数。
随后分析指定 bag：

```bash
python3 -s custom_client_adapter/tools/analyze_imu_camera_bag.py \
  log/imu_camera_YYYYMMDD_HHMMSS
```

省略路径时自动选择 `log/` 下最新的 `imu_camera_*` bag。分析界面显示 LK 光流、IMU 角速度和互相关
延迟；滚轮缩放时间轴，连续两次左键点击可读取两点时间差。

## 相关文档

- [Custom Client 接口、同步、标定与安全约定](docs/auv_client_ros_manual.md)
- [自瞄任务层协议](docs/Aim_Task_Protocol.md)
- [RoboMaster 2026 通信协议](docs/RoboMaster%202026%20%E6%9C%BA%E7%94%B2%E5%A4%A7%E5%B8%88%E9%AB%98%E6%A0%A1%E7%B3%BB%E5%88%97%E8%B5%9B%E9%80%9A%E4%BF%A1%E5%8D%8F%E8%AE%AE%20V2.0.0%EF%BC%8820260626%EF%BC%89.md)
- [项目协作约定](AGENTS.md)
