# custom_client_adapter：RoboMaster 自定义客户端 ROS 2 适配器

该工作空间提供低延迟 RoboMaster UDP 视频解码节点，以及在 ROS 2 与裁判系统自定义客户端
MQTT 之间双向桥接 0x0310 遥测和 0x0311 控制的节点。它既可独立运行，也可作为上级仓库
`sp_vision` 的图像、IMU、阵营和控制适配层；完整自瞄链路见 [`../README.md`](../README.md)。

当前开发环境为 Ubuntu 24.04 + ROS 2 Jazzy，代码同时避免使用 Jazzy 专属接口，以兼容 ROS 2 Kilted。

## 依赖

```bash
export ROS_DISTRO="${ROS_DISTRO:-jazzy}"
sudo apt install \
  ros-${ROS_DISTRO}-desktop \
  libavcodec-dev libavutil-dev libswscale-dev libjpeg-dev \
  python3-paho-mqtt python3-protobuf \
  python3-colcon-common-extensions
```

## 配置

以下路径均相对于本工作空间根目录。两个节点的参数位于同一功能包内：

```text
custom_client_adapter/config/rm_video.yaml
custom_client_adapter/config/rm_mqtt.yaml
```

常用参数：

- `udp.port`：视频 UDP 端口，默认 `3334`。
- `decoder.codec`：`hevc`、`h264` 或 `mjpeg`，默认 `hevc`。
- `decoder.threads`：FFmpeg 切片线程数，`0` 表示自动选择。
- `decoder.decode_queue_size`：待解码完整帧队列深度，默认 `4`，用于吸收关键帧计算峰值。
- `publisher.raw_topic`：完整画面 JPEG 话题，默认 `/rm_video/image_raw`。
- `publisher.processed_topic`：ROI、亮度和旋转处理后的 JPEG 话题，默认 `/rm_video/image_processed`。
- `compression.jpeg_quality`：JPEG 质量，范围 `1-100`，默认 `80`。
- `roi.center_x`、`roi.center_y`：归一化 ROI 中心点。
- `roi.crop_ratio`：保持原始宽高比的裁剪比例，当前配置为 `0.25`，`1.0` 表示不裁剪。
- `processed.rotation_quarter_turns`：逆时针旋转 90 度的次数，范围 `0-3`，当前配置为 `2`。
- `brightness_gain`：亮度增益，当前配置为 `0.5`，`1.0` 表示不改变。
- `timestamp_offset_sec`：始终叠加到当前 ROS 时间的补偿量，可为负数；视频当前为 `-0.06` s，
  MQTT IMU 当前为 `0.0` s。

参数修改后需要重启节点。节点不提供运行时动态调参接口。

## 编译

在上级仓库根目录构建。所有生成内容统一位于根 `build/`、`install/` 和 `log/`：

```bash
export ROS_DISTRO="${ROS_DISTRO:-jazzy}"
source /opt/ros/${ROS_DISTRO}/setup.bash
colcon --log-base log build --base-paths custom_client_adapter \
  --build-base build/custom_client_adapter --install-base install \
  --symlink-install --packages-select custom_client_adapter
source install/setup.bash
```

## 启动

上级仓库根目录的现场脚本会自动编译，并同时启动 `rm_video`、`rm_mqtt` 和 `auv_client`。
在仓库根目录执行，`client_id` 可以作为第一个参数传入：

```bash
# 在上级仓库根目录执行
./start.sh 3
```

不传参数时，脚本会先要求输入 `client_id`，校验为正整数后再作为 ROS 2 参数传入：

```bash
# 在上级仓库根目录执行
./start.sh
```

绕过脚本直接使用 launch 时，`client_id` 是必填参数；缺失或无效时 launch 会立即报错，MQTT 节点也会独立校验收到的 `mqtt.client_id`：

```bash
ros2 launch custom_client_adapter custom_client_adapter.launch.py client_id:=3
```

单独调试视频节点时，可直接加载对应 YAML：

```bash
ros2 run custom_client_adapter rm_video_node --ros-args \
  --params-file custom_client_adapter/config/rm_video.yaml
```

查看话题：

```bash
ros2 topic info /rm_video/image_raw
ros2 topic info /rm_video/image_processed
```

两个话题都发布 `sensor_msgs/msg/CompressedImage` JPEG，并使用 best-effort QoS。`image_raw` 保持完整解码尺寸，不应用 ROI、亮度和旋转；`image_processed` 应用配置中的 ROI、亮度和旋转。两个消息对同一帧使用相同的补偿后时间戳和 `frame_id`。FFmpeg 码流解码与 BGR 转换在不同线程执行，中间只保留最新的已解码原生帧；JPEG 压缩另有独立线程并只保留最新待压缩帧。默认 DDS 发布队列深度为 `1`，各阶段均不会积压旧画面。

## 自定义客户端 MQTT

运行 MQTT 节点前，将运行主机配置到图传自定义客户端网络，并确保能够访问 MQTT broker，协议默认地址为 `192.168.12.1:3333`。单独调试时需显式提供作为 MQTT client ID 的机器人 ID：

```bash
ros2 run custom_client_adapter rm_mqtt_node --ros-args \
  --params-file custom_client_adapter/config/rm_mqtt.yaml \
  -p mqtt.client_id:=3
```

节点订阅 Protobuf 主题 `CustomByteBlock`，校验 0x0310 data 和内层 `Aim_Tx` CRC16 后发布：

- `/rm_mqtt/imu`：`sensor_msgs/msg/Imu`，只包含归一化姿态四元数；角速度和线加速度标记为不可用。
- `/rm_mqtt/self_is_red`：`std_msgs/msg/Bool`；`enem_color=1` 时为 `true`，`enem_color=0` 时为 `false`。

两个话题均使用 best-effort、depth 1。`timestamp_offset_sec` 始终叠加到 IMU 的当前 ROS 时间戳，可通过 YAML 配置正负补偿；布尔消息没有 Header。MQTT 回调完成校验后直接发布，不使用中间消息队列。

节点还以 best-effort、depth 1 订阅 `/auto_aim/result` 的 `std_msgs/msg/String` JSON。每条上游消息到达时直接编码并发布 MQTT `CustomControl`，由图传链路封装为 0x0311：

- `control=false` 对应 `mode=0`，所有角度、速度和加速度字段强制清零。
- `control=true` 对应 `mode=1`；仅当 `shoot=true` 且 YAML 中 `control.allow_fire=true` 时使用 `mode=2`。
- JSON 的 `yaw_rad`、`pitch_rad` 以弧度直接写入 `Aim_Rx`；四个速度和加速度字段当前固定为零。
- `Aim_Rx` 固定为 28 字节，内层 CRC16 覆盖前 26 字节；后接 2 字节零填充，组成固定 30 字节 `CustomControl.data`。
- `control.max_send_rate_hz` 最大为裁判系统允许的 75 Hz。超过频率的上游消息直接丢弃，不缓存、不补发、不计为错误；上游没有消息时不会发送控制包。

全部 ROS 话题使用 best-effort，两个 MQTT 方向均使用 QoS 0 且控制消息不 retain。

## 与 sp_vision 集成

默认参数与上级项目 `auto_aim/configs/AUVClient.yaml` 直接对应，无需话题重映射：

| 话题 | ROS 类型 | 适配器方向 |
| --- | --- | --- |
| `/rm_video/image_processed` | `sensor_msgs/msg/CompressedImage` | 发布 |
| `/rm_mqtt/imu` | `sensor_msgs/msg/Imu` | 发布 |
| `/rm_mqtt/self_is_red` | `std_msgs/msg/Bool` | 发布 |
| `/auto_aim/result` | `std_msgs/msg/String` | 订阅 |

从上级仓库根目录运行 `./start.sh <client_id>` 即可同时启动适配器与 `auv_client`。脚本为两端
source 同一 ROS 2 发行版，并沿用当前 `ROS_DOMAIN_ID`。接口字段、时间同步、坐标系和失效保护
细节见 [`../docs/auv_client_ros_manual.md`](../docs/auv_client_ros_manual.md)。

MQTT 未连接时节点默认每秒自动重试一次，并且每秒只打印 warning：

```text
[WARN] [1785014670.626402232] [rm_mqtt]: MQTT disconnected
```

连接正常时每秒打印：

```text
[INFO] [1785014670.626402232] [rm_mqtt]: rx=50packets/s rx_error=0 tx=60packets/s tx_error=0
```

`rx` 是最近一秒收到的全部 `CustomByteBlock` 速率，包括随后校验失败的消息，`rx_error` 是累计的接收解析或 ROS 发布错误数。`tx` 是最近一秒成功提交给 MQTT 客户端的 `CustomControl` 速率，`tx_error` 是累计的 JSON 解析、控制编码或 MQTT 发布错误数。MQTT 断线跳过和主动限频丢弃不计入错误。

## UDP 包格式

每个 UDP 数据报以 8 字节包头开始：

| 字段 | 类型 | 说明 |
| --- | --- | --- |
| `frame_seq` | `uint16` | 帧序号 |
| `fragment_seq` | `uint16` | 帧内分片序号 |
| `total_size` | `uint32` | 完整编码帧字节数 |

字段优先按网络字节序解析，并兼容参考实现使用主机字节序发送的情况。包头后为视频分片数据。

## 运行日志

节点每秒打印一次：

```text
fps=30 udp=420packets/s dropped=0packets error=0frames
```

统计日志格式为 `[INFO] [时间戳] [节点名]: 消息`，启动文件不会额外添加进程名前缀。FPS 和 UDP 速率均按最近一秒窗口取整数。

- `fps`：最近一个统计窗口内成功解码的帧率。
- `udp`：最近一个统计窗口内收到的整数 UDP 包速率，单位为 `packets/s`。
- `dropped`：累计在应用内部丢弃的 UDP 分片，单位为 `packets`；包括无效包、重复包、残帧淘汰和解码前完整帧替换，不包含网络上未到达的包，也不统计已解码画面转换队列的覆盖。
- `error`：累计无法由 FFmpeg 解码的完整编码帧数，单位为 `frames`。

每次增加 `dropped` 时立即打印英文 warning，包含帧号、UDP 包数和具体原因；原因包括无效包头或负载、重复分片、重组尺寸冲突、重组超时、重组槽位耗尽和待解码队列满。

编码帧缺失时节点会刷新 FFmpeg 参考状态，并停止发布带损坏或纠错标记的输出帧，直到重新得到干净画面；短时可能跳帧，但不会继续传播依赖缺失参考帧的雪花画面。

## 测试

```bash
export ROS_DISTRO="${ROS_DISTRO:-jazzy}"
source /opt/ros/${ROS_DISTRO}/setup.bash
colcon --log-base log test --build-base build/custom_client_adapter \
  --install-base install --packages-select custom_client_adapter
colcon test-result --test-result-base build/custom_client_adapter --verbose
```

当前自动化测试覆盖 MQTT 传输、协议编解码、统计、节点逻辑和 launch 参数辅助逻辑。UDP 视频
链路仍需结合真实或录制码流检查画面、帧率、丢包和解码错误日志。

## 协议文档

- [`../docs/Aim_Task_Protocol.md`](../docs/Aim_Task_Protocol.md)：自瞄任务层协议。
- [`../docs/RoboMaster 2026 机甲大师高校系列赛通信协议 V2.0.0（20260626）.md`](../docs/RoboMaster%202026%20%E6%9C%BA%E7%94%B2%E5%A4%A7%E5%B8%88%E9%AB%98%E6%A0%A1%E7%B3%BB%E5%88%97%E8%B5%9B%E9%80%9A%E4%BF%A1%E5%8D%8F%E8%AE%AE%20V2.0.0%EF%BC%8820260626%EF%BC%89.md)：裁判系统通信协议原文。
