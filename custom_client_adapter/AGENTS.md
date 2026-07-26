# AGENTS.md

## 适用范围

本文件适用于 `custom_client_adapter/` 工作空间，是上级 `../AGENTS.md` 的附加约束。修改本目录
时同时遵守两份文件；冲突时以本文件的实时性、协议和日志约定为准。本文中的“工作区根目录”
指当前 `custom_client_adapter/` 目录，而“仓库根目录”指上一级合并仓库。

## 项目目标

这是一个以数据实时性为首要目标的 RoboMaster ROS 2 工作空间。视频和 0x0310/0x0311 MQTT
双向桥接位于工作区内的 `custom_client_adapter` 单一混合语言包；当前验证环境为 Ubuntu 24.04 +
ROS 2 Jazzy，并要求保持 Kilted 源码兼容。

## 快速架构

- `custom_client_adapter/include/rm_video` 与 `custom_client_adapter/src/*.cpp`：UDP 接收、分片重组、FFmpeg 解码、BGR 转换、JPEG 压缩和图像发布。
- `custom_client_adapter/src/rm_video_node.cpp`：视频参数读取与校验、ROI、亮度、旋转、时间戳和每秒统计日志。
- `custom_client_adapter/rm_mqtt`：MQTT 传输、0x0310/0x0311 协议、ROS 直接桥接和统计。
- `custom_client_adapter/config/rm_video.yaml`：`rm_video` 节点参数及中文说明。
- `custom_client_adapter/config/rm_mqtt.yaml`：`rm_mqtt` 节点参数及中文说明。
- `custom_client_adapter/launch/custom_client_adapter.launch.py`：统一启动两个节点，要求显式传入 `client_id:=N`。
- `../start.sh`：仓库现场启动脚本，从首个参数或交互输入读取正整数 `client_id`，构建后同时启动适配器与自瞄端。
- `README.md`：面向用户，只维护使用方式和必要协议说明，不写开发历程。
- `../docs/Aim_Task_Protocol.md` 与 `../docs/RoboMaster 2026 机甲大师高校系列赛通信协议 V2.0.0（20260626）.md`：协议依据，不随实现偏好改写协议字段。

## 实时性约束

- UDP 接收和解码必须保持在不同线程，防止解码阻塞收包。
- 接收与解码之间使用 YAML 配置的有界完整帧队列，默认深度为 4，仅吸收关键帧等短时计算峰值；队列满时清空积压、累计并立即记录丢弃原因，再从最新帧恢复。
- FFmpeg 码流解码和 BGR 转换/ROS 图像发布必须保持在不同线程；两者之间最多保留一个已解码原生帧，显示链路跟不上时只覆盖已解码画面，不能跳过参考帧解码。
- JPEG 压缩始终在独立线程执行并发布两个压缩话题，最多保留一个待压缩帧，压缩跟不上时覆盖旧帧。
- 发布 QoS 默认 best-effort、depth 1。
- FFmpeg 禁用帧级多线程缓存，只允许可配置的切片并行，并使用低延迟和快速解码标志。
- 不要为了保证每帧必达而引入无界队列、可靠发布或阻塞式重试。
- 0x0310 MQTT 回调完成校验后直接发布 ROS 消息，不增加中间队列或定时轮询延迟。
- 0x0311 在上游 ROS 回调中直接发布；超过配置频率的消息立即丢弃，不缓存、不补发，上游无消息时不发送。
- 所有 ROS 话题固定使用 best-effort，MQTT 固定使用 QoS 0；控制消息不得 retain。
- MQTT 使用异步网络循环，断线自动重连不能阻塞 ROS executor。

## 协议与统计语义

UDP 包头固定为 8 字节：`frame_seq:uint16`、`fragment_seq:uint16`、`total_size:uint32`。优先解析网络字节序，并保留主机字节序兼容逻辑。

每秒日志字段固定为英文：

- `fps`：窗口内成功输出的整数 FFmpeg 帧率。
- `udp`：窗口内成功 `recvfrom` 的整数包速率，显示单位 `packets/s`。
- `dropped`：累计由本进程丢弃的 UDP 分片数，显示单位 `packets`。待解码队列满并清空时按其中完整帧的分片数累计；已解码画面转换队列的覆盖不计入该值。不能声称该值包含网络上从未收到的包。
- 每次增加 `dropped` 时必须立即打印英文 warning，包含帧号、UDP 包数和明确原因；不得只依赖每秒累计值定位。
- `error`：累计发生明确 FFmpeg 发送、解码或像素转换错误的完整编码帧数，显示单位 `frames`。
- 统计日志格式固定为 `fps=N udp=Npackets/s dropped=Npackets error=Nframes`，控制台格式为 `[INFO] [时间戳] [节点名]: 消息`；launch 必须使用 `output_format="{line}"`，不要额外添加进程名前缀。
- FFmpeg 内部诊断和逐帧解码警告保持关闭，相关失败只累计到每秒日志的 `error`。
- 编码帧丢失时刷新 FFmpeg 参考状态；带损坏或纠错标记的输出帧不得发布，避免缺少参考帧后持续花屏。

MQTT 双向统计语义：

- `rx`：最近一秒窗口收到的全部 `CustomByteBlock` MQTT 消息速率，包括无效消息。
- `rx_error`：进程启动后累计的 Protobuf、长度、帧头、颜色、CRC、填充、四元数或 ROS 发布错误数。
- `tx`：最近一秒窗口成功提交给 MQTT 客户端的 `CustomControl` 速率。
- `tx_error`：进程启动后累计的控制 JSON、编码或已连接状态下 MQTT 发布错误数。
- 主动限频丢弃和 MQTT 断线跳过不属于错误；两侧 error 都是累计值，速率都是一秒窗口值。
- MQTT 已连接时日志格式为 `rx=Npackets/s rx_error=N tx=Npackets/s tx_error=N`；未连接时每秒只打印 warning `MQTT disconnected`。
- `Aim_Rx` 的 yaw/pitch 使用弧度；上游 `yaw_rad/pitch_rad` 直接写入，不进行单位转换。

## 图像规则

- 只发布两个 `sensor_msgs/msg/CompressedImage` JPEG 话题，不发布未压缩 `Image` 或 `CameraInfo`。
- `publisher.raw_topic` 发布完整画面 JPEG，不应用 ROI、亮度和旋转；`publisher.processed_topic` 发布处理结果 JPEG。两个话题名都来自 YAML 且必须不同。
- 同一解码帧的两个 JPEG 消息必须使用相同的补偿后时间戳和 `frame_id`。
- ROI 中心和裁剪比例均为归一化参数。宽高应用同一比例，保持源图宽高比，越界时平移裁剪框，不缩放回源尺寸。
- `processed.rotation_quarter_turns` 取值为 `0-3`，表示 processed 图像逆时针旋转 90 度的次数；旋转 1 或 3 次时交换输出宽高。
- 亮度仅使用非负乘法增益，结果饱和到 `[0, 255]`。
- 亮度处理使用启动时生成的 256 项查找表，不要在逐像素循环中恢复浮点乘法。
- 每帧时间戳始终为 `node.now() + timestamp_offset_sec`。不要加入或特判 `use_sim_time` 项目参数。
- 参数只从 YAML 配置并在启动时读取，修改后重启，不增加动态参数回调。

## 编码约定

- 保持实现简洁，优先重构完整功能块，不留下重复、废弃或临时兼容代码。
- C++ 标准固定为 C++17，不引入更高标准才提供的接口。
- C++ 文件使用工作区根目录 `.clang-format` 统一格式，修改后运行 `clang-format -i`。
- 类型使用 `PascalCase`，函数和变量使用 `snake_case`，成员变量以 `_` 结尾。
- 功能块和复杂算法使用适量中文注释；日志及用户可见错误信息使用英文。
- 避免不必要的 OpenCV/cv_bridge 依赖，颜色转换继续使用 FFmpeg `libswscale`，JPEG 编码使用系统 `libjpeg-turbo`。
- 使用 RAII 或明确且成对的清理逻辑管理线程、socket 和 FFmpeg 资源。
- 修改参数、启动方式、话题或依赖时同步更新本工作区 `README.md`；影响 `auv_client` 集成时也要
  更新仓库根目录 `../README.md` 和 `../docs/auv_client_ros_manual.md`。
- 修改架构、统计语义或编码约定时同步更新本文件。

## 验证

```bash
# 在上级仓库根目录执行，所有生成内容保留在上级仓库根目录
source /opt/ros/jazzy/setup.bash
colcon --log-base log build --base-paths custom_client_adapter \
  --build-base build/custom_client_adapter --install-base install \
  --symlink-install --packages-select custom_client_adapter
colcon --log-base log test --build-base build/custom_client_adapter \
  --install-base install --packages-select custom_client_adapter
colcon test-result --test-result-base build/custom_client_adapter --verbose
```

提交前至少完成 Jazzy 构建，并检查编译警告。Kilted 不在本机时，只能声明源码兼容，不能声明已在 Kilted 实测。
