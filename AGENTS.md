# AGENTS.md

## 范围与优先级

本文件适用于整个仓库，是仓库内唯一的协作约定文件。

仓库由两个可独立构建的项目和一个集成入口组成：

| 边界 | 职责 |
| --- | --- |
| `auto_aim/` 下的 `sp_vision` | C++17 自瞄框架，包含 I/O、识别、解算、跟踪、规划、火控、标定和离线测试；非 ROS 功能必须继续支持独立构建 |
| `custom_client_adapter/` | ROS 2 工作空间，负责 RoboMaster UDP 视频解码和裁判系统 MQTT 0x0310/0x0311 双向桥接 |
| `custom_client` | 使用压缩图像、IMU 和阵营信息运行自瞄，并输出 JSON 控制结果 |

当前验证环境为 Ubuntu 24.04 + ROS 2 Jazzy，同时保持 Kilted 源码兼容。所有修改按以下顺序权衡：

1. 控制失效时安全退化。
2. 实时数据保持最新，不形成无界积压。
3. ROS、MQTT、坐标系、时间戳和单位契约保持兼容。
4. 在上述前提下维护识别、跟踪和控制效果。

## 代码与文件边界

| 路径 | 所有权和用途 |
| --- | --- |
| `auto_aim/src/` | 不同兵种、集成和调试程序入口 |
| `auto_aim/tasks/auto_aim/` | 装甲识别、位姿解算、跟踪、瞄准和开火决策 |
| `auto_aim/tasks/auto_buff/`、`auto_aim/tasks/omniperception/` | 打符和全向感知休眠功能 |
| `auto_aim/io/` | 工业相机、串口、IMU、云台和 ROS 2 I/O；`auv_client.*` 是当前集成接口 |
| `auto_aim/calibration/` | 相机内参、手眼标定和数据采集程序 |
| `auto_aim/configs/` | 自瞄与标定 YAML；字段变更必须同步读取端和注释 |
| `auto_aim/models/` | 模型统一归档目录；活动链路只引用 `0526.onnx`，其余 ONNX/XML/BIN 备用 |
| `auto_aim/tests/` | 自瞄直接测试和 CTest 测试 |
| `custom_client_adapter/custom_client_adapter/include/rm_video/`、`src/` | UDP 接收、重组、FFmpeg 解码、BGR 转换、JPEG 压缩和发布 |
| `custom_client_adapter/custom_client_adapter/rm_mqtt/` | MQTT 传输、0x0310/0x0311 编解码、ROS 桥接和统计 |
| `custom_client_adapter/custom_client_adapter/config/`、`launch/` | 适配器 YAML 参数和统一启动文件 |
| `custom_client_adapter/custom_client_adapter/test/` | 适配器 C++ 直接测试和 pytest |
| `custom_client_adapter/tools/` | 固定图像/IMU 话题的 MCAP 录制与离线光流延迟分析 |
| `docs/auv_client_ros_manual.md` | Custom Client 对外接口、同步、坐标系、标定和安全契约 |
| `start.sh` | 从根目录构建并启动完整链路；只识别可选 `--show`，其他参数不转发 |

生成内容统一位于仓库根目录的 `build/`、`install/` 和 `log/`。不手工编辑或提交这些目录，也不在
源码子目录生成或保留构建产物。

活动集成程序、ROS 节点和主配置统一命名为 `custom_client`、`custom_client` 和
`auto_aim/configs/custom_client.yaml`。该链路的 ROS 日志使用 `auto_aim` logger 前缀。MQTT
client ID 由适配器自动探测，不由脚本、launch、ROS 参数或 YAML 注入。

## ROS 集成契约

默认完整链路固定为：

| 话题 | ROS 类型 | 发布方 | 订阅方 |
| --- | --- | --- | --- |
| `/rm_video/image_processed` | `sensor_msgs/msg/CompressedImage` | `rm_video` | `custom_client` |
| `/rm_mqtt/imu` | `sensor_msgs/msg/Imu` | `rm_mqtt` | `custom_client` |
| `/rm_mqtt/self_is_red` | `std_msgs/msg/Bool` | `rm_mqtt` | `custom_client` |
| `/auto_aim/result` | `std_msgs/msg/String` | `custom_client` | `rm_mqtt` |
| `/auto_aim/debug` | `sensor_msgs/msg/CompressedImage` | `custom_client` | 调试工具 |

- 全部实时 ROS 话题固定使用 best-effort、keep last 1。不得通过 reliable、深队列或无界缓存掩盖
  下游性能问题。
- 图像和 IMU 时间戳必须使用同一 ROS 时钟。固定上游延迟通过配置补偿，不在算法中添加隐藏常量。
- 图像和 IMU 的适配器时间戳均为各节点 `now() + timestamp_offset_sec`。不要添加项目级
  `use_sim_time` 特判。
- `self_is_red=true` 表示己方红、目标蓝；`false` 表示己方蓝、目标红。
- `/auto_aim/result` 的 `yaw_rad`、`pitch_rad` 使用弧度。
- 未收到阵营、输入失效、命令超时、检测/跟踪/解算失败或结果非有限数时，`custom_client` 必须
  发布安全命令，不得继续使用旧目标或旧开火结果。

坐标系、四元数顺序、角度单位、图像分辨率缩放和时间戳语义属于对外契约。修改这些边界前先阅读
`docs/auv_client_ros_manual.md`，并为正常输入、转换边界和失效保护补充测试。

## 活动算法范围与安全

- 默认活动构建只包含 Custom Client、自瞄核心、ROS 标定工具和直接测试。YOLOv8/11、传统检测器、
  全向感知和能量机关源码保留但不参与默认构建，不得为休眠源码重新引入 OpenVINO。
- 活动检测只接受 `yolo_name: yolov5` 和 ONNX Runtime `CPU`/`GPU` 设备。默认 0526 模型契约为
  FP16 `[1,3,640,640]` 输入、FP32 `[1,25200,22]` 输出。
- 请求 GPU 时，CUDA Provider 不可用、会回退 CPU 节点或构建未包含 GPU 支持均必须启动失败，
  不能静默切换到 CPU。
- 模型路径、设备和颜色顺序保持可配置。替换模型时同时验证输入尺寸、输出布局、颜色映射以及
  CPU/GPU 结果一致性。
- 当前赛制仅英雄使用大装甲；哨兵、工程、3/4/5 号步兵、前哨站和基地均使用小装甲。0526 的两个
  历史基地类别统一按小装甲处理；不保留平衡步兵语义或两装甲底盘跟踪分支。
- Target EKF 状态固定为 `x vx y vy z vz angle angular_velocity radius radius_delta height_delta`；
  Aimer 必须使用索引 `7` 判断普通目标是否进入小陀螺选板，索引 `8` 只表示半径。普通目标角速度
  绝对值不超过 YAML 的非负有限 `min_spin_speed` 时使用正面 `60 degree` 锁定选板，超过时按旋转
  方向和 `comming_angle`/`leaving_angle` 选择进入侧装甲；前哨站继续使用独立的
  `70/30 degree` 窗口。
- 改动开火逻辑、阵营处理、超时、CRC 或 MQTT 控制编码属于高风险修改，必须覆盖正常路径、无效
  输入和超时/断线等失效保护。
- 不提交真实机器人专属的网络凭据。仅适用于单台设备的标定值和硬件 ID 必须明确适用范围。

## 适配器实时性设计

### 视频流水线

- UDP 接收与 FFmpeg 解码保持在不同线程。两者之间使用 YAML 配置的有界完整帧队列，默认深度为
  4；队列满时清空积压、累计并立即记录丢弃原因，再从最新完整帧恢复。
- FFmpeg 解码与 BGR 转换/ROS 发布保持在不同线程，两者之间最多保留一个已解码原生帧。下游跟不
  上时覆盖画面，但不能跳过参考帧解码。
- JPEG 压缩在独立线程执行。raw 和 processed 分别检查订阅者，最多保留一个待压缩帧；压缩跟不
  上时覆盖旧帧。
- FFmpeg 禁用帧级多线程缓存，只允许可配置的切片并行，并保持低延迟和快速解码标志。
- 适配器不引入 OpenCV/cv_bridge。颜色转换使用 FFmpeg `libswscale`，JPEG 编码使用系统
  `libjpeg-turbo`。

### 图像输出

- 只发布两个 `sensor_msgs/msg/CompressedImage` JPEG 话题，不发布未压缩 `Image` 或
  `CameraInfo`。
- `publisher.raw_topic` 发布未应用 ROI、亮度和旋转的完整画面；
  `publisher.processed_topic` 发布处理结果，两个话题名必须不同。
- 同一解码帧的 raw 和 processed 消息使用相同的补偿后时间戳和 `frame_id`。
- ROI 中心和裁剪比例为归一化参数，宽高使用相同比例保持源图宽高比。越界时平移裁剪框，不缩放
  回源尺寸。
- `processed.rotation_quarter_turns` 取值为 `0-3`，表示逆时针 90 度旋转次数；旋转 1 或 3 次时
  交换输出宽高。
- 亮度使用非负乘法增益并饱和到 `[0,255]`。启动时生成 256 项查找表，不在逐像素循环执行浮点
  乘法。

### MQTT 流水线

- 0x0310 MQTT 回调完成校验后直接发布 ROS 消息，不增加中间队列或定时轮询。
- 0x0311 在上游 ROS 回调中直接发布。超过配置频率的消息立即丢弃，不缓存、不补发；上游无消息
  时不发送。
- MQTT 固定使用 QoS 0，控制消息不得 retain。ROS 侧同样保持 best-effort、keep last 1。
- 连接工作线程按 `1-6`、`101-106` 自动探测 client ID。任意非零 CONNACK 或明确握手拒绝后等待
  `100 ms` 切换下一候选；TCP 失败、连接超时或普通断线等待 `100 ms` 后重试当前候选。
- TCP 建连和等待 CONNACK 的超时均为 `1 s`，keepalive 为 `10 s`。连接和重连不得阻塞 ROS
  executor。

### 在线诊断

实时路径禁止无界队列、忙等、阻塞式重试和逐帧高频日志。光流等计算密集型诊断先以 best-effort、
keep last 1 和有界缓存录制标准 ROS bag，再按 `header.stamp` 离线分析；不得把诊断计算加入在线
适配器回调。录制时持续显示图像/IMU 活动状态，停止后用 rosbag 元数据核验实际落盘数量。

## 协议、统计与日志契约

### UDP 和图像统计

- UDP 包头固定为 8 字节：`frame_seq:uint16`、`fragment_seq:uint16`、`total_size:uint32`。优先按
  网络字节序解析，并保留参考实现的主机字节序兼容逻辑。
- 视频统计格式固定为 `fps=N udp=Npackets/s dropped=Npackets error=Nframes`。`fps`、`udp` 为
  窗口速率，`dropped`、`error` 为进程启动后的累计值。
- `dropped` 统计本进程丢弃的无效包、重复分片、残帧、重组淘汰和解码前完整帧的 UDP 分片数；
  不包含网络上从未收到的包，也不统计已解码或待压缩画面的覆盖。每次增加时立即输出英文 warning，
  注明可获得的帧号、分片号或包数以及具体原因。
- `error` 统计明确的 FFmpeg 发送、解码、损坏帧和像素转换失败。关闭 FFmpeg 内部诊断和逐帧解码
  warning；流不连续时刷新参考状态，带损坏或纠错标记的帧不得发布。

### MQTT 数据和统计

- `CustomByteBlock.data` 和 `CustomControl.data` 分别承载 0x0310 的 300 字节 data 与 0x0311 的
  30 字节 data，不包含 `0xA5`、`cmd_id` 或裁判串口外层 CRC。外层标准帧只属于 MCU 与图传模块
  之间的串口边界。
- `Aim_Tx.enem_color` 是历史字段名，当前按己方阵营解释：`1` 表示己方红、目标蓝，`0` 表示己方
  蓝、目标红；不得重新解释为通用敌方颜色枚举。
- `Aim_Rx` 的 yaw/pitch 直接使用上游 `yaw_rad`/`pitch_rad`，不转换单位；速度和加速度字段当前
  固定为零。
- MQTT `rx` 是窗口内收到的全部 `CustomByteBlock` 速率，包括无效消息；`rx_error` 累计 Protobuf、
  长度、帧头、颜色、CRC、填充、四元数和 ROS 发布错误。
- MQTT `tx` 是窗口内成功提交的 `CustomControl` 速率；`tx_error` 累计 JSON、编码和 MQTT 发布
  错误。主动限频丢弃和断线跳过不算错误。
- 已连接时统计格式为 `rx=Npackets/s rx_error=N tx=Npackets/s tx_error=N`。连接时记录
  `MQTT connected client_id=N`；断线时最多每秒记录一次
  `MQTT disconnected client_id=N reason=...`。

launch 必须使用 `output_format="{line}"`，控制台格式保持
`[INFO] [时间戳] [节点名]: 消息`，不增加进程名前缀。

## 实现与配置约定

- C++ 标准为 C++17。全部 C++ 文件统一使用根目录 `.clang-format`，包括适配器；子目录不得放置
  独立 `.clang-format`。修改 C++ 后格式化相关文件。
- 类型使用 `PascalCase`，函数和变量使用 `snake_case`，适配器成员变量以 `_` 结尾。旧模块存在
  局部差异时保持文件内一致，不做无关批量改名。
- 使用 RAII 管理线程、socket、相机、串口、FFmpeg 和 ROS 资源；所有退出路径必须能终止线程并
  释放设备。
- 共享数据必须有明确所有权和同步策略。
- 参数沿用现有 YAML 读取方式，只在启动时读取，不增加动态参数回调。新增或修改参数时同时提供
  合法范围校验和中文注释；注释说明用途、单位、范围以及对实时性或安全性的影响。
- 日志和用户可见错误使用英文；复杂算法、硬件和坐标系约束可以使用简洁中文注释。不要添加复述
  代码的注释。
- Python 保持现有模块边界和 pytest 风格，不在 ROS 回调中加入阻塞网络操作。

## 构建与验证流程

### 自瞄

`auto_aim/CMakeLists.txt` 是自瞄项目唯一的 CMake 入口。从仓库根目录构建到 `build/auto_aim/`，
不要在根目录添加包装 `CMakeLists.txt`：

```bash
cmake -S auto_aim -B build/auto_aim -DCMAKE_BUILD_TYPE=Release
cmake --build build/auto_aim -j "$(nproc)"
ctest --test-dir build/auto_aim --output-on-failure
```

ROS 2 构建环境不可用时，CMake 应跳过 `custom_client` 和 ROS 标定目标，自瞄核心及非 ROS 直接
测试仍须可构建。只修改单一模块时可以构建并运行直接相关目标；修改共享工具、I/O 或公共算法时
运行完整 CTest。

### 适配器

适配器从仓库根目录构建和测试，产物仍写入统一目录：

```bash
colcon --log-base log build --base-paths custom_client_adapter \
  --build-base build/custom_client_adapter --install-base install \
  --symlink-install --packages-select custom_client_adapter
colcon --log-base log test --build-base build/custom_client_adapter \
  --install-base install --packages-select custom_client_adapter
colcon test-result --test-result-base build/custom_client_adapter --verbose
```

### 依赖和验证结论

- CUDA Toolkit、cuDNN 和 ONNX Runtime 的系统安装命令只维护在根 README，不添加或调用仓库安装
  脚本。命令明确展示下载地址、软件包和有影响的系统写入位置。
- 依赖不得下载或缓存到 `build/`。CMake 缺少依赖时明确失败，不在配置阶段联网下载。
- 测试范围随风险扩大；共享接口、I/O、控制安全和协议改动不能只运行局部单元测试。
- 提交结论时区分“已在 Jazzy 构建/测试”和“保持 Kilted 源码兼容”，不得声称验证未实际运行的
  环境。

## 文档归属与完成条件

文档按读者和信息生命周期分工：

| 位置 | 应维护的内容 |
| --- | --- |
| 根 `README.md` | 新用户按顺序完成安装、首次配置、启动验收、ROSboard、标定、调参和故障拆分的流程 |
| `rm_video.yaml`、`rm_mqtt.yaml` | 适配器每个参数的中文用途、单位、范围和行为说明 |
| `docs/auv_client_ros_manual.md` | 对外字段、坐标系、时间同步、标定和安全语义 |
| `AGENTS.md` | 架构边界、实时性、内部协议、统计口径、工程和验证约束 |

- README 不展开 UDP 包头、CRC、重连状态机、线程/队列实现或逐字段参数表。用户需要识别的日志
  指标和故障处理方法可以保留，但不复制内部统计定义。
- README 的 ROS 命令假定 Jazzy 环境已配置，不重复添加 `/opt/ros/.../setup.bash`，也不要求用户
  手工设置由 ROS 环境提供的 `ROS_DISTRO`。只有 colcon overlay 确实为后续命令所需时才加载
  `install/setup.bash`。
- APT 依赖按活动构建、运行、维护或仓库工具的实际使用点维护，Python APT 依赖集中列出。NVIDIA
  驱动必须在 CUDA Toolkit/cuDNN 之前说明；配置完所需软件源后只执行一次 `apt update`。
- 安装章节只保留需要执行的命令、前置条件和有影响的系统写入位置，不添加常规安装后环境检查或
  解释临时下载不会进入仓库等显而易见的结果。
- 修改话题、消息类型、启动命令、依赖或目录时更新 README；修改 Custom Client 对外字段、同步、
  坐标系、标定或安全语义时更新接口手册；修改适配器实时性、协议、统计或编码约定时更新本文件。
- 每次协作形成且会持续影响实现、接口、文档或协作方式的结论，必须在同一次变更中提炼到相应
  文档。新增内容应融入读者的实际操作顺序和现有层级，必要时重组整节，避免孤立说明、规则重复
  或依赖对话语境。
