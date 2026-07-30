# ROS 2 Custom Client 对外接口手册

本文档面向图像/IMU 发布端、云台控制端和现场部署人员，说明 `custom_client` 的 ROS 2 接口、
数据格式、时间同步、坐标系、标定与安全约定。本文所述行为对应当前仓库中的 `io::AUVClient`、
`custom_client` 可执行程序及 `rm_mqtt` 桥接实现。`0x0310`/`0x0311` 的字节布局另见
`docs/Aim_Task_Protocol.md`。

## 1. 功能与数据流

`custom_client` 用 ROS 2 标准消息替代项目原有的工业相机和 CBoard 输入，始终运行自瞄链路，不包含 CBoard 模式切换和打符分支。

```text
/rm_video/image_processed (CompressedImage) ─┐
/rm_mqtt/imu (Imu) ──────────────────────────┼─ 配对 → YOLOv5/ORT → Solver
/rm_mqtt/self_is_red (Bool) ─────────────────┘                    → Tracker → Aimer → Shooter
                                                                             │
                                      /auto_aim/debug (CompressedImage) <─────┤
                                      /auto_aim/result (String/JSON) <────────┘
                                                                             │
                                                                  rm_mqtt → MQTT 0x0311
```

基本信息：

| 项目 | 值 |
|---|---|
| 可执行程序 | `custom_client` |
| ROS 节点名 | `custom_client` |
| ROS 日志前缀 | `auto_aim` |
| 默认配置文件 | `auto_aim/configs/custom_client.yaml` |
| 输入 | 压缩图像、IMU 姿态、己方阵营 |
| 输出 | JSON 格式的绝对 yaw/pitch 自瞄指令 |
| ROS 消息依赖 | `rclcpp`、`sensor_msgs`、`std_msgs` |

`custom_client` 不依赖 `sp_msgs`。系统未安装或未 source ROS 2 时，CMake 跳过 Custom Client 和 ROS
标定程序，仍构建自瞄核心和非 ROS 直接测试。YOLOv8/11、传统检测器、全向感知和能量机关检测器
源码保留，但不进入默认构建。

## 2. 构建与运行

先按根 README 的“安装依赖”章节，全局安装 ONNX Runtime 1.28.0、CUDA Toolkit 12.8 和
cuDNN 9。安装命令及其系统改动均直接列在 README 中；CMake 只从 `/opt/onnxruntime` 与系统
CUDA 路径查找，不会下载依赖到 `build/`。然后安装项目通用依赖和 ROS 2，并在执行 CMake 前
source 对应 ROS 2 环境。例如：

```bash
source /opt/ros/jazzy/setup.bash
cmake -S auto_aim -B build/auto_aim -DCMAKE_BUILD_TYPE=Release
cmake --build build/auto_aim -j "$(nproc)"
```

CMake 输出中出现以下内容表示标准 ROS 接口已启用：

```text
ROS2 standard messages found, compiling custom client I/O.
```

完整链路优先从仓库根目录启动：

```bash
./start.sh
./start.sh --show
```

脚本依次构建自瞄和适配器，再启动 `rm_video`、`rm_mqtt` 和带 `--debug` 的 `custom_client`。
它只识别可选的 `--show`；其他参数会被忽略，MQTT client ID 由适配器自动探测。

首次运行前，必须在 `auto_aim/configs/custom_client.yaml` 中填写真实标定结果，特别是
`calibration_image_width` 和 `calibration_image_height`。这两个值不为正数时程序会拒绝启动。
仅运行自瞄端时使用：

```bash
./build/auto_aim/custom_client auto_aim/configs/custom_client.yaml
```

不传路径时也默认使用 `auto_aim/configs/custom_client.yaml`：

```bash
./build/auto_aim/custom_client
```

可选的 `--debug` 每秒输出输入、识别、跟踪和控制统计；`--show` 创建本地 OpenCV 调试窗口。

配置通过 YAML 加载，不是 ROS 参数；修改配置后需要重启进程。

活动模型固定为 `auto_aim/models/0526.onnx`，模型颜色顺序为 `blue_red_gray_purple`。`device` 只
接受 `CPU` 或 `GPU`，默认 `GPU`。GPU 模式使用 CUDA Provider，并禁止模型节点回退 CPU；Provider
不可用、发生回退或模型输入输出契约不匹配时均启动失败。CPU 模式显式使用 ORT CPU Provider，
不会加载 OpenVINO。两种模式启动时都执行预热。

## 3. ROS 话题接口

### 3.1 接口总表

| 方向 | 默认话题 | ROS 类型 | QoS |
|---|---|---|---|
| 输入 | `/rm_video/image_processed` | `sensor_msgs/msg/CompressedImage` | best effort、volatile、keep last 1 |
| 输入 | `/rm_mqtt/imu` | `sensor_msgs/msg/Imu` | best effort、volatile、keep last 1 |
| 输入 | `/rm_mqtt/self_is_red` | `std_msgs/msg/Bool` | best effort、volatile、keep last 1 |
| 输出 | `/auto_aim/result` | `std_msgs/msg/String` | best effort、volatile、keep last 1 |
| 输出 | `/auto_aim/debug` | `sensor_msgs/msg/CompressedImage` | best effort、volatile、keep last 1 |

可在 YAML 中修改话题名：

```yaml
ros_image_topic: "/rm_video/image_processed"
ros_imu_topic: "/rm_mqtt/imu"
ros_self_is_red_topic: "/rm_mqtt/self_is_red"
ros_result_topic: "/auto_aim/result"
ros_debug_topic: "/auto_aim/debug"
```

发布端和 `custom_client` 预期位于同一主机。图像和 IMU 发布端应使用与订阅端兼容的传感器 QoS，并将队列深度设为 1。DDS 实现支持时，建议启用共享内存传输。

### 3.2 图像输入

消息类型为 `sensor_msgs/msg/CompressedImage`。使用字段如下：

| 字段 | 要求 |
|---|---|
| `header.stamp` | 与 IMU 共用同一 ROS 时钟；当前适配器写入 `rm_video.now()+timestamp_offset_sec` |
| `header.frame_id` | 当前版本不读取，不触发 TF 查询 |
| `format` | 仅作格式描述；程序不据此选择解码器，OpenCV 按负载内容解码 |
| `data` | 必须包含可解码的压缩图像负载 |

程序使用 OpenCV 将负载解码为三通道 BGR 图像。负载为空、损坏或无法解码时拒绝该帧并输出
安全结果；单通道压缩图像会在解码时扩展为 BGR。

首个成功解码的图像决定本次运行的固定输入分辨率。实际输入可以与内参标定分辨率不同，但宽高比
相对误差必须不超过 1%；此时程序按宽、高分别缩放 `fx`、`fy`、`cx`、`cy`，畸变参数保持不变。
首帧尺寸无效或宽高比不兼容时，程序发布安全结果并以失败状态退出。运行中尺寸变化时仅拒绝该帧
并发布安全结果；固定尺寸不会自动切换，Tracker 也不会因这一帧自动重置。

### 3.3 IMU 输入

消息类型为 `sensor_msgs/msg/Imu`。当前版本只使用 `orientation`，忽略 `angular_velocity` 和 `linear_acceleration`。

| 字段 | 要求 |
|---|---|
| `header.stamp` | 与图像共用同一 ROS 时钟；当前适配器写入 `rm_mqtt.now()+timestamp_offset_sec` |
| `header.frame_id` | 当前版本不读取，不触发 TF 查询 |
| `orientation.x/y/z/w` | IMU body 到绝对/世界方向的四元数，所有分量必须为有限数 |
| `orientation_covariance[0]` | 为 `-1` 时表示姿态不可用，该样本会被拒绝 |

ROS 消息中的四元数排列为 `x, y, z, w`。程序内部按 Eigen 的构造顺序 `w, x, y, z` 读入，并在使用前归一化。零四元数或模长过小的四元数无效。

当前版本不使用 `tf2`，因此仅填写 `frame_id` 不能纠正 IMU 安装方向。安装轴向差异必须通过 `R_gimbal2imubody` 配置。

### 3.4 阵营输入

消息类型为 `std_msgs/msg/Bool`，没有时间戳：

| `data` | 己方 | 自瞄目标 |
|---|---|---|
| `true` | 红方 | 蓝方 |
| `false` | 蓝方 | 红方 |

收到首条阵营消息前，节点会正常读取图像和 IMU，但只发布 `control=false`、`shoot=false` 的安全
结果。阵营首次到达或发生变化时，节点立即发布一条无源帧安全结果；目标颜色变化时 Tracker 会
重置。每次算法结果还携带其计算开始时的阵营版本，发布前若版本已经变化，该结果会被替换为安全
结果，避免旧阵营目标在竞态中下发。

该 Bool 来自 `Aim_Tx.enem_color` 的项目约定映射。历史字段名容易误解，实际映射以
`docs/Aim_Task_Protocol.md` 为准。

### 3.5 调试图像输出

存在 `/auto_aim/debug` 订阅者时，程序会为每个成功处理的输入帧生成包含检测框、EKF 模型、
瞄准点、Tracker 状态和当前匹配装甲板解算距离的 JPEG 调试图。距离单位为 m；本帧没有匹配到
当前目标或距离无效时，左上状态面板显示 `Distance: --`，不沿用上一帧或 EKF 预测距离。消息类型为
`sensor_msgs/msg/CompressedImage`，其 `header.stamp` 与对应输入图像一致。

JPEG 编码在独立线程执行，待编码槽位只保留最新一帧；编码速度低于处理速度时会覆盖旧调试帧，
不会积压并影响控制链路。发布器始终启用，不受 `--debug` 或 `--show` 控制；没有订阅者时跳过
调试图绘制和 JPEG 编码。传入 `--show` 时，程序按每个成功处理帧显示本地 OpenCV 窗口；窗口
显示不受 ROS 订阅状态限制，不传时不会创建窗口。

## 4. 输出帧格式

输出消息类型为 `std_msgs/msg/String`，其 `data` 是单行 JSON，不附加换行，固定包含以下字段：

```json
{
  "stamp": {
    "sec": 1785000000,
    "nanosec": 123456789
  },
  "control": true,
  "shoot": false,
  "yaw_rad": 0.125,
  "pitch_rad": -0.042,
  "horizon_distance_m": 0.0,
  "latency_ms": 8.7
}
```

### 4.1 字段定义

| JSON 字段 | 类型 | 单位 | 定义 |
|---|---|---|---|
| `stamp.sec` | integer | s | 源图像 `header.stamp.sec` |
| `stamp.nanosec` | unsigned integer | ns | 源图像 `header.stamp.nanosec` |
| `control` | boolean | — | 下游是否应执行本帧 yaw/pitch 指令 |
| `shoot` | boolean | — | 本帧是否允许开火 |
| `yaw_rad` | number | rad | 项目世界坐标系中的绝对目标 yaw |
| `pitch_rad` | number | rad | 项目控制器约定下的绝对 pitch；向上为负，向下为正 |
| `horizon_distance_m` | number | m | 水平目标距离预留字段；当前自瞄链路通常输出 `0.0` |
| `latency_ms` | number | ms | 从源图像时间戳到结果发布时刻的 ROS 时钟差 |

`yaw_rad` 和 `pitch_rad` 是绝对角度指令，不是相对当前云台姿态的增量，也不是角速度。下游只有在 `control == true` 时才可使用它们。开火条件应至少同时满足 `control == true && shoot == true`。

`shoot` 还受 YAML 中 `auto_fire` 控制；`auto_fire: false` 时不会给出开火许可。最终下发开火还要
经过适配器的 `control.allow_fire` 总开关。

### 4.2 安全结果

在尚未关联到具体输入帧的超时/看门狗场景，完整安全结果为：

```json
{
  "stamp": {"sec": 0, "nanosec": 0},
  "control": false,
  "shoot": false,
  "yaw_rad": 0.0,
  "pitch_rad": 0.0,
  "horizon_distance_m": 0.0,
  "latency_ms": -1.0
}
```

对于已经收到图像、但图像无效、IMU 无法配对、姿态无效、图像过期或分辨率变化等错误，安全结果
可能保留该图像的 `stamp` 和计算出的非负 `latency_ms`，但控制字段仍满足：

```text
control = false
shoot = false
yaw_rad = pitch_rad = horizon_distance_m = 0
```

未收到阵营、阵营变化、结果对应的阵营版本过期，或 yaw、pitch、水平距离出现非有限数时，也会
发布安全结果。协议故意不增加 `status` 字段。下游必须把 `control == false` 作为立即释放/忽略
自瞄控制权的依据，不得无限期保持上一条有效指令；`shoot == false` 必须立即撤销开火许可。

### 4.3 rm_mqtt 到 0x0311 的映射

`rm_mqtt` 只消费 JSON 的 `control`、`shoot`、`yaw_rad` 和 `pitch_rad`；四个字段都必须存在且
类型正确，两个角度必须是有限数。其余字段不会写入 `Aim_Rx`。

| 条件 | `Aim_Rx.mode` | yaw/pitch |
|---|---:|---|
| `control=false` | `0` | 强制写零 |
| `control=true, shoot=false` | `1` | 直接写入弧度值 |
| `control=true, shoot=true, control.allow_fire=false` | `1` | 直接写入弧度值 |
| `control=true, shoot=true, control.allow_fire=true` | `2` | 直接写入弧度值 |

速度和加速度字段固定为零。适配器按 `control.max_send_rate_hz` 立即限频，超频消息不排队、不补发；
MQTT 断线时跳过发送。完整 30 字节布局和 CRC 见 `docs/Aim_Task_Protocol.md`。

## 5. 时间戳、同步与低延迟策略

### 5.1 上游时间戳约定

当前完整适配器没有相机曝光时间或 MCU 采样时间：

- `rm_video` 在解码帧进入图像处理/JPEG 提交路径时写入
  `Image.header.stamp = rm_video.now() + rm_video.timestamp_offset_sec`。
- `rm_mqtt` 在有效 `0x0310` MQTT 回调中写入
  `Imu.header.stamp = rm_mqtt.now() + rm_mqtt.timestamp_offset_sec`。

因此两条时间戳必须来自同一个 ROS 时钟，但通常不会完全相等，也不能直接解释为曝光或 IMU 采样
时刻。两个适配器偏移用于补偿两路固定相对偏差，`custom_client` 再按最近邻进行配对。项目没有额外
的动态参数回调；修改任一 YAML 后都要重启对应节点。

该时间戳有两个用途：

1. 配对图像和 IMU。
2. 估算消息发布后到自瞄输出之间的延迟，并将其加入目标预测时间。

图像实际采集到 `header.stamp` 所代表时刻之间仍未体现的非负固定延迟，应填入
`upstream_latency_ms`。它只作用于自瞄内部预测和帧龄判断，不会改写 ROS 消息时间戳。调整视频
`timestamp_offset_sec` 后应重新核对该值，避免重复补偿同一段延迟。

### 5.2 配对规则

配对按以下顺序执行：

1. 优先选择与图像时间戳完全相同的 IMU。
2. 没有完全相同时，选择时间差绝对值最小且不超过 `sync_tolerance_ms` 的 IMU。
3. 暂时没有可用 IMU 时，最多等待 `sync_wait_ms`。
4. 等待结束仍不能配对则拒绝该图像，并输出安全结果。

当前仓库 `auto_aim/configs/custom_client.yaml` 为：

```yaml
sync_tolerance_ms: 500
sync_wait_ms: 1000
```

这两个值决定姿态误配和等待延迟的上限，应根据录包分析结果收紧，不能用来掩盖两路时钟或固定
偏移错误。字段从 YAML 缺失时，C++ 回退值分别是 `5 ms` 和 `10 ms`。

图像使用单槽“最新帧”缓存：处理期间到达的新图像会覆盖尚未处理的旧图像，不会形成应用层历史帧积压。IMU 使用最多 200 条的环形历史缓存。

### 5.3 帧龄与预测补偿

用于过期判断和 Aimer `to_now=true` 预测的有效帧龄为：

```text
ros_age_ms = max(0, ROS_now - Image.header.stamp)
effective_age_ms = ros_age_ms + upstream_latency_ms
```

`effective_age_ms > max_frame_age_ms` 时拒绝该帧。当前仓库配置为：

```yaml
upstream_latency_ms: 150.0
max_frame_age_ms: 1000
```

字段缺失时的 C++ 回退值分别是 `0 ms` 和 `100 ms`。

`upstream_latency_ms` 只对每一帧的内部稳态时间作固定前移，所以会增加预测补偿，但不会改变 Tracker 看到的相邻帧时间间隔。

输出中的 `latency_ms` 定义为：

```text
latency_ms = max(0, result_publish_ros_time - Image.header.stamp)
```

它不包含配置的 `upstream_latency_ms`，也不能表述为“相机曝光到结果发布”的完整端到端延迟。

如果图像时间戳比当前 ROS 时间超前超过 `sync_tolerance_ms`，该帧会被视为时钟异常并拒绝。适配器
不提供项目级 `use_sim_time` 特判；无论使用系统时钟还是仿真时钟，所有发布端和 `custom_client`
都必须看到一致的 ROS 时钟。

### 5.4 输出看门狗

`command_timeout_ms` 同时用于等待新图像和输出看门狗。当前仓库配置为 `1000 ms`，字段缺失时的
C++ 回退值为 `200 ms`。超过该时间没有新结果时，看门狗发布无源帧安全结果；主循环等待图像超时
时也会发布安全结果。因此持续断流期间仍会按该超时量级周期性产生安全结果，而不会高频忙等或
积压消息。任一新结果发布后重新计时。

下游仍应实现自己的接收超时保护，不能仅依赖本节点看门狗。

## 6. 坐标系与变换定义

所有旋转矩阵均为右手系中的主动旋转，采用行主序写入 YAML。下文记 `R_a2b` 为“把在 `a` 坐标系表达的向量转换为在 `b` 坐标系表达”。平移向量单位均为米。

### 6.1 Camera optical 坐标系

采用 OpenCV 光学坐标约定：

- 原点：相机光心。
- `+X_camera`：图像向右。
- `+Y_camera`：图像向下。
- `+Z_camera`：镜头光轴向前。

像素坐标原点位于图像左上角，`u` 向右、`v` 向下。

### 6.2 Gimbal 坐标系

采用右手 FLU 坐标系：

- 原点：云台旋转中心。
- `+X_gimbal`：云台零位朝前。
- `+Y_gimbal`：向左。
- `+Z_gimbal`：向上。

### 6.3 World 坐标系

采用右手局部 FLU 坐标系：

- 原点：与云台旋转中心重合。
- `+X_world`：零 yaw 朝前。
- `+Y_world`：向左。
- `+Z_world`：向上。
- 正 yaw：从 `+X_world` 转向 `+Y_world`；从 `+Z_world` 向原点看为逆时针。

本项目只建模方向，不建模机器人在全局地图中的平移，因此这里的 `world` 不是 ROS 地图中的 `map` 或 `odom`。节点不会发布或查询这些 TF。

### 6.4 IMU body 与绝对坐标系

`imubody` 由实际 IMU 安装方向定义。输入四元数必须表示 IMU body 到 IMU 绝对/世界方向的旋转，记为 `R_imubody2imuabs`，并遵循 ROS/REP-103 的右手姿态约定。

`R_gimbal2imubody` 用于描述云台轴与 IMU body 轴的安装映射。项目按下式生成云台到项目世界的旋转：

```text
R_gimbal2world =
    R_gimbal2imubodyᵀ
    · R_imubody2imuabs
    · R_gimbal2imubody
```

这条公式是当前实现的精确定义。若上游给出的是 world 到 body、NED 姿态，或不同轴顺序，必须先在上游转换，不能只修改 `frame_id`。

### 6.5 相机到云台外参

相机 PnP 得到的点按下式转换到云台坐标：

```text
p_gimbal = R_camera2gimbal · p_camera + t_camera2gimbal
```

再转换到项目世界坐标：

```text
p_world = R_gimbal2world · p_gimbal
```

外参字段定义：

| 字段 | 尺寸 | 单位 | 定义 |
|---|---|---|---|
| `R_camera2gimbal` | 3×3 | — | 将 camera optical 坐标中的向量主动旋转到 gimbal 坐标 |
| `t_camera2gimbal` | 3×1 | m | 相机光心在 gimbal 坐标中的位置 |
| `R_gimbal2imubody` | 3×3 | — | 云台轴与 IMU body 轴的右手安装映射 |

理想安装的轴向关系通常为：

```text
camera +Z（前） → gimbal +X（前）
camera +X（右） → gimbal -Y（右）
camera +Y（下） → gimbal -Z（下）
```

不要因为这个理想关系而直接复制示例矩阵；应以实际安装和标定结果为准。

### 6.6 输出角度约定

`yaw_rad` 为世界 FLU 坐标系中瞄准点的绝对方位角：

```text
yaw = atan2(y_world, x_world) + yaw_offset
```

`pitch_rad` 使用项目控制器符号约定，向上为负、向下为正。它已经包含弹道下坠补偿和 `pitch_offset`：

```text
pitch = -(ballistic_elevation + pitch_offset)
```

因此下游不应再次反号、再次加入弹道补偿或将其解释为标准 ROS 绕 `+Y` 的欧拉角，除非控制器接口明确进行了对应转换。

## 7. YAML 配置参考

### 7.1 ROS、同步和安全参数

下表区分仓库当前 YAML 的有效值和字段缺失时的 C++ 回退值。部署行为以实际加载的 YAML 为准，
回退值不是建议调参值。

| 配置项 | 当前仓库值 | 缺省回退值 | 单位 | 说明 |
|---|---:|---:|---|---|
| `ros_image_topic` | `/rm_video/image_processed` | `/camera/image_raw` | - | 压缩图像输入话题 |
| `ros_imu_topic` | `/rm_mqtt/imu` | `/imu/data` | - | IMU 输入话题 |
| `ros_self_is_red_topic` | `/rm_mqtt/self_is_red` | `/rm_mqtt/self_is_red` | - | 阵营输入；当前 YAML 未显式写出，使用回退值 |
| `ros_result_topic` | `/auto_aim/result` | `/auto_aim/result` | - | JSON 结果输出话题 |
| `ros_debug_topic` | `/auto_aim/debug` | `/auto_aim/debug` | - | JPEG 调试图像输出话题 |
| `bullet_speed` | `24.0` | `23.0` | m/s | 弹丸初速，必须为正 |
| `upstream_latency_ms` | `150.0` | `0.0` | ms | 未体现在图像时间戳中的固定估计延迟 |
| `sync_tolerance_ms` | `500` | `5` | ms | 图像/IMU 最大配对时间差 |
| `sync_wait_ms` | `1000` | `10` | ms | 等待匹配 IMU 的最长时间 |
| `max_frame_age_ms` | `1000` | `100` | ms | 包含上游固定延迟的最大有效帧龄 |
| `command_timeout_ms` | `1000` | `200` | ms | 输入等待和输出看门狗超时 |

所有时间和弹速参数必须为有限数；延迟不得为负，并且 `upstream_latency_ms` 必须小于 `max_frame_age_ms`。
虽然配置校验允许较小正弹速，Aimer 当前会把小于 `14 m/s` 的值替换为 `23 m/s`。

### 7.2 相机内参

```yaml
calibration_image_width: 1280
calibration_image_height: 720
camera_matrix: [fx, 0, cx, 0, fy, cy, 0, 0, 1]
distort_coeffs: [k1, k2, p1, p2, k3]
```

`calibration_image_width/height` 是生成该 `camera_matrix` 时使用的真实图像尺寸，不是任意期望输出尺寸。畸变系数顺序为 OpenCV 的 `[k1, k2, p1, p2, k3]`。

### 7.3 手眼外参

```yaml
R_gimbal2imubody: [r00, r01, r02, r10, r11, r12, r20, r21, r22]
R_camera2gimbal:  [r00, r01, r02, r10, r11, r12, r20, r21, r22]
t_camera2gimbal:  [tx, ty, tz]
```

矩阵按行展开。`t_camera2gimbal` 单位为米。`R_gimbal2imubody` 必须是正交、行列式为 `+1` 的右手旋转矩阵。

### 7.4 识别、自瞄与火控参数

活动识别配置为：

```yaml
yolo_name: yolov5
yolov5_model_path: auto_aim/models/0526.onnx
yolov5_color_order: blue_red_gray_purple
device: GPU
min_confidence: 0.8
```

模型必须有且仅有一个 FP16 `[1,3,640,640]` 输入和一个 FP32 `[1,25200,22]` 输出。图像保持现有
左上角 letterbox、BGR 到 RGB、`1/255` 归一化、关键点顺序、阈值和 NMS 语义。`0708.onnx` 和
`models/` 中其他模型只归档备用。

装甲 PnP 尺寸按当前赛制固定：英雄使用 `230 x 56 mm` 大装甲，哨兵、工程、3/4/5 号步兵、
前哨站和基地均使用 `135 x 56 mm` 小装甲。当前不保留平衡步兵或两装甲底盘模型；0526 模型的
两个历史基地类别均按小装甲基地处理。基地仍使用三块装甲、`0.3205 m` 回转半径的跟踪布局。

`enemy_color`、Tracker、Aimer 和 Shooter 参数沿用项目原有 UAV 自瞄链路。与外部控制接口直接相关的参数包括：

| 配置项 | 单位 | 说明 |
|---|---|---|
| `yaw_offset` | degree | yaw 固定补偿 |
| `pitch_offset` | degree | pitch/弹道固定补偿 |
| `high_speed_delay_time` | s | 高转速目标附加预测时间 |
| `low_speed_delay_time` | s | 低转速目标附加预测时间 |
| `first_tolerance` | degree | 近距离开火角度阈值 |
| `second_tolerance` | degree | 远距离开火角度阈值 |
| `judge_distance` | m | 近/远开火阈值切换距离 |
| `auto_fire` | boolean | 是否允许输出 `shoot=true` |

完整当前配置和注释见 `auto_aim/configs/custom_client.yaml`。

### 7.5 2026 前哨站预测

前哨站沿用小装甲 PnP 模型。三块装甲按 `120 degree` 间隔分布，回转半径为 `0.275 m`，装甲
中心高度相邻相差 `0.102 m`。由于首次观测的运动学 ID 不固定对应低、中、高装甲，Tracker 会根据
已观测 ID 的相对高度，在六种合法高度排列中持续选择残差最小的一种。

三块装甲尚未全部被观测时，Aimer 只使用当前 `last_id`；全部观测后恢复跨板预测。选板保留原有
`70/30 degree` 方向窗口，静止或方向条件没有候选时仅在 `30 degree` 正面窗口内回退。无合法
瞄准点时输出 `control=false`、`shoot=false`。前哨站 EKF 收敛且 `|angular_velocity| > 2 rad/s`
时仍将转速吸附到 `+/-2.51 rad/s`。高低速附加预测时间使用角速度绝对值与 `decision_speed` 比较，
因此正反转采用相同阈值。

## 8. 标定流程

ROS 标定工具使用棋盘格，板参数配置在 `auto_aim/configs/calibration.yaml`。`pattern_cols` 和
`pattern_rows` 是内部角点数，不是黑白格数量：

```yaml
pattern_cols: 10
pattern_rows: 7
square_size_mm: 50
ros_image_topic: "/rm_video/image_processed"
ros_imu_topic: "/rm_mqtt/imu"
```

两个标定工具均以 best-effort、keep last 1 直接订阅 `/rm_video/image_processed` 的 JPEG
`sensor_msgs/msg/CompressedImage` 并在进程内解码，不需要 `image_transport republish`。标定使用的
必须是最终送入自瞄的 processed 画面；改变视频 ROI、旋转、宽高比或相机安装位置时应重新标定。

### 8.1 相机内参标定

运行：

```bash
./build/auto_aim/ros_calibrate_camera auto_aim/configs/calibration.yaml \
  --output-folder=auto_aim/assets/ros_camera_calibration
```

操作流程：

1. 保持适配器持续发布固定分辨率、与运行时一致的 processed `CompressedImage`。
2. 标定板被正确识别时按 `s` 接受当前帧。
3. 让标定板覆盖画面中心、四角、远近位置和不同倾角，建议采集至少 10 帧。
4. 按 `q` 结束采样并求解。

输出目录包含采集图像和 `calibration_result.yaml`。将以下字段复制到 `auto_aim/configs/custom_client.yaml`，也复制到手眼标定使用的 `auto_aim/configs/calibration.yaml`：

- `calibration_image_width`
- `calibration_image_height`
- `camera_matrix`
- `distort_coeffs`

### 8.2 相机—云台手眼标定

完成内参标定并将结果写入 `auto_aim/configs/calibration.yaml` 后运行：

```bash
./build/auto_aim/ros_calibrate_handeye auto_aim/configs/calibration.yaml \
  --output-folder=auto_aim/assets/ros_handeye_calibration
```

操作流程：

1. 固定标定板，整个采样过程不得移动标定板。
2. 按实际机械安装方向预先填写 `R_gimbal2imubody`。
3. 转动云台到不同 yaw、pitch、roll；预览窗口会显示由 IMU 计算的角度，可用它检查轴向和正负号。
4. 每个有效姿态按 `s` 保存一组按 `header.stamp` 最近邻匹配的图像/IMU 样本。
5. 至少需要 3 组有效姿态，建议采集 10 组以上且旋转差异明显的姿态。
6. 按 `q` 求解。

输出目录包含图像、对应四元数文本和 `handeye_result.yaml`。将以下字段复制到 `auto_aim/configs/custom_client.yaml`：

- `R_gimbal2imubody`
- `R_camera2gimbal`
- `t_camera2gimbal`

手眼算法实际求解 `R_camera2gimbal` 和 `t_camera2gimbal`。`R_gimbal2imubody` 是机械轴向映射，由配置读入、校验后原样写回结果；它不会由普通手眼方程自动辨识。

手眼工具与 `custom_client` 使用相同的最近邻规则：图像只匹配当前 200 条有界 IMU 缓存中时间差
绝对值最小且不超过 `sync_tolerance_ms` 的样本。暂时无匹配样本时最多等待 `sync_wait_ms`；超时后
丢弃图像，不插值、不外推，也不改写任一消息的时间戳。

## 9. 上下游集成要求

### 9.1 图像、IMU 与阵营发布端

发布端必须满足：

- 图像和 IMU 使用同一 ROS 时钟；当前适配器分别按各自节点的 `now()+timestamp_offset_sec` 写入。
- 测量并配置两路固定时间偏移，使真实对应样本的时间差落在 `sync_tolerance_ms` 内。
- `CompressedImage.data` 可被 OpenCV 解码，且解码后的图像分辨率和宽高比在运行期间保持固定。
- IMU 发布 body 到 absolute/world 的有效、有限、非零四元数。
- 发布有效的 `/rm_mqtt/self_is_red`；`true` 表示己方红、目标蓝，`false` 表示己方蓝、目标红。
- 全部实时话题使用 best-effort、volatile、keep last 1。
- 同机高带宽图像场景优先启用 DDS 共享内存，并检查实际 DDS 配置是否生效。

`custom_client` 和手眼标定工具只做最近邻配对，不做姿态插值。不要通过增大 QoS depth、
`sync_tolerance_ms` 或 `sync_wait_ms` 保存历史数据来掩盖时间戳偏移和处理性能问题。

### 9.2 控制/发射接收端

接收端必须满足：

- 解析 `std_msgs/String.data` 中的 JSON，并检查字段类型。
- `control == false` 时立即忽略角度并退出/释放自瞄控制。
- 只在 `control == true && shoot == true` 时允许开火。
- 自行检查结果接收超时和 `stamp` 新鲜度，不无限保持上一条命令。
- 按“yaw 世界绝对角、pitch 向上为负”的约定对接控制器。
- 不依赖协议中不存在的 `status` 字段。

当前 `rm_mqtt` 会校验 JSON 字段和有限数，但不会根据 `stamp` 拒绝过期结果；它在 ROS 回调中直接
编码并发布。因此机器人 MCU 必须设置独立的 `0x0311` 接收超时，并在超时、校验失败或
`mode=0` 时撤销开火和旧控制。

## 10. 检查与排障

可以先检查 ROS 图和 QoS：

```bash
ros2 node info /custom_client
ros2 topic info --verbose /rm_video/image_processed
ros2 topic info --verbose /rm_mqtt/imu
ros2 topic echo --qos-reliability best_effort --once /rm_mqtt/self_is_red
ros2 topic echo --qos-reliability best_effort /auto_aim/result
```

常见问题：

| 现象 | 可能原因 | 处理 |
|---|---|---|
| CMake 不生成 `custom_client` | ROS 环境未 source，或缺少标准消息包 | source ROS 2 后重新执行 CMake，检查 `rclcpp/sensor_msgs/std_msgs` |
| CMake 提示找不到 ONNX Runtime/CUDA | 尚未全局安装依赖，或安装位置被修改 | 按根 README 的“安装依赖”章节安装；自定义 ORT 位置时设置 `ONNXRUNTIME_ROOT` |
| 启动立即退出 | CUDA Provider/模型契约异常、标定宽高为 0、YAML 缺字段或参数非法 | 查看启动错误；确认 `device`、0526 模型、CUDA/cuDNN 和标定配置 |
| 一直输出 `control=false` | 未收到阵营、无目标、图像非法、IMU 无效、配对失败或帧过期 | 查看日志，分别 echo 三个输入话题 |
| 图像有数据但节点拒绝 | 压缩负载损坏、宽高比错误或解码后尺寸在运行中改变 | 检查视频解码/编码错误并固定分辨率 |
| 经常提示 IMU 无法匹配 | 两路时钟/时间戳语义不一致或容差过小 | 修正上游对齐，测量时间差后谨慎调整同步参数 |
| 提示图像过期 | ROS 时钟不同步、处理堵塞或上游延迟未配置合理 | 统一时钟，降低队列深度，检查 `upstream_latency_ms` 和帧率 |
| yaw/pitch 方向错误 | IMU 四元数方向、NED/ENU 或外参轴向错误 | 按第 6 节逐级验证 `R_gimbal2imubody` 和 `R_camera2gimbal` |
| 静止时角度正确、转动后错误 | IMU body→world 方向或轴映射错误 | 在手眼标定预览中检查 yaw/pitch/roll 方向 |
| 近距离大致正确、远距离偏差明显 | 内参、畸变、外参、弹速或单位错误 | 重新标定并确认平移为米、弹速为 m/s |
| `latency_ms` 小于实际端到端延迟 | 时间戳是上游发布时间而非曝光时间 | 这是预期行为；用 `upstream_latency_ms` 补偿固定前段延迟 |
| ROS 有 `shoot=true` 但 MCU 不开火 | `control=false`、适配器 `control.allow_fire=false`、MQTT 断线或 MCU 超时 | 逐层检查 JSON、rm_mqtt 统计、0x0311 mode 和两级开火开关 |

## 11. 协议版本注意事项

当前接口以固定 JSON 字段集对外提供结果，没有单独的 ROS 自定义消息、状态码或 TF 接口。集成方
应按字段名解析 JSON，不依赖 JSON 键顺序或浮点数字符串精度。字段、单位、时间语义、坐标系或
安全语义变更时，必须同步修改 `custom_client`、`rm_mqtt`、根 README、
`docs/Aim_Task_Protocol.md` 和本手册。
