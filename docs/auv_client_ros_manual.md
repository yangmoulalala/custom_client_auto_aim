# ROS 2 AUV Client 对外接口手册

本文档面向图像/IMU 发布端、云台控制端和现场部署人员，说明 `auv_client` 的 ROS 2 接口、数据格式、时间同步、坐标系、标定与安全约定。本文所述行为对应当前仓库中的 `io::AUVClient` 和 `auv_client` 可执行程序。

## 1. 功能与数据流

`auv_client` 用 ROS 2 标准消息替代项目原有的工业相机和 CBoard 输入，始终运行自瞄链路，不包含 CBoard 模式切换和打符分支。

```text
sensor_msgs/CompressedImage ─┐
                             ├─ 时间戳配对 → Detector → Solver → Tracker → Aimer → Shooter
sensor_msgs/Imu ─────────────┘                                                   │
                                                                       ▼
                                                     std_msgs/String（JSON）
```

基本信息：

| 项目 | 值 |
|---|---|
| 可执行程序 | `auv_client` |
| ROS 节点名 | `auv_client` |
| 默认配置文件 | `auto_aim/configs/AUVClient.yaml` |
| 输入 | 压缩图像、IMU 姿态 |
| 输出 | JSON 格式的绝对 yaw/pitch 自瞄指令 |
| ROS 消息依赖 | `rclcpp`、`sensor_msgs`、`std_msgs` |

`auv_client` 不依赖 `sp_msgs`。系统未安装或未 source ROS 2 时，CMake 只跳过 AUV Client 和 ROS 标定程序，原有非 ROS 程序仍可构建。

## 2. 构建与运行

先安装项目通用依赖和 ROS 2，并在执行 CMake 前 source 对应 ROS 2 环境。例如：

```bash
source /opt/ros/jazzy/setup.bash
cmake -S auto_aim -B build/auto_aim -DCMAKE_BUILD_TYPE=Release
cmake --build build/auto_aim -j
```

CMake 输出中出现以下内容表示标准 ROS 接口已启用：

```text
ROS2 standard messages found, compiling AUV client I/O.
```

首次运行前，必须在 `auto_aim/configs/AUVClient.yaml` 中填写真实标定结果，特别是 `calibration_image_width` 和 `calibration_image_height`。这两个值为 `0` 时程序会拒绝启动。

```bash
./build/auto_aim/auv_client auto_aim/configs/AUVClient.yaml
```

不传路径时也默认使用 `auto_aim/configs/AUVClient.yaml`：

```bash
./build/auto_aim/auv_client
```

配置通过 YAML 加载，不是 ROS 参数；修改配置后需要重启进程。

## 3. ROS 话题接口

### 3.1 接口总表

| 方向 | 默认话题 | ROS 类型 | QoS |
|---|---|---|---|
| 输入 | `/camera/image_raw` | `sensor_msgs/msg/CompressedImage` | best effort、volatile、keep last 1 |
| 输入 | `/imu/data` | `sensor_msgs/msg/Imu` | best effort、volatile、keep last 1 |
| 输出 | `/auto_aim/result` | `std_msgs/msg/String` | best effort、volatile、keep last 1 |
| 输出 | `/auto_aim/debug` | `sensor_msgs/msg/CompressedImage` | best effort、volatile、keep last 1 |

可在 YAML 中修改话题名：

```yaml
ros_image_topic: "/camera/image_raw"
ros_imu_topic: "/imu/data"
ros_result_topic: "/auto_aim/result"
ros_debug_topic: "/auto_aim/debug"
```

发布端和 `auv_client` 预期位于同一主机。图像和 IMU 发布端应使用与订阅端兼容的传感器 QoS，并将队列深度设为 1。DDS 实现支持时，建议启用共享内存传输。

### 3.2 图像输入

消息类型为 `sensor_msgs/msg/CompressedImage`。使用字段如下：

| 字段 | 要求 |
|---|---|
| `header.stamp` | 与 IMU 共用同一 ROS 时钟，表示上游完成对齐后的发布时间 |
| `header.frame_id` | 当前版本不读取，不触发 TF 查询 |
| `format` | 记录压缩格式；解码由 OpenCV 根据负载内容完成 |
| `data` | 必须包含可解码的压缩图像负载 |

程序使用 OpenCV 将负载解码为三通道 BGR 图像。负载为空、损坏或无法解码时拒绝该帧并输出
安全结果；单通道压缩图像会在解码时扩展为 BGR。

首个有效图像决定本次运行的固定输入分辨率。实际输入可以与内参标定分辨率不同，但宽高比相对误差必须不超过 1%，此时程序按宽、高分别缩放 `fx`、`fy`、`cx`、`cy`，畸变参数保持不变。运行中改变图像分辨率会拒绝该帧并发布安全结果，不会跨分辨率延续 Tracker 状态。

### 3.3 IMU 输入

消息类型为 `sensor_msgs/msg/Imu`。当前版本只使用 `orientation`，忽略 `angular_velocity` 和 `linear_acceleration`。

| 字段 | 要求 |
|---|---|
| `header.stamp` | 与对应图像使用同一 ROS 时钟和对齐后的时间语义 |
| `header.frame_id` | 当前版本不读取，不触发 TF 查询 |
| `orientation.x/y/z/w` | IMU body 到绝对/世界方向的四元数，所有分量必须为有限数 |
| `orientation_covariance[0]` | 为 `-1` 时表示姿态不可用，该样本会被拒绝 |

ROS 消息中的四元数排列为 `x, y, z, w`。程序内部按 Eigen 的构造顺序 `w, x, y, z` 读入，并在使用前归一化。零四元数或模长过小的四元数无效。

当前版本不使用 `tf2`，因此仅填写 `frame_id` 不能纠正 IMU 安装方向。安装轴向差异必须通过 `R_gimbal2imubody` 配置。

### 3.4 调试图像输出

存在 `/auto_aim/debug` 订阅者时，程序会为每个成功处理的输入帧生成包含检测框、EKF 模型、
瞄准点和 Tracker 状态的 JPEG 调试图。消息类型为 `sensor_msgs/msg/CompressedImage`，其
`header.stamp` 与对应输入图像一致。

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

`shoot` 还受 YAML 中 `auto_fire` 控制；`auto_fire: false` 时不会给出开火许可。

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

对于已经收到图像、但图像无效、IMU 无法配对、姿态无效、图像过期或分辨率变化等错误，安全结果可能保留该图像的 `stamp` 和计算出的非负 `latency_ms`，但控制字段仍满足：

```text
control = false
shoot = false
yaw_rad = pitch_rad = horizon_distance_m = 0
```

协议故意不增加 `status` 字段。下游必须把 `control == false` 作为立即释放/忽略自瞄控制权的依据，不得无限期保持上一条有效指令；`shoot == false` 必须立即撤销开火许可。

## 5. 时间戳、同步与低延迟策略

### 5.1 上游时间戳约定

本系统接受“图像时间戳不是曝光时刻”的前提。`Image.header.stamp` 和 `Imu.header.stamp` 被解释为上游完成图像/姿态对齐后的发布时间，并且必须来自同一个 ROS 时钟。

该时间戳有两个用途：

1. 配对图像和 IMU。
2. 估算消息发布后到自瞄输出之间的延迟，并将其加入目标预测时间。

采集/曝光到上游发布时间之间没有体现在 `header.stamp` 中的固定延迟，应填入 `upstream_latency_ms`。

### 5.2 配对规则

配对按以下顺序执行：

1. 优先选择与图像时间戳完全相同的 IMU。
2. 没有完全相同时，选择时间差绝对值最小且不超过 `sync_tolerance_ms` 的 IMU。
3. 暂时没有可用 IMU 时，最多等待 `sync_wait_ms`。
4. 等待结束仍不能配对则拒绝该图像，并输出安全结果。

默认参数为：

```yaml
sync_tolerance_ms: 5
sync_wait_ms: 10
```

图像使用单槽“最新帧”缓存：处理期间到达的新图像会覆盖尚未处理的旧图像，不会形成应用层历史帧积压。IMU 使用最多 200 条的环形历史缓存。

### 5.3 帧龄与预测补偿

用于过期判断和 Aimer `to_now=true` 预测的有效帧龄为：

```text
ros_age_ms = max(0, ROS_now - Image.header.stamp)
effective_age_ms = ros_age_ms + upstream_latency_ms
```

`effective_age_ms > max_frame_age_ms` 时拒绝该帧。默认值：

```yaml
upstream_latency_ms: 0.0
max_frame_age_ms: 100
```

`upstream_latency_ms` 只对每一帧的内部稳态时间作固定前移，所以会增加预测补偿，但不会改变 Tracker 看到的相邻帧时间间隔。

输出中的 `latency_ms` 定义为：

```text
latency_ms = max(0, result_publish_ros_time - Image.header.stamp)
```

它不包含配置的 `upstream_latency_ms`，也不能表述为“相机曝光到结果发布”的完整端到端延迟。

如果图像时间戳比当前 ROS 时间超前超过 `sync_tolerance_ms`，该帧会被视为时钟异常并拒绝。使用仿真时钟时，所有发布端和 `auv_client` 必须使用一致的 ROS 时钟配置。

### 5.4 输出看门狗

`command_timeout_ms` 默认是 `200 ms`。超过该时间没有任何新结果时，看门狗发布一次无源帧安全结果，防止下游持续执行旧指令。持续没有输入时不会不断重复发送同一看门狗帧；恢复新结果后，看门狗重新计时。

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

| 配置项 | 默认值 | 单位 | 说明 |
|---|---:|---|---|
| `ros_image_topic` | `/camera/image_raw` | — | 图像输入话题 |
| `ros_imu_topic` | `/imu/data` | — | IMU 输入话题 |
| `ros_result_topic` | `/auto_aim/result` | — | JSON 结果输出话题 |
| `ros_debug_topic` | `/auto_aim/debug` | — | JPEG 调试图像输出话题 |
| `bullet_speed` | `23.0` | m/s | 弹丸初速，必须大于 0 |
| `upstream_latency_ms` | `0.0` | ms | 采集到上游发布时间的固定估计延迟 |
| `sync_tolerance_ms` | `5` | ms | 图像/IMU 最大配对时间差 |
| `sync_wait_ms` | `10` | ms | 等待匹配 IMU 的最长时间 |
| `max_frame_age_ms` | `100` | ms | 包含上游固定延迟的最大有效帧龄 |
| `command_timeout_ms` | `200` | ms | 输出看门狗超时 |

所有时间和弹速参数必须为有限数；延迟不得为负，并且 `upstream_latency_ms` 必须小于 `max_frame_age_ms`。

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

### 7.4 自瞄与火控参数

`enemy_color`、Detector、Tracker、Aimer 和 Shooter 参数沿用项目原有 UAV 自瞄链路。与外部控制接口直接相关的参数包括：

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

完整默认项和注释见 `auto_aim/configs/AUVClient.yaml`。

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
pattern_rows: 6
square_size_mm: 75
ros_image_topic: "/calibration/image_raw"
ros_imu_topic: "/rm_mqtt/imu"
```

`auv_client` 运行时订阅 `CompressedImage`，但两个标定工具订阅的是原始 `sensor_msgs/Image`。
当前适配器输出需要先临时解码，并通过 QoS override 兼容视频发布端的 best-effort QoS：

```bash
ros2 run image_transport republish --ros-args \
  -p in_transport:=compressed \
  -p out_transport:=raw \
  -p qos_overrides./rm_video/image_processed.subscription.reliability:=best_effort \
  -r in/compressed:=/rm_video/image_processed \
  -r out:=/calibration/image_raw
```

标定使用的必须是最终送入自瞄的 processed 画面。标定后改变视频 ROI、旋转、宽高比或相机安装
位置时应重新标定；转换进程会保留原消息时间戳，以便手眼工具继续与 IMU 配对。

### 8.1 相机内参标定

运行：

```bash
./build/auto_aim/ros_calibrate_camera auto_aim/configs/calibration.yaml \
  --output-folder=auto_aim/assets/ros_camera_calibration
```

操作流程：

1. 保持发布端持续发布固定分辨率原始图像。
2. 标定板被正确识别时按 `s` 接受当前帧。
3. 让标定板覆盖画面中心、四角、远近位置和不同倾角，建议采集至少 10 帧。
4. 按 `q` 结束采样并求解。

输出目录包含采集图像和 `calibration_result.yaml`。将以下字段复制到 `auto_aim/configs/AUVClient.yaml`，也复制到手眼标定使用的 `auto_aim/configs/calibration.yaml`：

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
4. 每个有效姿态按 `s` 保存一组同步 Image/IMU 样本。
5. 至少需要 3 组有效姿态，建议采集 10 组以上且旋转差异明显的姿态。
6. 按 `q` 求解。

输出目录包含图像、对应四元数文本和 `handeye_result.yaml`。将以下字段复制到 `auto_aim/configs/AUVClient.yaml`：

- `R_gimbal2imubody`
- `R_camera2gimbal`
- `t_camera2gimbal`

手眼算法实际求解 `R_camera2gimbal` 和 `t_camera2gimbal`。`R_gimbal2imubody` 是机械轴向映射，由配置读入、校验后原样写回结果；它不会由普通手眼方程自动辨识。

## 9. 上下游集成要求

### 9.1 图像/IMU 发布端

发布端必须满足：

- 两条消息使用同一 ROS 时钟和一致的 `header.stamp` 语义。
- 上游已经完成“某帧图像对应哪个姿态”的时间对齐。
- `CompressedImage.data` 可被 OpenCV 解码，且解码后的图像分辨率和宽高比在运行期间保持固定。
- IMU 发布 body 到 absolute/world 的有效、有限、非零四元数。
- 采用与 best-effort 传感器订阅兼容的 QoS，建议 depth 1。
- 同机高带宽图像场景优先启用 DDS 共享内存，并检查实际 DDS 配置是否生效。

如果发布端无法使用对齐后的同时间戳，可发布各自真实时间，但差值必须落在 `sync_tolerance_ms` 内；节点只做最近邻配对，不做姿态插值。

### 9.2 控制/发射接收端

接收端必须满足：

- 解析 `std_msgs/String.data` 中的 JSON，并检查字段类型。
- `control == false` 时立即忽略角度并退出/释放自瞄控制。
- 只在 `control == true && shoot == true` 时允许开火。
- 自行检查结果接收超时和 `stamp` 新鲜度，不无限保持上一条命令。
- 按“yaw 世界绝对角、pitch 向上为负”的约定对接控制器。
- 不依赖协议中不存在的 `status` 字段。

## 10. 检查与排障

可以先检查 ROS 图和 QoS：

```bash
ros2 node info /auv_client
ros2 topic info --verbose /camera/image_raw
ros2 topic info --verbose /imu/data
ros2 topic echo /auto_aim/result
```

常见问题：

| 现象 | 可能原因 | 处理 |
|---|---|---|
| CMake 不生成 `auv_client` | ROS 环境未 source，或缺少标准消息包 | source ROS 2 后重新执行 CMake，检查 `rclcpp/sensor_msgs/std_msgs` |
| 启动立即退出 | 标定宽高为 0、YAML 缺字段或时间参数非法 | 写入真实标定尺寸并检查 `auto_aim/configs/AUVClient.yaml` |
| 一直输出 `control=false` | 无目标、图像非法、IMU 无效、配对失败或帧过期 | 查看日志，并分别 echo 两个输入话题 |
| 图像有数据但节点拒绝 | 压缩负载损坏、宽高比错误或解码后尺寸在运行中改变 | 检查视频解码/编码错误并固定分辨率 |
| 经常提示 IMU 无法匹配 | 两路时钟/时间戳语义不一致或容差过小 | 修正上游对齐，测量时间差后谨慎调整同步参数 |
| 提示图像过期 | ROS 时钟不同步、处理堵塞或上游延迟未配置合理 | 统一时钟，降低队列深度，检查 `upstream_latency_ms` 和帧率 |
| yaw/pitch 方向错误 | IMU 四元数方向、NED/ENU 或外参轴向错误 | 按第 6 节逐级验证 `R_gimbal2imubody` 和 `R_camera2gimbal` |
| 静止时角度正确、转动后错误 | IMU body→world 方向或轴映射错误 | 在手眼标定预览中检查 yaw/pitch/roll 方向 |
| 近距离大致正确、远距离偏差明显 | 内参、畸变、外参、弹速或单位错误 | 重新标定并确认平移为米、弹速为 m/s |
| `latency_ms` 小于实际端到端延迟 | 时间戳是上游发布时间而非曝光时间 | 这是预期行为；用 `upstream_latency_ms` 补偿固定前段延迟 |

## 11. 协议版本注意事项

当前接口以固定 JSON 字段集对外提供结果，没有单独的 ROS 自定义消息、状态码或 TF 接口。集成方应按字段名解析 JSON，不依赖 JSON 键顺序或浮点数字符串精度。未来若扩展协议，建议在保持现有字段语义的前提下增加显式版本字段，并同步更新本手册。
