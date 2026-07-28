# Aim_Task 与 Custom Client 任务层协议

本文定义机器人 `Aim_Task`、RoboMaster 图传链路、自定义客户端 MQTT 接口和本仓库 ROS 2
适配器之间的任务层数据契约。裁判系统标准帧格式依据《RoboMaster 2026
机甲大师高校系列赛通信协议 V2.0.0（20260626）》；`Aim_Tx`、`Aim_Rx` 是本项目在标准
`0x0310`/`0x0311` data 内定义的私有报文。

本仓库不包含机器人 MCU 固件。固件内部任务、串口、DMA 和 VT03 复用方式应在对应固件仓库中
实现和验证；本文只规定与当前 `custom_client_adapter` 实现一致的线上数据和失效保护语义。

## 1. 链路与协议边界

```text
机器人 MCU
  Aim_Tx[43] + zero[257]
        │  裁判系统 0x0310（300-byte data，最高 50 Hz）
        ▼
图传链路 / 自定义客户端
  MQTT CustomByteBlock protobuf（data 恰好 300 bytes，QoS 0）
        ▼
rm_mqtt
  /rm_mqtt/imu + /rm_mqtt/self_is_red
        ▼
custom_client
  /auto_aim/result（JSON）
        ▼
rm_mqtt
  MQTT CustomControl protobuf（data 恰好 30 bytes，QoS 0，不 retain）
        │  裁判系统 0x0311（30-byte data，最高 75 Hz）
        ▼
机器人 MCU
  Aim_Rx[28] + zero[2]
```

边界说明：

- MCU 串口侧能看到裁判系统标准外层帧，包括 `SOF`、`data_length`、`seq`、外层 CRC8、
  `cmd_id` 和外层 CRC16。
- 本仓库的 `rm_mqtt` 节点看不到标准外层帧。它收到的是官方 MQTT Protobuf 消息，
  `CustomByteBlock.data` 已经是 `0x0310` 的 300 字节 data；发送的 `CustomControl.data` 会由
  图传链路封装为 `0x0311`。
- `Aim_Tx`/`Aim_Rx` 自带内层帧头和内层 CRC16。外层校验与内层校验用途不同，不能省略其中一层。
- 所有多字节整数使用小端序，浮点数为小端 IEEE 754 binary32。

## 2. 机器人到自定义客户端：0x0310

`0x0310` data 在本项目中固定为 300 字节。前 43 字节是 `Aim_Tx`，其余 257 字节必须全部为
`0x00`：

```text
data[300] = Aim_Tx[43] || zero[257]
```

### 2.1 Aim_Tx 布局

`Aim_Tx` 的 CRC16 覆盖偏移 `0..40`，校验值写在偏移 `41..42`，低字节在前。

| data 偏移 | 长度 | 字段 | 类型 | 任务层含义 | 当前适配器行为 |
| ---: | ---: | --- | --- | --- | --- |
| 0 | 1 | `head` | `uint8_t` | 内层帧头，固定 `0x53` | 严格校验 |
| 1 | 1 | `mode` | `uint8_t` | `0` 空闲、`1` 自瞄、`2` 小符、`3` 大符 | 不读取 |
| 2 | 1 | `enem_color` | `uint8_t` | 当前链路的阵营编码，见下表 | 严格校验并发布阵营 |
| 3 | 4 | `q[0]` | `float` | 四元数 `w` | 读取 |
| 7 | 4 | `q[1]` | `float` | 四元数 `x` | 读取 |
| 11 | 4 | `q[2]` | `float` | 四元数 `y` | 读取 |
| 15 | 4 | `q[3]` | `float` | 四元数 `z` | 读取 |
| 19 | 4 | `yaw` | `float` | 云台 yaw 状态，预留 | 不读取 |
| 23 | 4 | `yaw_vel` | `float` | 云台 yaw 角速度，预留 | 不读取 |
| 27 | 4 | `pitch` | `float` | 云台 pitch 状态，预留 | 不读取 |
| 31 | 4 | `pitch_vel` | `float` | 云台 pitch 角速度，预留 | 不读取 |
| 35 | 4 | `bullet_speed` | `float` | 弹速，预留 | 不读取；算法使用 YAML 的 `bullet_speed` |
| 39 | 2 | `bullet_count` | `uint16_t` | 子弹累计计数，预留 | 不读取 |
| 41 | 2 | `checksum` | `uint16_t` | 前 41 字节的 CRC16 | 严格校验 |

虽然历史字段名是 `enem_color`，当前上下游约定实际按“己方阵营”解释，不是通用颜色枚举：

| `enem_color` | `/rm_mqtt/self_is_red` | 己方 | 自瞄目标 |
| ---: | --- | --- | --- |
| `0` | `false` | 蓝方 | 红方 |
| `1` | `true` | 红方 | 蓝方 |

固件必须按这张表赋值。`custom_client` 未收到阵营时只发布安全结果；阵营变化时立即撤销旧命令、
重置 Tracker，并拒绝阵营版本已经过期的计算结果。

### 2.2 rm_mqtt 接收校验与 ROS 映射

`rm_mqtt` 按以下顺序处理 MQTT `CustomByteBlock`：

1. Protobuf 必须能够解析。
2. `data` 长度必须严格等于 300。
3. `data[0]` 必须为 `0x53`，`data[2]` 只能为 `0` 或 `1`。
4. `data[0..40]` 的 CRC16 必须等于 `data[41..42]`。
5. `data[43..299]` 必须全部为零。
6. 四元数四个分量必须为有限数，模长必须不小于 `1e-6`。

全部通过后，适配器将 `(w,x,y,z)` 重排为 ROS 的 `(x,y,z,w)` 并归一化，发布：

- `/rm_mqtt/imu`：`sensor_msgs/msg/Imu`。`orientation` 为归一化四元数；
  `angular_velocity_covariance[0]` 和 `linear_acceleration_covariance[0]` 为 `-1`。
- `/rm_mqtt/self_is_red`：`std_msgs/msg/Bool`，值按上表映射。

IMU 时间戳由 `rm_mqtt` 在回调中写为 `now() + timestamp_offset_sec`。任一校验失败时，两条 ROS
消息都不发布，并累计一次 `rx_error`；不会转发部分字段或沿用本帧数据。

## 3. 自定义客户端到机器人：0x0311

`rm_mqtt` 订阅 `/auto_aim/result`。JSON 中下列四个字段是编码 `Aim_Rx` 的必需字段；其他字段
（包括 `stamp`、`horizon_distance_m` 和 `latency_ms`）在该边界被忽略：

| JSON 字段 | 类型 | 单位 | 用途 |
| --- | --- | --- | --- |
| `control` | boolean | - | 是否交出有效云台控制指令 |
| `shoot` | boolean | - | 是否请求开火 |
| `yaw_rad` | finite number | rad | 绝对 yaw 目标角 |
| `pitch_rad` | finite number | rad | 绝对 pitch 目标角，向上为负 |

### 3.1 安全模式映射

| `control` | `shoot` | `control.allow_fire` | `Aim_Rx.mode` | 写入角度 |
| --- | --- | --- | ---: | --- |
| `false` | 任意 | 任意 | `0` | yaw/pitch 强制为 `0` |
| `true` | `false` | 任意 | `1` | 写入 JSON 的 yaw/pitch |
| `true` | `true` | `false` | `1` | 写入 JSON 的 yaw/pitch，不开火 |
| `true` | `true` | `true` | `2` | 写入 JSON 的 yaw/pitch，并请求开火 |

因此下发开火至少需要 `custom_client.yaml` 的 `auto_fire=true`、`rm_mqtt.yaml` 的
`control.allow_fire=true`，且本帧同时满足 `control=true` 和 `shoot=true`。`control=false` 的
安全结果即使携带非零角度也不会进入线协议。

### 3.2 Aim_Rx 布局

`0x0311` data 固定为 30 字节：前 28 字节为 `Aim_Rx`，最后 2 字节固定为零。

| data 偏移 | 长度 | 字段 | 类型 | 当前写入值 |
| ---: | ---: | --- | --- | --- |
| 0 | 1 | `head` | `uint8_t` | 固定 `0x50` |
| 1 | 1 | `mode` | `uint8_t` | `0` 不控制、`1` 控制不开火、`2` 控制并开火 |
| 2 | 4 | `yaw` | `float` | `yaw_rad`，弧度，不转换单位 |
| 6 | 4 | `yaw_vel` | `float` | `0.0` |
| 10 | 4 | `yaw_acc` | `float` | `0.0` |
| 14 | 4 | `pitch` | `float` | `pitch_rad`，弧度，不转换单位 |
| 18 | 4 | `pitch_vel` | `float` | `0.0` |
| 22 | 4 | `pitch_acc` | `float` | `0.0` |
| 26 | 2 | `checksum` | `uint16_t` | 前 26 字节的 CRC16，低字节在前 |
| 28 | 2 | padding | `uint8_t[2]` | 固定 `00 00` |

字节布局：

```text
50 mode yaw[4] 00[8] pitch[4] 00[8] inner_crc16[2] 00 00
```

适配器将完整 30 字节 data 包装为 MQTT `CustomControl` Protobuf，以 QoS 0、非 retain 发布。发送
频率由 `control.max_send_rate_hz` 限制且不得超过 75 Hz；超频消息立即丢弃，不缓存、不补发。
MQTT 断线时跳过控制消息。JSON、字段类型、有限数、float32 范围、编码或 MQTT 发布失败累计
`tx_error`。

## 4. 裁判系统外层帧

本节仅适用于 MCU 与图传模块之间的标准串口字节流；MQTT 适配器不解析或生成这一层。

| 偏移 | 长度 | 字段 | 类型 | 说明 |
| ---: | ---: | --- | --- | --- |
| 0 | 1 | `SOF` | `uint8_t` | 固定 `0xA5` |
| 1 | 2 | `data_length` | `uint16_t` | 小端 data 长度 |
| 3 | 1 | `seq` | `uint8_t` | 包序号，自然回绕 |
| 4 | 1 | `CRC8` | `uint8_t` | 覆盖偏移 `0..3` |
| 5 | 2 | `cmd_id` | `uint16_t` | 小端命令码 |
| 7 | n | `data` | `uint8_t[n]` | 本项目固定长度 data |
| 7+n | 2 | `CRC16` | `uint16_t` | 覆盖偏移 `0..(6+n)`，低字节在前 |

本项目两种完整外层帧为：

| 方向 | `cmd_id` | `data_length` | 整帧长度 | data 内容 |
| --- | ---: | ---: | ---: | --- |
| MCU -> 客户端 | `0x0310` | 300 | 309 | `Aim_Tx[43] + zero[257]` |
| 客户端 -> MCU | `0x0311` | 30 | 39 | `Aim_Rx[28] + zero[2]` |

```text
0x0310: A5 2C 01 seq crc8 10 03 Aim_Tx[43] zero[257] outer_crc16[2]
0x0311: A5 1E 00 seq crc8 11 03 Aim_Rx[28] 00 00 outer_crc16[2]
```

MCU 接收 `0x0311` 时至少应严格校验 SOF、外层 CRC8、`data_length=30`、`cmd_id=0x0311`、
外层 CRC16、两字节零填充、`Aim_Rx.head=0x50` 和内层 CRC16。校验失败不得更新控制目标；同时
固件必须设置独立的接收超时，超时后释放控制并禁止开火，不能无限沿用最后一帧有效命令。

## 5. CRC 规则

内层和外层 CRC 使用 RoboMaster 裁判系统算法：

- CRC8 初始值为 `0xFF`，外层帧头只对前 4 字节计算，结果写入第 5 字节。
- CRC16 初始值为 `0xFFFF`，反射多项式为 `0x8408`，无最终异或。
- CRC16 按字节更新：`crc = (crc >> 8) ^ table[(crc ^ byte) & 0xFF]`。
- CRC 字段不参与自身计算，结果按低字节、高字节顺序写入。
- 已知校验向量：ASCII `123456789` 的结果为 `0x6F91`。

内层 CRC16 只覆盖 `Aim_Tx` 或 `Aim_Rx` 自身指定的前缀；外层 CRC16 覆盖标准帧头、命令码和
完整固定长度 data，包括零填充。

## 6. 实时性与失效保护

- MCU 的 `0x0310` 发送频率不得超过 50 Hz；客户端的 `0x0311` 发送频率不得超过 75 Hz。
- 两个方向都不依赖重传。生产者应发送当前状态，消费者应允许丢包，并以新鲜数据优先。
- 不得用无界队列、补发或阻塞式重试保存历史控制消息。
- MCU 构造 `0x0310` 前应清零完整 300 字节 data，避免未初始化填充导致整帧被适配器拒绝。
- MCU 串口若与遥控等协议复用，必须使用可处理拆包、粘包和损坏后重新同步的流式解析器；具体
  串口和 DMA 实现不属于本仓库协议。
- 任一层校验失败、ROS 控制失效、MQTT 断线或控制接收超时时，都必须撤销开火许可；不得用旧的
  `mode=2` 维持开火。
