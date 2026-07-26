# AGENTS.md

## 适用范围

本文件适用于整个仓库。`custom_client_adapter/AGENTS.md` 对适配器子工作区提供更具体的实时性、
协议和日志约束；修改该目录时必须同时遵守两份文件，冲突时以更接近目标文件的约定为准。

## 项目目标

仓库由两个可独立构建、通过 ROS 2 标准消息集成的项目组成：

- `auto_aim/` 下的 `sp_vision`：C++17 自瞄框架，包含相机与电控 I/O、识别、解算、跟踪、规划、火控、
  标定和离线测试。非 ROS 功能必须继续支持独立构建。
- `custom_client_adapter/`：ROS 2 工作空间，负责 RoboMaster UDP 视频解码，以及裁判系统
  自定义客户端 MQTT 0x0310/0x0311 双向桥接。
- `auv_client`：两者的集成入口，使用适配器提供的压缩图像、IMU 和阵营信息，输出 JSON
  自瞄控制结果。

开发时优先保证数据实时性、控制安全和接口兼容，不能为了保留每一帧或每一条控制消息而引入
无界积压。

## 仓库结构

- `auto_aim/src/`：不同兵种和调试场景的应用入口。
- `auto_aim/tasks/auto_aim/`：识别、位姿解算、跟踪、瞄准和开火决策。
- `auto_aim/tasks/auto_buff/`、`auto_aim/tasks/omniperception/`：打符与全向感知功能。
- `auto_aim/io/`：工业相机、串口、IMU、云台和 ROS 2 I/O；`auto_aim/io/auv_client.*` 是集成接口。
- `auto_aim/calibration/`：相机内参、手眼标定和数据采集程序。
- `auto_aim/configs/`：自瞄及标定 YAML。修改字段时同步检查读取端和文档。
- `auto_aim/tests/`：自瞄项目的可执行测试和 CTest 测试。
- `docs/auv_client_ros_manual.md`：AUV Client 的权威对外接口说明。
- `start.sh`：从仓库根目录构建并启动适配器和自瞄端，负责交互读取 MQTT client ID。
- `custom_client_adapter/custom_client_adapter/`：ROS 2 功能包源码、参数、launch 与 pytest。
- `docs/`：AUV Client 接口、任务层协议和 RoboMaster 通信协议。
- `build/`、`install/`、`log/`：仓库根目录下的统一生成目录，不手工编辑或提交。源码子目录内
  不生成或保留编译产物。

## 集成接口

默认完整链路固定为：

| 话题 | ROS 类型 | 发布方 | 订阅方 |
| --- | --- | --- | --- |
| `/rm_video/image_processed` | `sensor_msgs/msg/CompressedImage` | `rm_video` | `auv_client` |
| `/rm_mqtt/imu` | `sensor_msgs/msg/Imu` | `rm_mqtt` | `auv_client` |
| `/rm_mqtt/self_is_red` | `std_msgs/msg/Bool` | `rm_mqtt` | `auv_client` |
| `/auto_aim/result` | `std_msgs/msg/String` | `auv_client` | `rm_mqtt` |

- 全部实时 ROS 话题使用 best-effort、keep last 1；不要改为 reliable 或增加深队列来掩盖消费端
  性能问题。
- 图像和 IMU 时间戳必须使用同一 ROS 时钟。图像/IMU 的固定上游延迟使用配置补偿，不在算法中
  添加隐藏常量。
- `self_is_red=true` 表示己方为红方、自瞄目标为蓝方；`false` 表示己方为蓝方、自瞄目标为红方。
- `auv_client` 未收到阵营、输入失效、命令超时或结果非有限数时必须输出安全命令，不能沿用旧的
  开火结果。
- `/auto_aim/result` 的 `yaw_rad`、`pitch_rad` 使用弧度。协议字段、单位、JSON 键或安全语义变更
  必须同步修改两侧实现、配置、根 README 和接口手册。

## 构建与验证

`auto_aim/CMakeLists.txt` 是自瞄项目唯一的 CMake 入口，从仓库根目录调用并将构建产物写入
`build/auto_aim/`；不要在仓库根目录添加包装用 `CMakeLists.txt`：

```bash
source /opt/ros/jazzy/setup.bash
cmake -S auto_aim -B build/auto_aim -DCMAKE_BUILD_TYPE=Release
cmake --build build/auto_aim -j "$(nproc)"
ctest --test-dir build/auto_aim --output-on-failure
```

未 source ROS 2 时，CMake 应跳过 `auv_client` 和 ROS 相关目标，其余目标仍应可构建。只改动某个
根项目模块时，可构建并运行直接相关目标；改动共享工具、I/O 或公共算法时应运行完整 CTest。

适配器也从仓库根目录构建和测试，生成内容统一写入根 `build/`、`install/` 和 `log/`：

```bash
source /opt/ros/jazzy/setup.bash
colcon --log-base log build --base-paths custom_client_adapter \
  --build-base build/custom_client_adapter --install-base install \
  --symlink-install --packages-select custom_client_adapter
colcon --log-base log test --build-base build/custom_client_adapter \
  --install-base install --packages-select custom_client_adapter
colcon test-result --test-result-base build/custom_client_adapter --verbose
```

提交验证结论时区分“已在 Jazzy 构建/测试”和“保持 Kilted 源码兼容”；未实际运行的环境不能
声明为已验证。

## C++ 与 Python 约定

- C++ 标准为 C++17。根项目使用根目录 `.clang-format`，适配器使用
  `custom_client_adapter/.clang-format`；修改 C++ 后格式化相关文件。
- 保持现有命名风格：类型使用 `PascalCase`，函数和变量使用 `snake_case`，适配器成员变量以
  `_` 结尾。旧模块存在局部差异时优先保持文件内一致，不做无关的批量改名。
- 使用 RAII 管理线程、socket、相机、串口、FFmpeg 和 ROS 资源；退出路径必须可终止线程并释放
  设备。
- 共享数据必须有清晰的所有权和同步策略。实时路径禁止无界队列、忙等、阻塞式重试和逐帧高频
  日志。
- 参数使用现有 YAML 读取方式。新增或修改参数时提供合法范围校验，并同步更新示例配置和用户
  文档。
- 日志及用户可见错误信息使用英文；复杂算法或硬件/坐标系约束可用简洁中文注释。不要添加仅仅
  复述代码的注释。
- Python 遵循现有模块边界和 pytest 风格，不在 ROS 回调中加入阻塞网络操作。

## 算法与安全约束

- 坐标系、四元数顺序、角度单位、图像分辨率缩放和时间戳语义属于对外契约。修改前先阅读
  `docs/auv_client_ros_manual.md`，并为转换边界补充测试。
- 检测、跟踪或解算失败时显式退化到安全状态；不得使用未初始化、过期或与当前阵营版本不一致
  的目标继续控制。
- 保持模型路径和类别/颜色顺序可配置。替换模型时同时验证输入尺寸、输出布局和颜色映射。
- 改动开火逻辑、阵营处理、超时、CRC 或 MQTT 控制编码属于高风险修改，必须增加针对正常路径、
  无效输入和失效保护的测试。
- 不提交真实机器人专属的敏感网络凭据。标定值和硬件 ID 若仅适用于单台设备，应明确其适用范围。

## 文档维护

- 根 `README.md` 面向整个仓库，维护总体架构、公共依赖、完整链路和自瞄算法说明。
- `custom_client_adapter/README.md` 面向适配器独立使用，维护参数、节点、协议、日志和调试方式。
- 修改话题、消息类型、启动命令、依赖或目录时同步更新相关 README。
- 修改 AUV Client 的字段、时间同步、坐标系、标定或安全语义时同步更新
  `docs/auv_client_ros_manual.md`。
- 修改适配器的实时性、统计语义或编码约定时同步更新 `custom_client_adapter/AGENTS.md`。
