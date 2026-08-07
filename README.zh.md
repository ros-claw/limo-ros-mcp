# limo-ros-mcp

AgileX LIMO ROS 1 的独立巡检 MCP 服务。默认 `core` profile 只暴露 10 个有界工具；`inspection` 和 `full` profile 提供完整的 ROS、RGB-D、音频、显示与 Jetson 接口。会改变机器人状态的请求仍全部交给 ROSClaw 守护进程。

本次真机接口验证、动态状态和巡检前置问题记录在
[`docs/ROS_INSPECTION_READINESS.md`](docs/ROS_INSPECTION_READINESS.md)。
USB、音频、显示、触摸屏、相机健康与 Jetson 证据边界详见
[`docs/PERIPHERAL_INSPECTION.md`](docs/PERIPHERAL_INSPECTION.md)。

## ROS 巡检接口

- 27 个具名 ROS 观测契约覆盖 LIMO 驱动、AMCL、move_base、地图/costmap、诊断、TF、日志，以及 Dabai 彩色、深度、红外、点云和标定流。
- 只读外设工具会将手册中的外设与 USB、ALSA、framebuffer、触摸屏、温度、内存和磁盘真机证据交叉核验。
- `limo_measure_microphone` 只在内存中采集 1–3 秒，返回 RMS/峰值后立即丢弃样本，不保存或返回音频内容。
- `limo_capture_camera_frame` 优先使用 loopback rosbridge；原始 RGB 大消息在老旧
  rosbridge 上阻塞时，自动回退到随包固定的只读 ROS 1 helper。回退同样不发布话题、
  不接受任意 topic 或输出路径，只返回经过尺寸、字节数与 SHA-256 校验的单帧 PNG。
- `limo_observe` 默认返回紧凑摘要；消息级调试时可显式设置 `include_raw=true`。
- `limo_sample_topic` 可采样 1-10 条消息，返回摘要和估算频率。
- 图像、点云、路径、激光数组和占用栅格默认只返回适合模型处理的统计量。
- Agent 进程仍不提供 ROS advertise/publish、`/cmd_vel`、串口、CAN 或厂商 SDK 工具。
- 导航只建模为高层 `/move_base` 能力，并经 `rosclawd` 的 `request_action` 路径提交。
- Navigation Contract v2 将请求绑定到同一 MCP 进程生成且未过期的 readiness snapshot，并要求 body snapshot 与 readiness 中使用的完全一致。
- 目标校验覆盖运维侧 route policy、地图 YAML/PGM 哈希、map 坐标系、地图边界、地理围栏、禁行区、占用栅格、净空、航向和容差。
- REAL 必须具备 daemon-owned 可信 preflight、守护进程签发的许可、已验证执行器以及最终执行回执。
- `limo_request_initial_pose(execution_mode="REAL")` 会在同一次 MCP 调用里显示精确动作确认；接受后 permit 由 ROSClaw 内部注入，Agent 不再填写或看到 permit ID。
- `limo_get_readiness` 默认返回紧凑的 SHA-256 封印 readiness 引用；旧名 `limo_get_patrol_readiness` 只在 `--compat-tools` 或 `--profile full` 下提供。
- 有界 `ObservationHub` 会复用只读传输、缓存新鲜摘要、合并并发 readiness 请求，并在 transport generation 变化时使旧快照失效。
- 调用者自报的 readiness 布尔值已经删除。REAL 导航会在守护进程内重新检查 AMCL、雷达、底盘状态、TF 和 move_base，并通过对话框确认精确目标。

## 工具

默认 `core` 包含 `limo_get_context`、`limo_observe`、`limo_get_readiness`、导航验证、三个受控请求、动作状态、回执和急停。使用 `--profile inspection` 获取全部只读诊断，或用 `--profile full` 获取完整工具面。

| 分组 | 工具 |
| --- | --- |
| 契约与 ROS graph | `limo_get_contract`、`limo_list_observations`、`limo_probe_ros`、`limo_get_topic_info` |
| 消息检查 | `limo_observe`、`limo_sample_topic` |
| 巡检快照 | `limo_get_base_state`、`limo_get_laser_summary`、`limo_get_localization_state`、`limo_get_navigation_state`、`limo_get_map_summary`、`limo_get_diagnostics`、`limo_get_transform_state`、`limo_get_patrol_readiness` |
| 相机与外设 | `limo_get_camera_state`、`limo_capture_camera_frame`、`limo_get_robot_pose`、`limo_get_dabai_device_state`、`limo_list_peripherals`、`limo_get_audio_state`、`limo_measure_microphone`、`limo_get_display_state`、`limo_get_platform_health` |
| 参数验证 | `limo_validate_navigation_goal`、`limo_validate_velocity_command` |
| ROSClaw 控制面 | `limo_get_runtime_status`、`limo_request_navigation`、`limo_request_initial_pose`、`limo_request_tone`、`limo_request_speech`、`limo_get_action_status`、`limo_get_execution_receipt`、`limo_emergency_stop` |

## 安装与 Codex 接入

PyPI 上的 `rosclaw` 名称不是本项目的正式发布包，因此必须从相邻代码仓安装：

```bash
uv venv --python 3.12
uv pip install -e ../rosclaw
uv pip install -e '.[dev]'

codex mcp add rosclaw-limo \
  --env ROSCLAW_PROJECT_ROOT=/absolute/path/to/rosclaw \
  --env ROS_PACKAGE_PATH=/absolute/path/to/catkin_ws/src:/opt/ros/melodic/share \
  --env LD_LIBRARY_PATH=/absolute/path/to/catkin_ws/devel/lib:/opt/ros/melodic/lib \
  --env CMAKE_PREFIX_PATH=/absolute/path/to/catkin_ws/devel:/opt/ros/melodic \
  -- /absolute/path/to/limo-ros-mcp/.venv/bin/python -m limo_ros_mcp.server --profile core
codex mcp list
```

当获准的巡检策略使用 `package://` 地图资源时，必须配置 `ROS_PACKAGE_PATH`；
TF 检查调用 `rospack`、`rosrun` 等 ROS 辅助程序时还必须配置 `LD_LIBRARY_PATH`
和 `CMAKE_PREFIX_PATH`。缺失时导航验证或 readiness 会在下发前按失败关闭。
首次添加服务器或工具 schema 变化后，
需要让 Codex 重新发现工具。REAL
确认还要求 Codex 允许 MCP elicitation，并由用户本人审核，而不是自动审核：

```toml
approval_policy = { granular = { mcp_elicitations = true } }
approvals_reviewer = "user"
```

普通只读、SHADOW 和 MCP 协议测试不需要新开 Codex 会话；已运行会话是否会
热更新工具 schema 取决于客户端版本。

`limo_get_runtime_status` 会返回 `mcp_process` 与 `interaction_plane`。更新代码后，
如果 `mcp_process.restart_required=true`，说明常驻 MCP 进程仍在执行旧代码；如果
`interaction_plane.daemon_restart_required=true`，说明当前 rosclawd 尚未加载 Operator
Broker。两项都恢复为 `false` 后，才应使用重构后的 REAL 确认链路。拉取代码本身不会
热更新已经运行的 MCP server 或 rosclawd。

操作员启动 ROS 栈后，可运行只读真机集成测试：

```bash
LIMO_LIVE_ROS=1 .venv/bin/pytest tests/test_live_ros.py -q -m live_ros
```

## 真机验证顺序

操作员先启动已有 LIMO ROS 栈。`transport=auto` 会在 rosbridge 不可用时回退到固定的只读 `rostopic`/`rosnode` 命令，因此 rosbridge 是可选项：

```bash
roslaunch limo_bringup limo_start.launch pub_odom_tf:=false
# 仅远程 websocket 访问需要：
roslaunch rosbridge_server rosbridge_websocket.launch port:=9090
```

验证顺序：

1. `limo_probe_ros` 检查驱动、AMCL、move_base、地图与传感器 graph。
2. `limo_get_topic_info` 核对关键 topic 类型、发布者和订阅者。
3. `limo_sample_topic` 验证状态与导航消息字段和频率。
4. `limo_get_patrol_readiness` 聚合底盘、激光、定位、导航、地图、global/local costmap、诊断和 map→odom→base TF 证据；阈值来自 operator-owned 的 [`configs/limo_readiness_policy.yaml`](configs/limo_readiness_policy.yaml)。
5. `limo_validate_navigation_goal` 依据 [`configs/patrol_lab.example.yaml`](configs/patrol_lab.example.yaml) 固定的地图哈希、围栏、占用栅格与容差，在不下发的情况下验证目标。
6. 使用同一 MCP 进程刚生成的未过期 readiness snapshot hash，以及其中绑定的 body snapshot hash 提交 SHADOW。
7. 只有守护进程安全检查、已验证 REAL executor、物理急停和人工授权均满足时，才允许提交精确的 REAL 动作。
8. 必须用动作状态和执行回执判断结果，不能根据文字输出或 topic 变化猜测成功。

准备度证据、导航契约与 fail-closed 语义详见 [`docs/READINESS_EVIDENCE_V1.md`](docs/READINESS_EVIDENCE_V1.md) 和 [`docs/NAVIGATION_CONTRACT_V2.md`](docs/NAVIGATION_CONTRACT_V2.md)。Readiness snapshot 只是关联证据，不能替代 daemon 在 dispatch 前重新执行的可信 preflight。

本版本除观测扬声器增益外，还提供 `limo_request_tone`：仅允许
440/660/880 Hz、0.2–1.0 秒、5–25% 临时音量的合成短音。固定版本的守护进程执行器
只选择唯一的 USB PnP 声卡，播放后恢复原混音状态，不接收文件、命令、混音器名称或
设备参数。上游驱动仍没有提供前置 OLED 或车身 RGB 灯接口，真机也没有枚举出独立功放
的电源、温度或故障接口。

0.9.0 新增 `limo_request_speech`，支持最长 80 字符的普通话或英语短句。固定的 daemon
worker 不经过 shell，直接调用本机 eSpeak-NG 库，在内存中把 PCM 归一化到 10–25% 音量，
仅通过白名单 USB 扬声器播放，用车载麦克风验证声能增益，立即丢弃采样并恢复原混音状态。
该闭环证明声学输出，不声称识别或验证了具体语义内容。

`scripts/limo_find_person_greet.py` 默认仅使用本地 Whisper 转写。只有操作员显式传入
`--cloud-asr` 时才会发送录音；此时还必须在受保护的进程环境中设置
`ROSCLAW_LIMO_GOOGLE_SPEECH_API_KEY`。密钥不得写入脚本、配置仓库、MCP 参数或执行回执。

## 上游来源

- [`agilexrobotics/limo_ros`](https://github.com/agilexrobotics/limo_ros)：`4c78efc674cfc154012fe851fbda89c50be5b983`
- [`agilexrobotics/limo-doc`](https://github.com/agilexrobotics/limo-doc)：`d78a730163f50e1a5e5631ffd651444e0ac6abc5`
- [`ros-claw/rosclaw`](https://github.com/ros-claw/rosclaw)：只用于受控执行边界
