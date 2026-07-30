# limo-ros-mcp

AgileX LIMO ROS 1 的独立巡检 MCP 服务。它把 `limo_ros` 的底盘、激光、定位、导航、地图、诊断、TF 和 RGB-D 接口转换成 21 个 Codex 可调用工具，并把会改变机器人状态的请求交给 ROSClaw 守护进程。

本次真机接口验证、动态状态和巡检前置问题记录在
[`docs/ROS_INSPECTION_READINESS.md`](docs/ROS_INSPECTION_READINESS.md)。

## ROS 巡检接口

- 23 个具名观测契约覆盖 LIMO 驱动、AMCL、move_base、地图/costmap、诊断、TF、日志和文档中的 RGB-D 数据流。
- `limo_observe` 默认返回紧凑摘要；消息级调试时可显式设置 `include_raw=true`。
- `limo_sample_topic` 可采样 1-10 条消息，返回摘要和估算频率。
- 图像、点云、路径、激光数组和占用栅格默认只返回适合模型处理的统计量。
- Agent 进程仍不提供 ROS advertise/publish、`/cmd_vel`、串口、CAN 或厂商 SDK 工具。
- 导航只建模为高层 `/move_base` 能力，并经 `rosclawd` 的 `request_action` 路径提交。
- REAL 必须具备不可变 body snapshot、守护进程签发的许可、已验证执行器以及最终执行回执。
- `limo_get_patrol_readiness` 返回带 SHA-256 封印的 `limo.readiness.v1`：默认 5 秒有效期，包含 policy hash、观测接收时间、稳定检查项、blocker 与 warning。
- v0.3 的调用者自报 readiness 布尔值已废弃，只为 SHADOW 兼容保留；在 Navigation Contract v2 与 daemon-owned preflight 完成前，REAL 请求一律 fail closed。

## 工具

| 分组 | 工具 |
| --- | --- |
| 契约与 ROS graph | `limo_get_contract`、`limo_list_observations`、`limo_probe_ros`、`limo_get_topic_info` |
| 消息检查 | `limo_observe`、`limo_sample_topic` |
| 巡检快照 | `limo_get_base_state`、`limo_get_laser_summary`、`limo_get_localization_state`、`limo_get_navigation_state`、`limo_get_map_summary`、`limo_get_diagnostics`、`limo_get_transform_state`、`limo_get_patrol_readiness` |
| 参数验证 | `limo_validate_navigation_goal`、`limo_validate_velocity_command` |
| ROSClaw 控制面 | `limo_get_runtime_status`、`limo_request_navigation`、`limo_get_action_status`、`limo_get_execution_receipt`、`limo_emergency_stop` |

## 安装与 Codex 接入

PyPI 上的 `rosclaw` 名称不是本项目的正式发布包，因此必须从相邻代码仓安装：

```bash
uv venv --python 3.12
uv pip install -e ../rosclaw
uv pip install -e '.[dev]'

codex mcp add rosclaw-limo \
  --env ROSCLAW_PROJECT_ROOT=/absolute/path/to/rosclaw \
  -- /absolute/path/to/limo-ros-mcp/.venv/bin/python -m limo_ros_mcp.server
codex mcp list
```

添加后新开 Codex 会话，让客户端重新发现工具。

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
5. `limo_validate_navigation_goal` 在不下发的情况下验证目标。
6. 使用当前 body snapshot 提交 SHADOW。
7. 只有守护进程安全检查、已验证 REAL executor、物理急停和人工授权均满足时，才允许提交精确的 REAL 动作。
8. 必须用动作状态和执行回执判断结果，不能根据文字输出或 topic 变化猜测成功。

准备度证据、时钟与 fail-closed 语义详见 [`docs/READINESS_EVIDENCE_V1.md`](docs/READINESS_EVIDENCE_V1.md)。Readiness snapshot 只是关联证据，不能替代 daemon 在 dispatch 前重新执行的可信 preflight。

## 上游来源

- [`agilexrobotics/limo_ros`](https://github.com/agilexrobotics/limo_ros)：`4c78efc674cfc154012fe851fbda89c50be5b983`
- [`agilexrobotics/limo-doc`](https://github.com/agilexrobotics/limo-doc)：`d78a730163f50e1a5e5631ffd651444e0ac6abc5`
- [`ros-claw/rosclaw`](https://github.com/ros-claw/rosclaw)：只用于受控执行边界
