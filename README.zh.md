# limo-ros-mcp

AgileX LIMO ROS 1 的独立 MCP 服务。它把 `limo_ros` 中经过核对的接口转换成 Codex 可调用的 MCP 工具，并把所有会改变机器人状态的请求交给 ROSClaw 守护进程。

## 安全边界

- rosbridge 或本地 ROS CLI 只允许读取 `/limo_status`、`/odom`、`/imu` 和可选的 `/scan`。
- 不提供 ROS advertise/publish、`/cmd_vel`、串口、CAN 或厂商 SDK 工具。
- 导航只建模为高层 `/move_base` 能力，并经 `rosclawd` 的 `request_action` 路径提交。
- REAL 必须具备不可变 body snapshot、守护进程签发的许可、已验证执行器以及最终执行回执。

## 工具

| 工具 | 作用 |
| --- | --- |
| `limo_get_contract` | 返回带上游提交版本的接口与安全契约。 |
| `limo_probe_ros` | 只读检查 ROS 图和必需 topic。 |
| `limo_observe` | 读取一个白名单遥测消息。 |
| `limo_validate_navigation_goal` | 验证导航目标，不下发。 |
| `limo_get_runtime_status` | 读取 `rosclawd` 就绪状态。 |
| `limo_request_navigation` | 向 `rosclawd` 提交 SHADOW/REAL 导航申请。 |
| `limo_get_action_status` | 查询动作状态。 |
| `limo_get_execution_receipt` | 查询规范执行回执。 |
| `limo_emergency_stop` | 请求守护进程急停；物理急停仍是最终手段。 |

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

## 真机验证顺序

操作员先启动已有 LIMO ROS 栈。`transport=auto` 会在 rosbridge 不可用时回退到固定的只读 `rostopic`/`rosnode` 命令，因此 rosbridge 是可选项：

```bash
roslaunch limo_bringup limo_start.launch pub_odom_tf:=false
# 仅远程 websocket 访问需要：
roslaunch rosbridge_server rosbridge_websocket.launch port:=9090
```

验证顺序：

1. `limo_probe_ros` 必须看到 `/limo_status`、`/odom` 和 `/imu`。
2. `limo_observe(status)` 检查新鲜状态、`error_code == 0` 和已知运动模式。
3. 读取 odometry、imu、laser_scan；它们只证明观测链，不证明执行。
4. `limo_validate_navigation_goal` 在不下发的情况下验证目标。
5. 使用当前 body snapshot 提交 SHADOW。
6. 只有守护进程安全检查、已验证 REAL executor、物理急停和人工授权均满足时，才允许提交精确的 REAL 动作。
7. 必须用动作状态和执行回执判断结果，不能根据文字输出或 topic 变化猜测成功。

## 上游来源

- [`agilexrobotics/limo_ros`](https://github.com/agilexrobotics/limo_ros)：`4c78efc674cfc154012fe851fbda89c50be5b983`
- [`agilexrobotics/limo-doc`](https://github.com/agilexrobotics/limo-doc)：`d78a730163f50e1a5e5631ffd651444e0ac6abc5`
- [`ros-claw/rosclaw`](https://github.com/ros-claw/rosclaw)：只用于受控执行边界
