> 新对接文档的 `followMode` / `accompanyMode` 已单独接入，字段、功能及状态格式见 [跟随与伴随接口](FOLLOW_ACCOMPANY.md)。本文保留旧 `targetFollowing` 接口说明。

直线加速至50节默认使用实测控制器响应，安全检查兼顾能力上限；适用范围与开关见[实测响应说明](../MOTION_LIMITS.md)。

运动约束与曲线接口见[统一运动参数](../MOTION_LIMITS.md)。

# 动态目标追踪

参考 [USV2DSimulator 的距离跟随实现](https://github.com/cocoder777/USV2DSimulator/blob/bda51be4fe68f27d8b787b631a6f981a67eff6ca/jev_api/tools/following_geometry.py)，新增 `targetFollowing` 任务。它是参考项目自定义任务编码，需要上游行为树同步支持。当前接入 **Kev** 后端；Jev、Julia 会在任务接纳前返回校验失败，保留正在执行的任务。

控制目标是两艇中心距离处于 `[minFollowingDistanceM, maxFollowingDistanceM]`。区间中点为软参考，可以在目标侧面或前方跟随；不要求追逐目标后方的固定点。进入区间后继续执行，直到收到停止请求、新任务覆盖，或发生终止错误。

## 报文与使用

任务样例：[target_following.json](../examples/target_following.json)。沿用现有 `0x0660` JSON 分包封装、`0x0060` 态势、`0x0662` 速度航向输出与 `0x0664` 状态输出。将样例作为 `build_json_packet(task, ALGORITHM_TASK_ID)` 的输入即可。原 `trackingMode` 航点任务继续使用原有格式。

- `platCode`：本艇编号，受服务的 `--platform-id` 过滤。
- `targetPlatformId`：被追踪目标编号，必须与本艇不同。
- `minFollowingDistanceM` / `maxFollowingDistanceM`：中心距离下限、上限，单位米；必须满足 `0 < 下限 < 上限`。为兼容当前避碰配置，下限必须大于静态和动态安全半径的较大值，默认约 86.04 米。
- `targetMaxAgeS`：目标本地接收年龄上限，默认 2.5 秒，必须为正数。动态避障自身的新鲜度限制也继续生效。

追踪任务不带 `planWaypointList`、`planWaypointNum`、`currentRoute` 或 `next_waypoint_index`。两艇都必须提供当前经纬度、实际速度和航向；静止目标也需要有效 `targetCourse`。静止本艇缺少航向时可沿用 Kev 的 `--initial-course-deg` 备用方向。外部速度单位继续由 `--speed-unit knots/mps` 管理，候选和模型展示的速度为节，运动计算为米/秒。

```bash
.venv-kev/bin/python server/local_models/serve_kev.py --device cuda
python3 server/decision_service.py --backend kev --platform-id 1010 --multicast-interface 本机网卡IPv4
```

代码修改后需重启报文服务。`0x0666` 继续使用同一 `simId/id` 停止任务；持续下发零速，实际本艇速度进入停车阈值后才报告 `status=2`。停止不依赖目标继续更新。新任务覆盖、同 ID 替换及滞后结果撤权沿用公共任务控制流程。

## 决策与状态

每轮使用新鲜目标快照，按本地接收年龄和目标当前速度估算当前位置。对目标作匀速预测，对本艇使用统一加减速度、转向角速度和角加速度预测，在同一时刻比较两艇位置；动作区间的最小距离还检查相邻采样间的相对扫掠。模型看到距离区间、当前距离、距离变化率、动作期间最小／最大／结束距离、结束距离变化率、径向制动余量及 10 秒保持指令预览。负距离变化率表示正在接近。预览至少覆盖一个控制周期，不代表承诺执行的未来航迹。

Kev 从通过静态和动态安全检查的候选中选动作。距离恢复阶段沿用当前项目约束候选的做法：过远且存在安全闭合动作时排除同速空转；过近且存在安全拉开动作时优先恢复间距。朝向背离目标时允许先作有用的转向，即使短期距离增加。目标静止且本艇已在区间内时，如零速通过避碰检查，则进入零速保持，避免缓慢逼近穿越下限。由模型在剩余候选中选择具体速度与航向。保留五档常规速度，补充零速、2.5 节、5 节及目标实际速度以支持低速匹配；航向受配置的每周期转向范围限制，补充可达的视线方位和目标航向。发送前（含旧指令续发）再次检查目标新鲜度和最新静动态态势。静态目标实际距离日志继续使用固定坐标系记录。

`0x0664.params` 提供：

| 字段 | 含义 |
| --- | --- |
| `targetPlatformId` | 追踪目标 |
| `targetAvailable` | 当前目标数据是否可用 |
| `targetDistanceM` | 估算中心距离，米；目标不可用时为 null |
| `rangeRateMps` | 径向距离变化率，米/秒 |
| `followingStage` | `waiting_target` / `too_close` / `in_band` / `too_far` |
| `minFollowingDistanceM` / `maxFollowingDistanceM` | 本次任务的距离区间 |
| `phase` / `consecutiveErrors` | 原有任务阶段与连续错误计数 |

不返回虚构的航点数量、航段索引、完成百分比或剩余时间。目标缺失、过期或运动字段无效时，立即发零速，清空有效追踪指令并作可恢复错误计数；恢复前的在途结果会被丢弃。默认连续 3 个控制周期失败后，以 `FOLLOWING_TARGET_UNAVAILABLE` 报告 `status=3`。目标恢复且未达到上限时重新决策。

无共同安全候选时返回零速等待，决策记录标记 `following_safety_hold`。这是临时控制抑制，不能保证入侵目标会停下。追踪功能复用局部运动与避碰检查，尚未接入航点任务的全局静态绕障路径及回线引导；复杂障碍场景可能停车等待。预测未包含目标未来转弯、完整船体水动力或侧滑模型，需继续做仿真与现场参数验证。

## 初步验证

```bash
PYTHONPATH=jev_api:. python3 -m unittest server.tests.test_target_following -v
PYTHONPATH=jev_api:. python3 -m unittest discover -s server/tests -v
PYTHONPATH=jev_api:. python3 -m server.experiments.target_following --mock --steps 60 --output /tmp/following-mock.json
# 已启动真实 Kev 时，去掉 --mock；只调用模型，不向现场发送控制报文。
PYTHONPATH=jev_api:. python3 -m server.experiments.target_following --steps 30 --output /tmp/following-kev.json
```

离线脚本独立用 0.02 秒中点积分更新两艇运动，逐采样统计最小距离；三个场景分别覆盖超距追赶、目标转向和目标停车。`--mock` 仅替代模型选择，不能用来评价真实模型效果。UDP 集成测试使用回环地址、临时端口及模拟模型，验证完整任务／态势／控制／停止／状态链路。
