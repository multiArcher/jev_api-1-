# 两条实际任务测试

后续 Kev 导航回归优先使用这两条任务，入口为 `server/experiments/field_tasks.py`：

- `test_task_one(output, config=None, policy=None)`：任务一，12个航点，基础长航线。
- `test_task_two(output, config=None, policy=None)`：任务二，45个航点，更多方向变化。

JSON保存用户提供的完整原始任务。`routeSpeed`、态势速度和控制输出均按**节**处理；两条任务的航点限速均为约10.289节，不转换成20节。`routeRadius`保留原值但不参与计算；所有到达圈统一100米。

默认起点在首个航点以西50米，静止、航向90度，`currentRoute=0`；由于起点位于100米到达圈内，首个航点会在初始态势中判定到达。可将 `--start-offset-m` 设为大于100米来测试首个航点的进圈过程。默认控制步长1秒、转向上限12度/秒、角加速度上限4度/秒²、最多5000步；每条任务创建独立策略和历史。起点、初始航向和步长可通过参数明确调整。Python接口还可用 `TestConfig(initial_position_xy_m=(x, y))` 指定首航点坐标系中的诊断起点。

先单独启动本地Kev服务，再从项目根目录运行：

```bash
.venv/bin/python server/experiments/field_tasks.py
.venv/bin/python server/experiments/field_tasks.py --task one
.venv/bin/python server/experiments/field_tasks.py --task two
```

默认同时执行两条任务，输出到 `server/experiments/results/field-tasks/task_one/` 和 `task_two/`。每条任务保存原始 `task.json`、逐步完整输入/候选/响应及运动记录 `trajectory.jsonl`、汇总 `summary.json`（含恢复决策次数与停车确认）、轨迹/速度/航向图 `trajectory.png`。没有绘图依赖时可加 `--no-plots`；`--output` 可指定新的结果目录以保留不同版本。

可加 `--checkpoint-every 10`：每完成10个航点自动保存 `checkpoints/waypoints_10.png`、`waypoints_20.png` 等运行图，`checkpoints.json` 记录当时的时间、净空和违规统计。结束时另存 `checkpoints/final.png`，并标明是否通过；中间进度图不代表全程通过。图中橙线突出最近10个航点区间，绿点表示已到达航点，灰点表示尚未访问，红圈为静态目标的安全中心距离范围。`--no-plots` 会同时关闭进度图。

`completed`要求所有航点按序访问，并在下发零速后由态势确认速度不高于0.1节。`passed`还要求没有速度或指令约束违规。正常航行指令不得超过50节，常规航向候选使用当前控制周期的目标偏差区间；动态避障允许更远的持续转弯目标，但实际角速度及角加速度仍受统一物理模型限制。航点限速检查覆盖预测运动路径与当前到达圈的交段，以及尚未离开的已访问圈，记录圈内峰值速度（数值容差0.01节）；违规会记录航点索引和时刻。索引从0开始。

态势报文的航向字段按整数度编码，候选转角范围依据模型实际收到的 `observed_course_deg` 检查。运动积分仍使用未取整的 `actual_course_deg` 并限制实际转向角速度，不能把两种精度混用而误报合法候选超限。二者同时写入轨迹记录，汇总中的 `max_actual_turn_rate_deg_s` 来自实际积分。

当前Kev为10、20、30、40、50节五档速度，按下一航点转角和距离划分速度阶段；对齐的远距离巡航只开放50节，靠近航点、仍在已访问圈内或需要大角度转向时限制速度，恢复模式仅提供10节。两条任务限速约10.289节，较高速度在圈内可能违规，完成路线也不代表限速测试通过。测试会记录选择的速度/航向、固定首航点坐标和进展模式；恢复失败后模拟零速停车并保存失败原因，不代替模型选择方向。尚未完成、模型异常或任何违规时，命令退出码为1，全部通过为0。

仿真采用理想化运动预测，模型推理期间暂停仿真时间；该入口用于策略导航评估。实时控制发送、超时、任务接管等继续由 `server/tests/` 的离线服务测试覆盖，不能用这两条航线替代这些测试。

不调用真实模型的测试：

```bash
python3 -m unittest server.tests.test_field_tasks -v
```
## 静态障碍物回放

`static_situation.json` 来自2026-10-26的实际组播记录，包含1001～1009静止船艇和4001～4007静态障碍物，保留协议字段，速度单位为节。两条原任务不改动，可直接运行：

```bash
.venv/bin/python server/experiments/field_tasks.py --task both --steps 6000 --static-situation server/experiments/tasks/static_situation.json --output server/experiments/results/static-field-tasks
```

仿真按本艇5公里范围提供新探测，策略保留已知静态位置。碰撞统计使用全部已录制障碍物，检查积分路径各线段与约86.04米有效避碰圆的净空；报告完成情况、静态碰撞数、最小净空、航点限速和最终停车。图中的红圈是已包含艇体大小的禁入范围。当前近似运动和推理暂停世界的限制仍然适用，不代表实艇验证。
