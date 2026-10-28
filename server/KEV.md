# Kev-4B 本地后端

静态避障由 `policies/static_obstacles.py` 实现：80米障碍物禁入半径叠加艇体外接圆，默认中心间距约86.04米。内部可见路径绕行、航点安全到达区域、整段轨迹及制动路径检查、发送前最新态势复核的参数和限制见 [服务说明](README.md)。无相关目标时保留原模型输入；有相关目标时增加引导和候选净空等字段，`turn_deg` 继续放在候选末尾。动态避障已通过独立适配器接入 Kev，运动目标与静态目标分别预测、检查；不改变静态引导与近处半速规则，详见 [动态避障说明](navigation/dynamic_avoidance/README.md)。

绕行状态下，模型的 `target_xy_m` 是内部引导点，`arrival_radius_m=10` 是该引导点的跟踪容差；`static_avoidance.waypoint_xy_m` 和 `waypoint_arrival_radius_m=100` 保留原航点区域。外部任务、控制会话的100米到达判定和任务航点计数不变，临时引导点不计入完成数量。

静态绕行采用“直航接近，再绕开”：远处障碍物先使用原目标方向上的内部接近点，避免直接指向远处绕行顶点；接近段不会被可见路径快捷跳过。接近距离按速度、制动与转向能力计算，保留安全余量。实际接近并进入绕行时，速度上限为正常速度阶段上限的一半，例如50节降至25节；远处直航接近时保持原速，绕过后恢复正常规划。半速上限不根据当前实际速度反复减半，也不因绕行偏离原航点方向再额外限速；上限内候选若仍不安全，继续降至下一可行档。航点限速和终点停车等约束继续生效。测试入口为 `server/experiments/static_avoidance.py`。

动态避障分为60秒预警和10秒动作检查。预警只保存在日志与决策记录，不影响模型候选和发送；近程检查才触发避让，发送前动态守卫也使用同一动作窗口。必要的速度响应、转向过程及制动尾段仍完整保留，不能把10秒理解为固定的碰撞前触发时刻。配置入口为 `AvoidanceConfig(warning_horizon_s=60., action_horizon_s=10.)`，静态规则不受这两个参数影响。

实际静态绕行或动态避让后，默认先向前平滑回到固定的原航段，再沿线前往下一航点；首段起点是任务开始时本艇位置，后续使用上一任务航点。回线引导由独立的 `navigation/route_rejoin.py` 管理，静态绕行仍优先，回线候选继续分别通过两类安全检查。横向10米、航向5度内连续3轮视为回线完成，之后保持沿线引导。默认参数、独立配置和闭环测试见 [回线说明](navigation/ROUTE_REJOIN.md)。

`decision_service.py` 默认使用 Kev-4B，可用 `--backend kev/jev/julia` 切换。Kev速度档位为 **10、20、30、40、50节**，由 `policies/kev_policy.py` 的 `KEV_SPEEDS_KNOTS` 定义；按速度阶段开放相应档位，每档配16个局部绝对航向（含直行），单次模型选择。直线巡航只提供最高档，防止持续选择低速。`--max-speed-knots` 可限制可用速度档位，低于10节会报错。默认转向上限45°/秒，1秒周期的航向范围为当前方向±45°。下文单速15节、5°/秒等测试数据属于历史配置。

`routeRadius` 是航路点转弯半径，当前不参与计算。所有航点的到达判定、候选预测和仿真绘图统一使用 **100米** 到达半径，后续按无人艇数据设置。提示词保留目标、运动状态、预测距离和最近决策，任务说明不再重复航速，移除重复的预测说明；方向约定和选择要求使用短句。

决策内部统一以首个航点为固定原点，横轴向东、纵轴向北，单位米。模型状态中的 `boat_xy_m`、`target_xy_m` 是固定坐标，`target_offset_m` 是目标相对本艇的位移；经纬度只用于接口适配，不截断、不修改外部报文。控制会话与候选预测共用该投影，到达半径仍为100米。

速度阶段由 `policies/speed_profile.py` 管理。下一航点的固定坐标、下一段方位和转角通过短数值字段传给Kev，当前航点仍必须按序访问。`next_turn_deg` 使用入航段与出航段的最小夹角，用于提前减速规划；首个目标缺少上一航点时以本艇到目标的方向作为入航方向。当前实测航向与目标方向的误差继续用于实际转向和进展监督。实测航向与下一段的夹角仅记录在本地 `turn_diagnostics.observed_next_turn_deg` 中，不重复传入模型。远距离且目标航向误差不超过15度时只开放最高速度档（默认50节）。接近航点时，转角≤15°、≤45°、≤90°、≤135°、更大转角的初始速度上限分别为50、40、30、20、10节，并叠加航点限速。终点按10节准备进圈后停车；大角度航向修正也会限制速度。仍在已访问航点圈内时继续保持其限速，出圈且对齐后恢复满速。

提前减速距离包含100米到达半径、配置的速度过渡距离、两个控制周期的反应距离，以及按准备速度/转向角速度计算的转弯弧长。规则基于当前近似仿真，须随无人艇动力数据校准。限速低于最小10节档位时通过 `below_minimum_speed_limit` 明示；不把该情况当作满足限速。

进展监督由 `policies/progress.py` 管理，每个任务线程独立。默认窗口5次决策观测、距离改善阈值1米、航向改善阈值2度。窗口内持续远离或大航向误差下无进展且航向未改善时，切入恢复模式，仅提供 **10节 × 16航向＝16候选**。恢复时有方向或距离改善就清零失败计数；方向误差不超过30度且距离减小后退出恢复。连续20次恢复观测仍无改善则抛出不可恢复错误，控制流程下发零速并报告 `NAVIGATION_NO_PROGRESS`。监督参数通过 `ProgressConfig` 管理；阈值用于当前近似仿真，后续按实艇数据调整。换航点会清空旧航点决策历史，新任务使用独立监督状态。

后续导航回归优先运行用户提供的两条实际任务：`.venv/bin/python server/experiments/field_tasks.py`，默认同时执行12航点任务一和45航点任务二。初始位置、速度单位、结果指标及单任务入口见 [两条实际任务测试](experiments/tasks/README.md)。

## 结构

- `policies/kev_policy.py`：航点状态、候选运动预测事实、最近四步决策、模型请求与动作解码。分别通过静态和动态安全检查后交给模型选择，动态风险需要时扩展原可达方向和低速候选，仍遵守静态速度上限。
- `navigation/dynamic_avoidance/`：独立数据类型、预测、规划及 Kev 适配；只对运动或不完整目标做动态检查，保留独立调用和离线验证。
- `navigation/route_rejoin.py`：固定任务航段、绕障后向前汇入和持续跟线，不包含障碍物预测或控制发送。
- `policies/factory.py`：创建后端，Kev的`--kev-endpoint`与Jev地址独立。
- `control_runtime.py`：独立推理工作线程、周期控制发送、连续错误计数、限时续发和终点停车确认；Kev与Jev共用。
- 同艇每个有效新任务都会接管旧任务，包括相同ID/参数。旧任务的控制、停车与状态输出被撤销，后台请求的迟到结果不会下发；Kev的请求入口允许不同任务线程独立发起HTTP请求，不被旧请求的客户端锁阻塞。任务接管规则见 [README.md](README.md)。
- `local_models/setup_kev.py`：下载固定运行时、LoRA适配器、决策头和基座，并生成可离线加载的部署副本。
- `local_models/serve_kev.py`：用官方Kev运行时启动本机HTTP推理服务，模型常驻显存。
- `experiments/simulate.py`：复用现有任务/态势/控制报文及地图，新增Kev选择，记录有效速度、候选、模型概率与耗时。

官方来源：https://huggingface.co/jaredpalmer/kev-4b ，https://github.com/jaredpalmer/kev 。这是Qwen3.5-4B-Base基座上的LoRA加决策头，不是通过生成文本选择动作。

固定版本：

- 官方代码 `1f04cfa28e6a708485b28a9585010b9e436b776e`。
- 适配器 `6cfce5c2fa4b4bd64026336ab649c5ca78857d52`。
- 基座 `1001bb4d826a52d1f399e183466143f4da7b741b`，与发布决策头的base_revision一致。

## 两个独立环境

项目 `.venv` 保留原Julia依赖并运行UDP服务/绘图模拟器。新 `.venv-kev` 只运行Kev官方服务，因为两者需要不同版本的PyTorch和Transformers。不要把Kev依赖安装到原 `.venv`。

常规有venv/pip的机器上安装：

```bash
python3 -m venv .venv-kev
.venv-kev/bin/python -m pip install huggingface_hub
.venv-kev/bin/python server/local_models/setup_kev.py
```

本机原系统没有ensurepip，已在独立环境内初始化pip，未改系统包。

模型目录：`models/kev-runtime`（官方代码）、`models/kev-4b`（原发布适配器/头）、`models/kev-4b-base`（基座）、`models/kev-4b-local`（离线部署副本）。原始发布文件保持原值；部署副本只把head元数据中的base指向本地目录，数值权重不变，版本和校验值存于deployment.json。迁移目录后重新执行 `setup_kev.py --prepare-only` 更新绝对基座路径。

## 启动与测试

终端一，保持运行：

```bash
.venv-kev/bin/python server/local_models/serve_kev.py --device cuda
```

默认只监听 `127.0.0.1:18028`，HTTP接口为 `/v1/systemone`。启动脚本设置HF离线模式，关闭可选融合/图编译，使用官方bf16 CUDA标准路径；未安装额外可选内核时速度可能低于官方优化后的基准。

终端二，在1440×1440米地图测试：

```bash
.venv/bin/python server/experiments/simulate.py \
  --backend kev --scenario crossing_km --steps 2000 --dt 1 \
  --max-turn-rate-deg-s 5 --initial-course-deg 90 \
  --output server/experiments/results/kev-crossing-km
```

模拟器的 `--device` 和 `--model-path` 用于进程内模型；Kev的实际设备由终端一的 `--device` 决定，权重路径由离线服务决定。更换服务地址时，在模拟器或UDP服务上设置 `--kev-endpoint http://127.0.0.1:新端口/v1/systemone`；请求超时为 `--kev-timeout`，默认120秒。

当前任务列表只包含待访问航点，起点取态势位置；`currentRoute=0` 表示尚未完成任何点。报文服务在最后一个航点到达后主动发送0速并根据新鲜态势确认停车，再返回完成状态。本地仿真同步使用停车完成判据（`completion_mode=final_parking`），旧的“到点立即结束”测试仅保留为历史结果。

输出目录包含地图轨迹、速度/耗时三联图、逐步完整输入与模型响应JSONL、汇总与地图快照。实际速度候选读取当前 `KEV_SPEEDS_KNOTS` 常量；历史15节测试不代表后来修改的速度配置。加速仿真推理时暂停世界，不代表实艇实时性。

真实localhost UDP测试，仍需先启动Kev服务：

```bash
.venv/bin/python server/experiments/simulate.py \
  --backend kev --transport udp --scenario crossing_km \
  --udp-seconds 15 --output server/experiments/results/kev-udp
```

实际运行报文服务：

```bash
.venv/bin/python -m server.decision_service --backend kev \
  --kev-endpoint http://127.0.0.1:18028/v1/systemone \
  --host 127.0.0.1 --port 18024 --situation-port 18025 \
  --action-host 127.0.0.1 --action-port 18124 --platform-id 1010 \
  --max-turn-rate-deg-s 5 --initial-course-deg 90
```

以上本地测试地址与旧文档一致。新增加的是Kev本机推理口18028；原现场任务 `0.0.0.0:18014`、态势 `239.255.0.10:18010` 均未修改；原动力目标源码 `192.168.3.201:18123` 与环境示例/Compose `192.168.3.126:18123` 的既有差异仍保留。

## 提示词与历史版本的差异

Kev使用独立提示词，明确0/90/180/270度方位、当前目标坐标和方位、每个方向的预测距离/进度及转向量，附最近四步的距离与选择。任务仍按序进入航点圈；未触发避障时沿用原航点导航，实际避障后增加有明确几何定义和容差的原航段回线引导。相比历史Julia同时改变了模型、候选速度/航向及提示组织，结果不能作为仅换模型的受控对照。

## 本轮实测

真实RTX5090测试按序到达10/10航点，1825次决策，1824秒仿真时间，平均完整决策217毫秒、P95约219毫秒。固定15节与16选项逐步核对通过。路径存在绕行；最终按进入终点圈完成，没有停车。真实UDP显式停止流程也通过。25项回归测试及两个环境的依赖检查通过。完整结果见 [KEV_REPORT.md](experiments/results/KEV_REPORT.md)。

服务已在本次会话启动并监听本机18028口；若端口已有服务，无需重复启动。停止服务后可用上文命令重新加载。提示词在 `policies/kev_policy.py` 的 `_request()`，可直接查看 `experiments/examples/kev_request.json`；每步完整输入与响应保存在本轮JSONL。

## 精简提示词与输入精度

当前Kev输入已精简：保留每个候选的速度、航向、目标角误差、预测距离和限速事实；状态提供固定坐标、目标相对位置和简短进展模式。提示词用短句要求按序到达、减速转弯、恢复时优先减小航向误差。

2026-10-26转向回归定位：候选末尾恢复 `turn_deg=abs(signed_turn_deg)`，恢复此前优先减小当前目标航向误差的简短选择说明。24个固定状态的真实Kev回放中，有效转向选择从8/24提高到24/24；字段位置也会影响结果，不能仅因数学上可推导就删除。相同五档速度、100米到达半径和45度/秒约束的闭环到达10/10并停车，但重穿已访问P8圈时出现1次超速，尚未全部通过。完整证据与限制见 [转向回归诊断](experiments/results/kev-turn-input-fix-20261026-162105/REPORT.md)。

`policies/kev_policy.py` 的 `round_model_values()` 在请求出口统一递归四舍五入所有浮点数至两位小数，覆盖状态、速度/航向候选、转向/时间参数及历史。保留数值类型，不为凑显示位数转成字符串；例如22.5表示22.50的同一数值。整数与空值保持类型。内部经纬度转换、运动积分和计时保持原精度；这里限制的是模型可见数据，不把经纬度直接截成0.01度。

精简前初始请求JSON字符数7469，精简后4342，减少约41.9%；这不是token数统计。更新后的输入示例仍在 `experiments/examples/kev_request.json`。历史10/10完整航线结果属于精简前版本，不自动视为精简版本的验证结果；新短测单独保存在 `experiments/results/kev-compact-smoke/`。


## 当前转向约束及注释字段修正

- `--max-turn-rate-deg-s` 当前默认45度/秒，是候选和预测使用的配置值，未实艇标定；可通过参数或 `MAX_TURN_RATE_DEG_S` 覆盖。历史5度/秒测试保留原配置和结果。
- 单周期最大转角 = 角速度上限 × 决策时长。默认1秒为当前方向±45度；0.2秒为±9度。16候选从区间内等距取样，将最接近直行的中间样本替换为直行项（偶数个对称样本不能同时精确包含中心），仍为16项。命令量化到0.01度时向区间内部取整，避免四舍五入后越界。范围不足以表示16个不同两位小数航向时显式报错。
- 提示词已移除 `speed_transition_s` 和 `course_transition_s`。速度响应时间仍用于内部启动/变速预测；Kev转向不再使用“任意角度0.5秒完成”，而是角速度受限的近似运动。预测与仿真使用同一约束；原Jev/Julia默认运动模式不变。
- 你注释的 `total_waypoints` 和数值限速说明没有运行依赖，保持删减。`arrival_radius_m` 和 `command_duration_s` 恢复为短数值字段：前者定义何时进入目标圈，后者定义预测距离对应多久，具有决策含义。历史记录改由内部观测计算距离，不再读取可被注释掉的提示词字段，避免KeyError。
- 报文服务每轮读取最新态势的位置、速度和 `targetCourse`；Kev 在速度为0时也保留态势中的有效航向，0度合法。`--initial-course-deg` 仅在静止且态势未提供航向时作为备用，不覆盖有效态势。模拟器示例90度是其明确设置的向东初始方向；不能根据目标点推测本艇方向。
- 假定无侧滑时以地速航向近似艏向，未建模舵机迟滞、横流和角加速度；这里只是角速度上限模型，不是已辨识的真实船体动力学。
- Kev分支没有角加速度参数或上限：转向角速度直接从0切到±45度/秒，到达指令航向后归零，切换处没有有限的角加速度值。0.5秒是速度过渡时间，不能据此推算Kev的角加速度。本次输入修复未改动该运动模型。

资料依据：

1. [USV Yaw Modeling and Identification](https://arxiv.org/abs/2303.13064)：转向模型参数由试验数据辨识，不能从目标方位推断。
2. [IMO MSC.137(76)](https://wwwcdn.imo.org/localresources/en/KnowledgeCentre/IndexofIMOResolutions/MSCResolutions/MSC.137%2876%29.pdf)：操纵性通过试验/模型验证，适用范围有船长/船型限制，且排除其定义的高速船；不能将战术回转直径5船长要求直接套给本艇或作为最大角速度。

当前有界转向版本应重新验证；此前10/10结果属于旧的全局候选/快速转向仿真，不代表新动力学下也已通过整条航线。


受限转向还补充了 `target_relative_bearing_deg`、候选的有符号转向与 `target_heading_error_deg`，避免目标在侧后方时被“必须立即减小距离”误导。问题优先引导模型减小目标航向误差，对齐后直行；不对候选做排序或规则代选。这些量均在请求出口保留两位小数。

最初5Hz受限候选试验见 `experiments/results/kev-bounded-turn-5hz/`：600步全部16候选均在±1度，实际最大角速度约5度/秒，但初始提示词换向不佳；该试验不是最后提示词的导航成绩。随后侧后方提示探针和1Hz中间试验另存，最终短测目录为 `experiments/results/kev-bounded-turn-final/`。各版本保留，不复用旧10/10结果来描述当前约束版本。

受限转向的参数依据、逐轮实测范围与代码检查记录见 [KEV_TURNING.md](experiments/results/KEV_TURNING.md)。31项测试与真实UDP停止短测通过；当前受限模型的导航仍有不足，旧10/10只作为历史结果保留。
