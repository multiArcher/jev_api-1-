# 独立动态避障

**已接入 Kev，静态与动态分别判断；Jev/Julia 不接入。** 核心模块仍可独立调用，不读实时态势、不发控制指令，不依赖模型服务。调用方为每个任务创建一个 `DynamicAvoidancePlanner`，串行调用，切换任务时创建新实例或执行 `reset()`。Kev 接入集中在本目录的 `kev.py`，动态预测不接收静态目标。

## 使用

从仓库根目录设置 `PYTHONPATH=jev_api:.`。内部速度全部使用米／秒，位置使用固定原点的东／北米坐标；航向从正北顺时针，0°为北、90°为东。

```python
from server.navigation.dynamic_avoidance import (
    Action, DynamicAvoidancePlanner, Target, Vessel,
)

planner = DynamicAvoidancePlanner()
result = planner.plan(
    Vessel(position_m=(0, 0), speed_mps=10, course_deg=0),
    [Target(target_id=20, position_m=(0, 600), speed_mps=10, course_deg=180)],
    Action(speed_mps=10, course_deg=0),
)
print(result.status, result.action, result.assessment)
```

也可以调用 `scene_from_situation(records, origin_lon_lat=(120, 30), speed_unit='knots', own_id=1010)` 转换协议态势，返回 `(Vessel, tuple[Target, ...])`。`speed_unit` 必须显式指定为 `knots` 或 `mps`。输入不会被修改；本艇必须具有有效位置、速度、航向和不超过5秒的接收年龄。相同编号采用最后一条记录，非法编号报错。使用本地 `_received_age_s`，不使用源协议时间戳；未提供年龄按当前快照处理，调用方必须确保这一假设成立。原点必须在同一任务中保持不变。

目标的 `position_m`、`speed_mps`、`course_deg` 可为 `None`，表示未知，不能自行用0填充。静止目标允许省略航向。直接构造对象时非法数值报错；协议转换时目标非法字段转为未知，本艇非法字段报错。模块不维护目标历史，调用方每次须提供完整已知目标快照，不能把暂时未探测到的目标直接从输入删除。

## 结果含义

| 状态 | 含义 |
| --- | --- |
| `clear` | 期望动作通过预测检查，原样返回 |
| `avoiding` | 期望动作存在风险，返回检查通过的替代动作 |
| `blocked` | 没有安全候选，`action=None` |
| `insufficient_data` | 缺失／过期目标的保守可达范围影响规划，无法确认安全，`action=None` |

`assessment.risks` 给出各有效目标的预测最小净空、最近会遇距离和时间（包括制动尾段）；`threat_ids` 为期望动作涉及的危险或不确定目标。避让成功时 `assessment` 对应返回动作；失败时对应期望动作。`uncertain_target_ids` 和 `diagnostics` 说明数据不足原因。没有有效目标时最小净空为 `None`。

失败结果中的 `emergency_action` 是经过评估的应急建议，**不是安全指令**；其完整检查结果在 `emergency_assessment` 中。它按不确定目标数量较少、最小净空较大的顺序选择。模块不会把零速直接当作安全，也不自动决定是否执行不安全建议。

## 默认预测与策略

- Kev 使用 `warning_horizon_s=60` 预警窗口和 `action_horizon_s=10` 动作窗口，积分步长0.2秒、加速1.46m/s²、减速1.75m/s²、最大转向45°/s。`horizon_s=60` 保留为独立预测／独立规划调用的默认长度；Kev 的动作筛选以新的动作窗口为准。输入应使用实际艇的能力配置。
- 本艇复用现有运动预测工具，保持候选指令至预测末尾，再追加具有真实时间长度的制动尾段。主预测长度至少覆盖速度转换和转向时间。
- 目标按恒速、恒航向推进，首先补偿接收年龄；独立调用可传入静止目标，Kev 接入仅传运动或信息不完整的目标，静态目标交给原静态模块。同一时间区间内检查双方相对运动线段，不以空间轨迹相交替代同时会遇。
- 基础安全中心距离为 `80 + hypot(11.6, 3.4)/2 ≈ 86.04m`，已包含本艇尺寸，不再次叠加艇体红圈。另加 `safety_margin_m`，以及 `uncertainty_mps × (接收年龄 + 预测时间)`，默认不确定性增长速度0.5m/s。净空超过0.5米才通过检查。
- 目标数据超过5秒或运动字段缺失时，用最后位置和默认30m/s的未知目标速度上界形成保守可达圆；若已报告速度更大，采用更大的值。没有位置时始终视为相关。只有可达范围完全不影响候选轨迹，才允许继续规划并保留诊断。此上界是环境假设，可按实际目标速度范围调整。
- 危险时以当前航向为中心，默认每15°生成候选，单周期航向命令偏差上限为 `max_turn_rate_deg_s × planning_interval_s`（默认45°）；实际运动依然连续转向。速度候选含期望速度的100%、75%、50%、25%、本艇当前速度与期望速度的较小值及零速，不主动加速到期望速度以上。
- 筛除危险候选后优先右转，其次直行减速，再次左转；同类优先靠近期望航向，再减少速度变化。上一次避让方向在偏差相差不超过15°时优先保持。期望动作重新安全时恢复原动作并清除方向记忆。

这些结果只适用于输入态势、配置的运动模型和预测范围。第一版不判断完整海上会遇规则，不预测目标意图、加速度或转弯，不包含真实侧滑、风流和感知误差模型。独立模块不会绕过现有静态规划或控制守卫。

## 当前接入状态

静态规划、直航接近、转向触发、半速规则和原静态安全检查保持原实现。有效静态目标（速度绝对值不超过0.1节，随静态阈值配置）只进入静态模块；运动及不完整目标进入动态预测。目标重新报告静止或运动时更新分类，不能重复计入两套检查。协议态势存储保留完整已知目标快照及接收年龄；运动目标过期不直接消失，而按保守可达范围判断。

Kev 先生成原静态候选，再逐一做动态检查。全部通过时原候选和提示词保持不变；动态风险影响原期望动作时，允许恢复原转向范围中的其他方向并增加低速／零速候选。新增候选全部重新通过原静态轨迹与制动检查，不把静态目标混入动态60秒预测，也不修改静态引导点。近处静态绕行上限为25节时，动态动作同样不超过25节；只有动态目标时不固定减半，安全时保持原速度。风险解除后恢复原静态候选流程。

60秒窗口现在仅评估原期望动作的远期风险，记录 `Kev dynamic early warning` 和 `last_decision.dynamic_warning`；预警不进入模型提示词，不剔除候选、不暂停航点进展，也不拦截控制指令。10秒窗口用于 `needs_avoidance`、候选筛选和发送前复核，两处采用相同设置，避免规划放行后被长窗口守卫拦停。进入短窗口风险后才调整动作；长窗口预警解除时更新日志。

动作窗口不是硬性的“碰撞前10秒”：主预测至少覆盖完整变速及转向过程，之后还追加完整制动尾段。例如50节保持航速时，10秒主预测之后仍有约14.7秒制动尾段；安全检查可能据此更早调整。窗口参数必须为有限正数，且动作窗口不得大于预警窗口。

候选同时通过有界物理加减速和配置的速度响应预测；按控制报文速度单位的两位小数量化后检查。动态避让优先右转并保持相近的既有避让方向，右侧被静态安全检查否决时可选择左侧。仅在存在动态风险时缩减或扩展原模型选项，最多16项，`turn_deg` 保持末字段。模型可见 `dynamic_avoidance` 为简短状态，完整风险详情保留在 `last_decision`。

通过 `KevPolicy(dynamic_config=AvoidanceConfig(...))` 或工厂参数 `kev_dynamic_config` 配置动态预测；默认几何尺寸继承静态设置，但配置对象独立。动态转向能力和控制周期沿用当前 Kev 参数。无共同安全动作时不请求模型，返回零速等待；等待或主动让行不累计航点无进展失败，态势恢复后继续，新任务重置动态状态。

```python
config = AvoidanceConfig(warning_horizon_s=60., action_horizon_s=10.)
# KevPolicy(dynamic_config=config) 或 create_policy(kev_dynamic_config=config)
```

发送前原 `validate_control` 继续只做静态检查，新增 `validate_dynamic_control` 独立复核最新运动目标。正速指令保留静态守卫；新指令、续发旧指令和零速指令均做动态检查。风险分别记录 `static obstacle control veto`、`dynamic obstacle control veto`；动态拦截后如果零速仍不安全，另记 `dynamic obstacle zero-speed risk`。**零速不是安全保证**，显式停止与任务结束不会根据动态建议自行恢复移动。

## 离线验证

```bash
PYTHONPATH=jev_api:. python3 -m unittest server.tests.test_dynamic_avoidance -v
PYTHONPATH=jev_api:. python3 -m server.experiments.dynamic_avoidance --output /tmp/dynamic-avoidance.json
PYTHONPATH=jev_api:. python3 -m unittest discover -s server/tests -q
PYTHONPATH=jev_api:. python3 -m server.experiments.kev_dynamic_avoidance --mock --output /tmp/kev-separated-mock
# 使用已启动的本机 Kev HTTP 服务，不发送现场控制报文：
PYTHONPATH=jev_api:. python3 -m server.experiments.kev_dynamic_avoidance --scenario mixed_near --output /tmp/kev-separated-real
```

示例验证对遇、交叉、追越、右侧受静态目标阻挡的场景，以及75次重复规划的闭环过程。JSON 输出预测双方轨迹、规划状态、风险信息和闭环采样；使用独立的0.02秒积分器重新计算输出动作及制动尾段的最小净空，断言安全。该密集采样复核用于回归验证，不是连续运动的形式化证明，也不代表实艇验证。

分离接入示例覆盖对遇、交叉、追越和近／远静态障碍混合场景，分别使用物理加减速与配置响应模型，独立积分检查两类实际最小净空，输出双方轨迹和两套发送前检查。推理期间仿真时间暂停，不代表实艇或真实时间调度验证。已运行的报文服务不会自动加载代码变化，需重启报文决策服务；Kev 模型服务无需重启。
