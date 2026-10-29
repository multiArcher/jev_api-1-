# FollowMode / AccompanyMode 对接

按用户提供的《对接单艇决策服务(2).md》§5.2、§5.3 接入，默认 Kev 后端支持。旧 `targetFollowing` 接口继续可用。本文描述代码行为，本轮按用户要求未运行本地测试或仿真，尚需联调确认。

## 接收与执行

沿用 `0x0660` 调用、`0x0666` 停止、`0x0664` 状态和 `0x0662` 航速航向下发。`simId` / `id` / `code` 原样关联并返回；状态点播回复任务发送地址与源端口。请求保留既有最多10片重组机制。任务平台必须匹配服务 `--platform-id`（若设置），目标按 `targetPlatCode` 从态势取得，不接受以瞬时经纬度代替目标平台号。

- `followMode`：`platCode`、`targetPlatCode`、`followDistanceMinM`、`followDistanceMaxM`、`maxSpeed`。允许 `0 ≤ min ≤ max` 且 `max > 0`；默认以区间中点为目标沿已记录的目标轨迹尾随。历史不足时向目标初始轨迹方向后方延伸；未把未来转向当成已知。
- `accompanyMode`：`platCode`、`targetPlatCode`、`offsetAlongMinM`、`offsetAlongMaxM`、`offsetAcrossMinM`、`offsetAcrossMaxM`、`syncHeading`、`maxSpeed`。纵横均闭区间且 `min ≤ max`。纵向前正后负，横向右舷正左舷负。在指定位置带内保位，`syncHeading=1` 时连续三个决策周期到位后，优先在保持位置带的前提下对齐目标航向；`0` 不强制同步。

新接口平台编号校验为 UINT16。`maxSpeed=0`（省略时也按0）使用本艇配置上限，仍不超过50节；非零值在候选生成、应急替代动作和最终下发三处限幅。航速沿用服务 `--speed-unit`，默认节；若配置 mps，请求、态势和状态航速均按 m/s，内部统一换算。

目标本身与其他平台一样参与静态/动态安全检查及实际碰撞任务终止。跟随下限小于避碰距离不会直接拒绝请求，而是选择请求范围内的安全参考；跟随上限或整个伴随区域都落在避碰范围内时，直接失败并说明不可达。实际安全候选仍需考虑动态不确定性、控制延迟和制动尾段。

## 行为树状态

`status=1` 持续执行；跟随在距、伴随到位均不自动完成。`0x0666` 停止后持续发送零速，态势确认实际停车再回 `status=2`。目标缺失或过期立即抑制旧动作并暂发零速，沿用默认连续3次失败后 `status=3`；尚未达到失败上限时目标恢复可重新决策。碰撞立即 `status=3`。无位置恢复进展持续60秒时以 `TARGET_TASK_UNREACHABLE` 失败；可选扩展参数 `unreachableTimeoutS` 调整该阈值。可选 `targetMaxAgeS` 默认2.5秒。

`0x0664.params` 按任务分别提供以下明细，不混入航路点进度：

| 任务 | 业务字段 |
| --- | --- |
| followMode | `targetPlatCode`、`relativeDistanceM`、`relativeBearingDeg`、`inRange`、`distanceOutM`、`targetSpeed`、`targetCourse` |
| accompanyMode | `targetPlatCode`、`currentAlongM`、`currentAcrossM`、`alongOutM`、`acrossOutM`、`inPosition`、`targetSpeed`、`targetCourse` |

`relativeBearingDeg` 为本艇相对目标艏向的方位，前0°、右90°、后180°、左270°。越界量均在范围内为0、低于下限为负、高于上限为正；`inRange`/`inPosition` 为整数0/1。目标不可用时测量值为null、标志为0，并提供 `targetAvailable=false`。另保留 `phase`、停车确认和错误诊断等服务状态字段。

## 请求示例

- `server/examples/follow_mode.json`：文档180～220米跟随、最大18节。
- `server/examples/accompany_mode.json`：文档后方140～160米、右舷70～90米伴随，航向同步，最大18节。

重启决策服务后生效，Kev模型服务无需重启。没有运行发送命令向现场下发这些示例。
