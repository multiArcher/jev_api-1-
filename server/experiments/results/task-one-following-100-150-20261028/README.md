# 任务一地图中的分级目标追踪测试

使用任务一原始12个经纬度航点及 `static_situation.json` 中16个已记录静态目标作为地图层。目标初始位置位于第3航点附近，本艇初始距离目标300米，位于允许区间100–150米以外；直线场景另外侧向偏离目标航线75米。目标没有预先向策略提供完整轨迹，每轮仅提供当前目标位置、速度和航向。

| 测试 | 目标运动 | 模拟时长 | 图像与数据 |
| --- | --- | ---: | --- |
| 直线 | 10节，向东匀速直线 | 120秒 | [路线图](straight/routes.png) · [汇总](straight/summary.json) |
| S形 | 10节，航向以80秒周期在50–130度间连续变化，3个周期 | 240秒 | [路线图](s_curve/routes.png) · [汇总](s_curve/summary.json) |
| 交叉折返、变速 | 连续8字交叉路线，速度在6–18节之间连续变化 | 420秒 | [路线图](crossing_variable/routes.png) · [汇总](crossing_variable/summary.json) |

任务距离配置直接读取 `server/examples/target_following.json`，通过 `--task-config` 可指定其他配置；每场景保存当次任务快照，不使用写死的距离范围。

每种测试结束立即生成 `routes.png`，并独立保存任务、原始地图／障碍物、实验配置、逐轮轨迹和统计。橙线代表目标、蓝线代表本艇，灰虚线为任务一航点参考路线，红圈为记录静态目标的有效安全圈；圆点为起点，X为终点。右侧总览显示试验区在任务一地图中的位置，下方显示距离和速度曲线。

## 复现

先启动真实Kev模型服务，然后在项目根目录运行：

```bash
PYTHONPATH=jev_api:. .venv/bin/python -m server.experiments.target_following_field --scenario straight
PYTHONPATH=jev_api:. .venv/bin/python -m server.experiments.target_following_field --scenario s_curve
PYTHONPATH=jev_api:. .venv/bin/python -m server.experiments.target_following_field --scenario crossing_variable
```

`--scenario all` 依次运行三组；每组完成即保存图像。`--output` 指定新结果目录，避免覆盖；`--duration` 和 `--dt` 可以调整时长和控制步长；`--initial-distance-m` 调整区间外起步距离，`--straight-offset-m` 调整直线场景侧向起点偏移。`--mock` 仅用于工具预检，不代表真实模型结果。

## 参数与适用范围

本次使用真实本地Kev HTTP服务，控制动作时长2秒，航速响应0.5秒，最大转向角速度45度/秒；用独立0.02秒中点积分更新本艇轨迹。S形目标连续积分运动，8字目标按弧长及变速函数推进，不瞬移。每个0.02秒采样检查本艇与目标间距，以及两者到全部记录静态目标的安全净距。静态安全圈沿用当前默认约86.04米。

这里的“地图”是原任务经纬度及记录障碍物图层，没有加载海岸线、水深、Unity地形或完整船体动力学。模型推理期间暂停模拟时间，不能据此认定实际控制延迟下的效果。未向现场地址发送控制报文，也未修改或重启已运行的模型服务。


[动图与交互回放说明](REPLAY.md)：[直线动图](straight/replay.gif) · [S形动图](s_curve/replay.gif) · [交叉变速动图](crossing_variable/replay.gif)。
