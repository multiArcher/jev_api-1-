# 动态回放

直接读取已保存的真实Kev测试轨迹生成，没有重新运行模型，也没有修改测试结果。

| 场景 | 动图 | 帧数 | 播放时长 |
| --- | --- | ---: | ---: |
| 直线 | [replay.gif](straight/replay.gif) | 61 | 约12秒 |
| S形 | [replay.gif](s_curve/replay.gif) | 121 | 约24秒 |
| 交叉折返、变速 | [replay.gif](crossing_variable/replay.gif) | 211 | 约42秒 |

动图每帧对应一个已记录的2秒控制采样，5帧/秒播放，约10倍速。蓝色为本艇、橙色为目标；三角标记方向来自记录航向。目标周围虚线圈100米、实线圈150米，本艇位于两圈之间时满足距离范围。两艇连线在区间内为绿色，超出区间为红色。深色线显示最近40秒运动，浅色线显示此前经过的路线，下方距离曲线与当前时间同步。

交互回放可在本次对话中播放／暂停、拖动时间、切换三种路线及跟随／全景视角，支持2／5／10／20倍速。交互播放使用相邻已记录位置的线性插值，航向按最短角插值，方便观察运动；不是新增的动力学积分数据。

重新生成动图：

```bash
PYTHONPATH=jev_api:. .venv/bin/python -m server.experiments.animate_following_field
# 只生成某个场景，或修改GIF帧率：
PYTHONPATH=jev_api:. .venv/bin/python -m server.experiments.animate_following_field --scenario crossing_variable --fps 5
```

GIF编码、三组帧数、首／中／末帧和交互操作已检查；浏览器验证覆盖拖动、路线切换、播放／暂停与窄屏布局，未发现运行时错误。原始实验的地图与运动近似限制继续适用，见 [测试报告](REPORT.md)。
