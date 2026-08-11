# 改进版 Tacview 生成脚本

## 主要改动

| 改进项 | 实现方式 |
|---|---|
| 用 Simulink 的 attitudeLog 生成 ACMI | 读取 `exp1SimulinkForTacview.mat` 或 `exp1SimulinkForTacview.csv` |
| 把 PD 改成 PID + 抗积分饱和 | 不再用 Python 重新跑 PD；直接使用 Simulink 中 PID + anti-windup 的输出轨迹 |
| 仿真时长改为 800 或 2000 s | 自动使用 Simulink 导出的时间轴，Simulink 跑多久 Tacview 就生成多久 |
| 增加收敛/饱和 Bookmark | 自动增加 ±2°、±1° 收敛时间、角速度收敛、飞轮饱和相关 Bookmark |
| 改卫星模型外观 | 默认 `Type=Space+Satellite`，如果 Tacview 不显示，可在脚本顶部改为 `Misc+Container` 或 `Air+FixedWing` |

## 使用方式

把以下文件放在同一个文件夹：

- `build_acmi_exp1_simulink_pid.py`
- `exp1SimulinkForTacview.mat`

然后在该文件夹打开命令行运行：

```bash
python build_acmi_exp1_simulink_pid.py
```

生成：

- `exp1_attitude_simulink_pid.acmi`
- `exp1_acmi_summary.txt`

将 `.acmi` 文件拖入 Tacview 播放。
