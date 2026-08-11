# 飞行器制导与控制课程项目

[English](README.md)

这是一个基于 MATLAB/Simulink 的飞行器制导与控制课程项目，围绕航天器姿态控制、飞行器纵向稳定控制以及多种制导律的建模、仿真与结果分析展开。

## 项目亮点

- 四反作用飞轮构型下的航天器三轴姿态稳定控制
- PID 姿态控制、执行机构分配、饱和处理与动态响应分析
- 飞行器纵向俯仰稳定回路设计与性能评估
- 三点法、追踪法和比例导引法的对比仿真
- 同时提供 MATLAB 与 Simulink 模型、结果图、CSV 数据、实验报告和可选的 Tacview 可视化

## 仿真结果

### 航天器三轴姿态响应

在闭环控制器作用下，三个姿态角由初始偏差逐渐收敛。

![三轴姿态角响应](1/%E5%AE%9E%E9%AA%8C1/%E4%B8%89%E8%BD%B4%E5%A7%BF%E6%80%81%E8%A7%92%E5%93%8D%E5%BA%94.png)

### 不同制导律的轨迹对比

在相同初始条件下，对三种制导策略的轨迹形态和收敛特性进行比较。

![三种制导律轨迹对比](2/task2/exp2_task2_outputs_matlab/%E5%BC%B9%E9%81%93%E5%AF%B9%E6%AF%94.png)

### 飞行器与目标的相对距离

![相对距离随时间变化](2/task2/exp2_task2_outputs_matlab/%E5%BC%B9%E7%9B%AE%E8%B7%9D%E7%A6%BB.png)

## 目录结构

```text
.
├── 1/实验1/                 # 航天器姿态控制实验
│   ├── satelliteACModel.slx
│   ├── satelliteACSetup.m
│   ├── satelliteACPlot.m
│   └── tacview/             # 可选的可视化资源
├── 2/task1/                 # 飞行器俯仰稳定控制实验
│   ├── task1.slx
│   └── task1_run.m
├── 2/task2/                 # 制导律对比实验
│   ├── exp3_task.m
│   ├── exp2_task2_*_model.slx
│   └── exp2_task2_outputs_*/
├── 实验报告.pdf              # 课程综合报告
└── 实验报告.docx             # 可编辑报告源文件
```

## 环境要求

- MATLAB
- Simulink
- Control System Toolbox（用于俯仰响应性能指标计算）
- Python 3 和 Tacview 为可选项，仅用于扩展可视化流程

本项目源于课程实验，模型文件能否直接打开可能受 MATLAB/Simulink 版本影响。

## 快速开始

### 1. 航天器姿态控制

在 MATLAB 中进入 `1/实验1`，运行：

```matlab
satelliteACSetup
out = sim('satelliteACModel');
satelliteACPlot(true)
```

### 2. 飞行器俯仰稳定控制

进入 `2/task1`，运行：

```matlab
task1_run
```

### 3. 制导律对比

进入 `2/task2`，运行：

```matlab
results = exp3_task;
```

生成的图像和数据位于 `exp2_task2_outputs_matlab/`。

## 项目文档

完整综合报告见 [`实验报告.pdf`](%E5%AE%9E%E9%AA%8C%E6%8A%A5%E5%91%8A.pdf)，各实验目录中也保留了相应的独立报告。

## 说明

- `.gitignore` 已排除 Simulink 缓存和 IDE 配置等生成内容。
- CSV、图片、视频和 Tacview 文件作为可复现实验结果保留。
- 本仓库用于课程学习、数值仿真和控制算法研究。

## 许可证

本项目目前未声明开源许可证。除非仓库所有者另行说明，代码和报告仅供浏览与学习参考。
