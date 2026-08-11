# Guidance and Control Project

[中文说明](README_CN.md)

A MATLAB/Simulink course project for modeling, simulation, and analysis of guidance and control systems for aerospace vehicles. The repository covers spacecraft attitude control, vehicle longitudinal stabilization, and comparative guidance-law simulation.

## Highlights

- Three-axis spacecraft attitude stabilization with a four-reaction-wheel configuration
- PID-based attitude control, actuator allocation, saturation handling, and response analysis
- Longitudinal pitch stabilization for an aerospace vehicle
- Comparison of three-point, pursuit, and proportional-navigation guidance laws
- MATLAB and Simulink implementations with plots, CSV exports, reports, and optional Tacview visualization

## Results

### Spacecraft attitude response

The three attitude angles converge from their initial offsets under the closed-loop controller.

![Three-axis attitude response](1/%E5%AE%9E%E9%AA%8C1/%E4%B8%89%E8%BD%B4%E5%A7%BF%E6%80%81%E8%A7%92%E5%93%8D%E5%BA%94.png)

### Guidance-law trajectory comparison

The same engagement geometry is simulated with three guidance strategies to compare trajectory shape and convergence behavior.

![Guidance-law trajectory comparison](2/task2/exp2_task2_outputs_matlab/%E5%BC%B9%E9%81%93%E5%AF%B9%E6%AF%94.png)

### Relative-distance history

![Relative-distance history](2/task2/exp2_task2_outputs_matlab/%E5%BC%B9%E7%9B%AE%E8%B7%9D%E7%A6%BB.png)

## Repository structure

```text
.
├── 1/实验1/                 # Spacecraft attitude-control experiment
│   ├── satelliteACModel.slx
│   ├── satelliteACSetup.m
│   ├── satelliteACPlot.m
│   └── tacview/             # Optional visualization assets
├── 2/task1/                 # Vehicle pitch-stabilization experiment
│   ├── task1.slx
│   └── task1_run.m
├── 2/task2/                 # Guidance-law comparison
│   ├── exp3_task.m
│   ├── exp2_task2_*_model.slx
│   └── exp2_task2_outputs_*/
├── 实验报告.pdf              # Consolidated course report
└── 实验报告.docx             # Editable report source
```

## Requirements

- MATLAB
- Simulink
- Control System Toolbox (used by the pitch-response performance analysis)
- Python 3 and Tacview are optional and only needed for the visualization workflow

The project was developed as coursework. Exact compatibility can depend on the MATLAB/Simulink release used to open the model files.

## Quick start

### 1. Spacecraft attitude control

Open MATLAB in `1/实验1` and run:

```matlab
satelliteACSetup
out = sim('satelliteACModel');
satelliteACPlot(true)
```

### 2. Vehicle pitch stabilization

Open MATLAB in `2/task1` and run:

```matlab
task1_run
```

### 3. Guidance-law comparison

Open MATLAB in `2/task2` and run:

```matlab
results = exp3_task;
```

The generated figures and data are written to `exp2_task2_outputs_matlab/`.

## Documentation

The repository includes the complete consolidated report as [`实验报告.pdf`](%E5%AE%9E%E9%AA%8C%E6%8A%A5%E5%91%8A.pdf), together with individual experiment reports in their corresponding directories.

## Notes

- Generated Simulink caches are excluded through `.gitignore`.
- Included CSV, image, video, and Tacview files are retained as reproducible experiment outputs.
- This repository is intended for education, simulation, and control-algorithm study.

## License

No open-source license is currently granted. The source and reports are provided for viewing and educational reference unless the repository owner states otherwise.
