# FR3 Impedance Test Chart Report

Generated charts from `impedance_task_soft_log.csv` and `impedance_task_hard_log.csv`.

## Key result

- Soft max hold-phase position error: 24.50 mm
- Hard max hold-phase position error: 11.20 mm
- Soft median hold-phase position error: 21.38 mm
- Hard median hold-phase position error: 7.41 mm

Interpretation: the soft impedance run shows substantially larger deflection than the hard run, while the measured external force range is comparable. This is the expected behavior of a lower-stiffness impedance controller.

## Chart files

1. `01_position_error_norm_soft_vs_hard.png`
2. `02_external_force_norm_soft_vs_hard.png`
3. `03_force_vs_deflection_hold_phase.png`
4. `04_x_actual_vs_desired_soft_vs_hard.png`
5. `05_soft_force_components.png`
6. `06_hard_force_components.png`
7. `07_commanded_torque_norm_soft_vs_hard.png`
8. `08_external_joint_torque_norm_soft_vs_hard.png`
