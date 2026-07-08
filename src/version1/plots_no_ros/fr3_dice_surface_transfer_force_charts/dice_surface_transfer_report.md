# FR3 Dice Surface Transfer Force-Control Report

Generated from `dice_surface_transfer_force_log.csv`.

## Key metrics

- Duration: 27.00 s
- Start x: 0.4490 m
- Final x: 0.3562 m
- Final desired x: 0.3500 m
- Final x error: 6.17 mm
- Max XY tracking error: 10.15 mm
- Median XY error during transfer/release: 7.57 mm
- Median Fz contact during transfer/release: 1.38 N
- Median Fz target during transfer/release: 0.80 N
- Mean absolute force tracking error during transfer/release: 0.76 N
- Minimum z: 0.0393 m
- z safety minimum: 0.0295 m

## Interpretation

The log shows that the task sequence completed and the robot performed surface transfer while applying z-axis force. The actual x position moved from about 0.449 m to about 0.356 m. The final target was about 0.350 m, so the final x error was about 6.2 mm.

The normal force was present during the transfer, but the median measured contact force was higher than the target. This suggests that the normal-force controller still needs tuning if the goal is tighter force tracking. Lower normal force, slower transfer, and softer z-force gains are reasonable next adjustments.

## Chart files

1. `01_actual_vs_desired_x.png`
2. `02_actual_vs_desired_y.png`
3. `03_z_position_and_safety.png`
4. `04_xy_tracking_error.png`
5. `05_fz_contact_vs_target.png`
6. `06_fz_command.png`
7. `07_xy_path_actual_vs_desired.png`
8. `08_phase_timeline.png`
9. `09_external_joint_torque_norm.png`
10. `10_commanded_joint_torque_norm.png`

Άρα η μεταφορά έγινε σωστά, αλλά το robot σταμάτησε περίπου 6.2 mm πριν το target και η κάθετη δύναμη ήταν κατά μέσο όρο μεγαλύτερη από το target. Τα πιο χρήσιμα διαγράμματα για την αναφορά είναι:

05_fz_contact_vs_target.png
04_xy_tracking_error.png
07_xy_path_actual_vs_desired.png
01_actual_vs_desired_x.png
