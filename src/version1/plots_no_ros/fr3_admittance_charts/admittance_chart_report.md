# FR3 Admittance Test Chart Report

Generated from `admittance_task_soft_log.csv` and `admittance_task_hard_log.csv`.

## Key result

- Soft max interaction-phase admittance offset: 15.00 mm
- Hard max interaction-phase admittance offset: 7.39 mm
- Soft median interaction-phase admittance offset: 4.26 mm
- Hard median interaction-phase admittance offset: 1.40 mm
- Soft median interaction-phase external force: 1.56 N
- Hard median interaction-phase external force: 1.90 N

Interpretation: admittance control is demonstrated by the fact that measured external force is converted into a commanded Cartesian offset `adm_x, adm_y, adm_z`, so the commanded pose `x_cmd,y_cmd,z_cmd` deviates from the nominal task pose. The soft setting should generally allow a larger offset than the hard setting for comparable force, but the exact result depends on how similarly the robot was pushed during the two runs.

## Chart files

1. `01_admittance_offset_norm_soft_vs_hard.png`
2. `02_external_force_norm_soft_vs_hard.png`
3. `03_force_vs_admittance_offset_interaction.png`
4. `04_nominal_vs_commanded_x_soft_vs_hard.png`
5. `05_actual_vs_command_tracking_error.png`
6. `06_admittance_velocity_norm_soft_vs_hard.png`
7. `07_soft_admittance_offset_components.png`
8. `08_hard_admittance_offset_components.png`
9. `09_soft_force_components.png`
10. `10_hard_force_components.png`

## Conclusion
Άρα φαίνεται σωστά ότι στο soft admittance η εξωτερική δύναμη παράγει μεγαλύτερη μετατόπιση εντολής, ενώ στο hard admittance η μετατόπιση είναι μικρότερη.

Τα πιο σημαντικά διάγραμματα για απόδειξη είναι:

03_force_vs_admittance_offset_interaction.png

01_admittance_offset_norm_soft_vs_hard.png
04_nominal_vs_commanded_x_soft_vs_hard.png

Για την αναφορά, μπορώ να γράψω ότι το admittance control επιβεβαιώνεται επειδή η μετρούμενη εξωτερική δύναμη Fx,Fy,Fz μετατρέπεται σε admittance offset adm_x, adm_y, adm_z, το οποίο μεταβάλλει το commanded pose x_cmd,y_cmd,z_cmd σε σχέση με το nominal task pose.
