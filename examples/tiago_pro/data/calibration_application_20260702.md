# Offsets à ajouter — calibration du 2026-07-02

Source : `data/calibration_results_20260702_0756.yaml` (94 échantillons, RMSE 6.46mm).
Ces valeurs sont des deltas à AJOUTER aux origines de joint existantes (x,y,z en m, roll/pitch/yaw
en rad) — pas encore décidé où/comment les appliquer (problème bras gauche/droit partagé en cours
de discussion).

Non inclus : `base_*_torso` (6 params, mélange mocap/torse indissociable) et `pEEx_1`/`pEEy_1`
(fixés à 0 par construction).

| Joint | Axe | Delta |
|---|---|---|
| arm_right_1 | phix (roll) | -0.016283497 rad |
| arm_right_1 | phiy (pitch) | +0.013443139 rad |
| arm_right_1 | phiz (yaw) | +0.000000011 rad |
| arm_right_2 | px (x) | +0.019881403 m |
| arm_right_2 | pz (z) | +0.000000026 m |
| arm_right_2 | phix (roll) | -0.009890504 rad |
| arm_right_2 | phiy (pitch) | -0.019360118 rad |
| arm_right_2 | phiz (yaw) | -0.042014925 rad |
| arm_right_3 | px (x) | -0.004109735 m |
| arm_right_3 | pz (z) | -0.020218086 m |
| arm_right_3 | phix (roll) | +0.011929481 rad |
| arm_right_3 | phiy (pitch) | -0.006161320 rad |
| arm_right_3 | phiz (yaw) | -0.018246001 rad |
| arm_right_4 | px (x) | +0.000076321 m |
| arm_right_4 | pz (z) | +0.000328900 m |
| arm_right_4 | phix (roll) | +0.003164674 rad |
| arm_right_4 | phiy (pitch) | +0.034311886 rad |
| arm_right_4 | phiz (yaw) | +0.009909231 rad |
| arm_right_5 | px (x) | -0.006723301 m |
| arm_right_5 | phix (roll) | +0.006465796 rad |
| arm_right_5 | phiy (pitch) | +0.012907285 rad |
| arm_right_5 | phiz (yaw) | +0.041627699 rad |
| arm_right_6 | px (x) | +0.000259036 m |
| arm_right_6 | py (y) | +0.001029058 m |
| arm_right_6 | phix (roll) | +0.000689383 rad |
| arm_right_6 | phiy (pitch) | -0.006097649 rad |
| arm_right_6 | phiz (yaw) | -0.013187732 rad |
| arm_right_7 | px (x) | +0.004952491 m |
| arm_right_7 | py (y) | +0.000000048 m |
| arm_right_7 | pz (z) | +0.000944093 m |
| gripper_right_tool_mount | px (x) | -0.002998004 m |
| gripper_right_tool_mount | py (y) | +0.001139139 m |
| gripper_right_tool_mount | pEEz_1 (tip, hors-URDF) | -0.005066 m |

Axes non listés pour un joint = pas de correction identifiée sur cet axe (éliminé par Figaroh
comme non-identifiable, laisser inchangé).
