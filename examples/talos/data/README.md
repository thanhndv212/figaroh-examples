# TALOS data

- `talos_left_arm_02_10_contact.csv`: the left-arm contact postures used by the
  default calibration (see `../config/talos_unified_config.yaml`).
- `calibration/mocap/`: three Qualisys motion-capture sessions of the torso and
  arms (2022-03-15), below.

## Motion-capture calibration data (`calibration/mocap/`)

Three sessions of 2022-03-15
([#114](https://github.com/thanhndv212/figaroh-examples/issues/114)), extracted
from the original recordings as static postures: one row per plateau with the
joint angles and the mean pose of three tracked bodies. Their own protocol
(`calibration/mocap/protocol.yaml`) freezes roles and hashes. v1 shipped the
files; v2 ([#172](https://github.com/thanhndv212/figaroh-examples/issues/172),
same files and hashes) adds the bodies' attachment links, the measurement and
flip rules and the held-out block of the calibration study
(`examples/talos/mocap_heldout.py`,
[docs/development/talos-mocap.md](../../../docs/development/talos-mocap.md)).
`protocol_1354_kfold.yaml` is the study's row k-fold plan of the training run
(selects the estimation method).

| File | Role | Arm | Run (UTC) | Postures | Plateaus | Lag (s) | sha256 (prefix) |
|---|---|---|---|---|---|---|---|
| `qualisys_2022-03-15-1354_left_static_postures.csv` | training | left | 13:54 | 56 | 65 | -0.08 | `4bfa50c2219acf77` |
| `qualisys_2022-03-15-1418_right_static_postures.csv` | validation | right | 14:18 | 39 | 65 | +0.02 | `8b1d0c5e3b4e5bef` |
| `qualisys_2022-03-15-1439_right_static_postures.csv` | confirmation | right | 14:39 | 23 | 28 | -0.02 | `c6cc469bd21da32e` |

All three runs are usable (the gate: fewer than about 15 kept plateaus would
mark a run not usable and ship nothing for it). Roles: the left run trains and
the first right run validates, as the issue asks, so the two roles span the two
arms. The third run (the bag is named `calib_TEST27_right`) is a second
right-arm run with 23 postures of its own: no posture repeats one of the
validation run within 5 mrad, and its shorter session (490 s) makes it a check rather than a training or validation set, hence `confirmation`.

### Audit

Per session, from the bag (`protocol.yaml` holds the same figures).

| Session | Duration | Joint rate | waist / upper body / end-effector samples | Rates (Hz) | Frozen | Tracking-loss gaps (s lost) | Lag |
|---|---|---|---|---|---|---|---|
| 1354 left | 1115.8 s | 62.5 Hz (70529) | 42061 / 35695 / 807 | 49.1 / 33.5 / 1.0 | 0 / 0 / 0 | 56 (8.2) / 163 (32.7) / 27 (183.2) | -0.08 s |
| 1418 right | 1105.0 s | 62.5 Hz (69884) | 43067 / 31826 / 615 | 49.8 / 33.4 / 1.1 | 0 / 0 / 0 | 79 (11.0) / 255 (49.5) / 39 (393.0) | +0.02 s |
| 1439 right | 489.6 s | 62.5 Hz (30982) | 19787 / 13216 / 343 | 49.9 / 33.3 / 1.2 | 0 / 0 / 0 | 27 (3.6) / 153 (30.4) / 16 (96.7) | -0.02 s |

- **Provenance:** original recordings (not distributed), named and hashed in
  `protocol.yaml`: one ROS bag per session. A bag holds two topics: the
  Qualisys poses with the robot's own link transforms on `/tf_throttle`, and
  the PAL introspection statistics (joint positions among 604 values) on
  `/introspection_data/values_throttle`. The throttled topics are what was
  recorded; the original publishing rates are unknown. PAL Robotics TALOS.
  Recorded at LAAS-CNRS by @thanhndv212; distributed under Apache 2.0, as the
  repository. Sensor hardware is a disclosed limit: the Qualisys system,
  camera count, marker model and calibration quality are unknown, and nothing
  is assumed about them.
- **Bodies and frames:** three rigid bodies, parent frame `Qualisys` (the
  Qualisys world frame, the frame of every shipped pose):
  `waist_frame` (`waist_*`), `upper_body_frame` (`upper_body_*`),
  `endeffector_frame` (`endeffector_*`). Poses are position (m) and unit
  quaternion `x, y, z, w` (scalar part >= 0, the mean of the plateau samples).
  Nothing in the recordings states the links; they are established fit-free
  (v2, `test_attachment_links_from_rotation_angles`): if a body rides on a
  link and the waist on `base_link`, the angle of the body's rotation in the
  waist frame between two rows equals the angle of that link's nominal
  rotation between them (the unknown mounts cancel). `upper_body_frame`
  matches `torso_2_link` to 0.17 deg RMS (`base_link`, `torso_1_link`: 8 deg
  or more); `endeffector_frame` matches `arm_<side>_7_link` to 0.8 to 1.2
  deg (`arm_<side>_6_link`: 15 deg or more), the remainder being joint
  errors. `waist_frame` is on `base_link`, which moves in the lab (the waist
  spreads by about 0.2 m over a run; it does not move with the torso joints,
  as v1 said). The offsets body to link are unknown and are estimated.
  The robot base pose in the world is not recorded; the study measures the
  end-effector in the waist frame, so it is not needed.
- **Flipped orientations:** the end-effector body's orientation jumps by 90
  or 180 deg on some rows (the marker body re-identified with its axes
  permuted; its origin moves with it, a rotation about a point): 14 of 39 rows of the 14:18 run in three clusters, 1 of 23 of
  the 14:39 run, none of the training run. The fit-free rule (45 deg from the
  run's medoid, `mocap_heldout.flip_clusters`) flags them; the study repairs
  clusters of at least 3 rows with a fitted rotation about a centroid and
  drops smaller ones from its metrics. The files are not changed.
- **Joints:** radians, joint positions `<joint>_joint_position` of the PAL
  statistics (the joint-side measurement, not the motor side): the seven joints
  of the recorded arm and `torso_1_joint`, `torso_2_joint`, at the plateau mean.
  The other arm and all other joints are not shipped.
  Controller joint offsets in force that day are unknown.
- **Clock:** the robot clock is the introspection header stamp; mocap uses the
  `/tf_throttle` header stamps; both are ROS time (seconds, epoch). The offset
  is measured per run in the extractor: the lag maximising the correlation
  between the `upper_body_frame` angular speed and the speed norm of
  `torso_1_joint`, `torso_2_joint`, over +-2 s in 0.02 s steps (`t_mocap =
  t_robot - lag`). The correlation peak is flat to about 0.1 s, which is well inside the 0.5 s
  head and 0.3 s tail trimmed from each plateau. The measured lags are all at most 0.08 s. The stamps of the three bodies were not checked against one another beyond this.
- **Extraction rules** (`examples/talos/utils/mocap_extraction.py`): a plateau
  is the nine joints within 1 mrad over 0.5 s for at least 2 s; the averaging
  window is `[start + 0.5 s, end - 0.3 s]` on the robot clock, shifted by the lag
  on the mocap clock. A plateau is dropped when a body has too
  few samples in the window (20 for waist and upper body, 3 for the
  end-effector body, which arrives at about 1 Hz) or when a body's
  coordinate standard deviation exceeds 5 mm. Drops (plateaus found / too few
  samples / unsteady): 65 / 9 / 0, 65 / 25 / 1, 28 / 5 / 0. Kept windows are below 5 mm. `n_<body>` is the sample count
  and `<body>_std_mm` the largest per-axis std of that body in the window (the end-effector means rest on few samples).
- **Frozen samples and tracking loss:** a frozen sample is a pose identical to
  the previous sample of its body: none, and no NaN. The bags carry no flag
  for a lost body (it publishes nothing), so tracking loss is a gap above 5
  median sample periods; the counts above are those gaps and their total
  length. The end-effector body is lost for a large part of each run
  (183 s, 393 s and 97 s) and arrives at 1 Hz when tracked; that limits the number of usable postures.
- **Known limits:** sensor hardware unknown; body offsets unknown (estimated);
  flipped end-effector orientations on the right-arm runs (above); the
  end-effector pose is a few samples per posture; the clock lag is good to about
  0.1 s; the unshipped plateaus (39 of 158 found are dropped) are mostly those with too few end-effector samples; the throttled topics hide the original
  rates; the validation run is the other arm than the training run, so the
  arm-specific parameters of the left arm are not checked by it (the torso
  chain and the registrations are shared).
- **Not shipped:** the CSV exports that accompany the bags are not used; the extractor reads the bags.
- **Extraction:** `python -m examples.talos.utils.mocap_extraction BAG OUT --arm
  left|right --expected-sha256 SHA` from the repository root (`rosbags`
  required; the lag is measured and printed with the drops and the audit).
  `tests/test_talos_mocap_extraction.py` re-creates the three files byte for
  byte, and checks lag, drops and audit, when `TALOS_RECORDINGS_ROOT` points at
  a folder holding the original recordings; `tests/test_talos_mocap_2022_03_15.py`
  checks the shipped files alone.
