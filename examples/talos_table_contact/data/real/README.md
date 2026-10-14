# TALOS table-contact postures (real data)

Static postures of TALOS (robot "pyrene") whose gripper was commanded to touch
a table flush, recorded 2022-10-28 (left arm) and 2022-11-07 (both arms).
Provenance, roles and hashes are in [`protocol.yaml`](protocol.yaml)
(protocol v2). The sources are original recordings, not distributed; they are named there by file name and sha256. Everything the
example needs is in this directory.

| File | Postures | Session | Role |
|---|---|---|---|
| `left_train.csv` | 21 | 2022-10-28 | training |
| `left_validation.csv` | 9 | 2022-11-07 | validation |
| `right_train.csv` | 29 | 2022-11-07 | training |
| `right_validation.csv` | 9 | 2022-11-07 | validation |
| `contact_evidence.csv` | 68 | all of the above | evidence, not a fit input |

## Provenance

Each posture row is one line of a per-posture measurement file written by the
recording session (a `Saved joint states` step of the motion playback). All
68 shipped rows match their source line exactly; `protocol.yaml` lists the
source file and line of every posture, and the sha256 of every measurement
file. `python -m examples.talos_table_contact.utils.real_data_extraction
--source-root <recordings> --check` rebuilds every file here byte for byte
(sources use mixed CRLF/LF; the shipped files use LF). Never re-save these
CSVs in a spreadsheet.

Two postures are genuine repeats, kept: `table/contact_21` appears twice in
`left_train.csv` (rows 2 and 8) and `table/contact_20` twice in
`right_train.csv` (rows 25 and 26); they are separate measurement lines.

## Format, units, frames, clock

- Posture CSVs: `gripper`, gripper frame (`talos/<side>_gripper`), `handle`,
  contact marker (`table/contact_NN`), `joint_states`, then 32 joint
  positions in rad in the order of the header. No timestamps: each row is a
  static posture, and `session_id` does not exist in the data (one file, one
  session). Joints are read by position; the marker IDs carry no table
  coordinates. Frames are those of the TALOS URDF (root `<side>_sole_link`).
- **Header fix (protocol v2).** v1 headers of `left_validation.csv`,
  `right_train.csv` and `right_validation.csv` lacked the four leading empty
  cells, so `joint_names` sat over `joint_states` and every joint name was
  4 columns off. v2 gives all four files the header `left_train.csv` always
  had (37 fields, like the rows). No value changed. v1 sha256 (recorded in
  `protocol.yaml`): `left_train` d2fadbf1, `left_validation` d103fa…,
  `right_train` 799fb8…, `right_validation` 370fcb… (only the last three
  changed; see `previous_version`).
- `contact_evidence.csv` columns: `posture_file`, `row` (line in that file),
  `contact_marker`, `status`, `bag`, `stamp_s`, `max_joint_diff_rad`,
  `force_{x,y,z}_N`, `torque_{x,y,z}_Nm`, `contact`. Wrench is the wrist F/T
  sensor of the posture's own arm (`<side>_wrist_ft_*`), sensor frame, N and
  N m, as logged. `stamp_s` is the ROS header stamp of the introspection
  message (seconds since the Unix epoch, UTC, robot clock).

## Contact evidence

The postures were saved from the joint-state stream; the bags are 1 Hz
throttled PAL introspection (`/introspection_data/*_throttle`, joint
position/velocity/effort and wrist, ankle F/T). Each posture is joined to the
nearest introspection sample by maximum absolute joint difference, accepted
only if it is at most **0.3 mrad**; the time resolution is therefore about
**±1 s**. `contact` is 1 when the wrist `Fz` is at least **+10 N**. Both
thresholds were fixed before screening and are not tuned; #98 should use them
as they are.

Result: **21 of 68 postures** have evidence (left train rows 2-5 and 17-22,
right train rows 2-12). All 21 have `Fz` between +15.4 and +21.3 N, against a
resting median of about -10 N in the same bags, so all are flagged as in
contact. The other **47 have `status = no_matching_sample` and empty
fields**: no readable recording holds them. The relevant bags are empty
(zero messages), not indexed (`.bag.active`) or statistics-only (4 channels,
no joints). One bag (2022-10-28 09:00:43, status `recovered`) has 604-value
messages but no matching name list; it is decoded with the 604-channel list of
the searched bags, which is identical in every one of them (the extractor
checks this), and its 42 samples match no shipped posture. This covers all 18 validation postures, left train
rows 6-16 and right train rows 13-30. A missing signal is not evidence of
no contact.

## Not shipped

Listed in `protocol.yaml` (`bags`, `unshipped`, `unshipped_sessions`), with the
reason per recording: the 2022-10-14 and 2022-10-25 sessions (reported drift,
role diagnostic; their bags are searched and match nothing), the 2022-10-27
test session (`measurements_1.csv` is used by no shipped file), and every
bag that is empty, unindexed or statistics-only. Roles are frozen: changing a
file or a role needs a new protocol version.

## Known limits

- Not 6D truth: postures are assumed flush contacts; there is no table pose,
  no gripper contact sensor, and no rejected-contact log.
- Wrist `Fz` is an unzeroed raw reading; the +10 N rule rests on the observed
  separation (about 30 N), not on a calibrated bias.
- Whether the table moved between 2022-10-28 and 2022-11-07 is unrecorded.
- Contact evidence exists only for training postures of two sessions.

## Attribution

Recorded by Thanh Nguyen (FIGAROH maintainer) on the TALOS robot (host
`pyrene`) with the ROS/HPP pipeline
[`agimus-demos/talos/calibration/contact`](https://github.com/agimus/agimus-demos/tree/master/talos/calibration/contact);
the posture files were copied from the deprecated FIGAROH repository.
Extraction and evidence: examples#113.
