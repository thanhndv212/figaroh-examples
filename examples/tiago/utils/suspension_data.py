"""Validated data containers for the experimental TIAGo suspension example."""

from __future__ import annotations

import csv
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path
from typing import Dict, Optional

import numpy as np
from scipy.signal import butter, filtfilt
from scipy.spatial.transform import Rotation

from examples.tiago.utils.suspension_extraction import full_header, COLUMNS


@dataclass(frozen=True)
class SuspensionTrajectory:
    """Synchronized generalized-base motion and wrench measurements in SI units."""

    time: np.ndarray
    base_displacement: np.ndarray
    base_velocity: np.ndarray
    wrench: np.ndarray
    frame: str

    def __post_init__(self) -> None:
        time = np.asarray(self.time, dtype=float)
        displacement = np.asarray(self.base_displacement, dtype=float)
        velocity = np.asarray(self.base_velocity, dtype=float)
        wrench = np.asarray(self.wrench, dtype=float)

        if time.ndim != 1 or time.size < 2:
            raise ValueError(
                "time must be a one-dimensional array with at least two samples"
            )
        if not np.all(np.isfinite(time)) or np.any(np.diff(time) <= 0.0):
            raise ValueError("time must be finite and strictly increasing")
        for name, values in {
            "base_displacement": displacement,
            "base_velocity": velocity,
            "wrench": wrench,
        }.items():
            if values.shape != (time.size, 6):
                raise ValueError(
                    f"{name} must have shape ({time.size}, 6), got {values.shape}"
                )
            if not np.all(np.isfinite(values)):
                raise ValueError(f"{name} must contain only finite values")
        if not self.frame:
            raise ValueError("frame must be a non-empty frame identifier")

        object.__setattr__(self, "time", time)
        object.__setattr__(self, "base_displacement", displacement)
        object.__setattr__(self, "base_velocity", velocity)
        object.__setattr__(self, "wrench", wrench)

    @property
    def sample_count(self) -> int:
        """Return the synchronized sample count."""
        return self.time.size

    @property
    def wrench_vector(self) -> np.ndarray:
        """Return a DOF-major wrench vector: Fx, Fy, Fz, Mx, My, Mz."""
        return self.wrench.reshape(-1, order="F")


def load_vicon_forceplate_trajectory(
    path: str | Path, fy_sign: float = 1.0
) -> SuspensionTrajectory:
    """Load the historical TIAGo Vicon and force-plate CSV export.

    The source file has no header and uses the historical column order: timestamp,
    ten marker triplets, force, moment, and center of pressure. The fitted model
    uses base marker 1 as the translational origin and the base 1/2/3 marker triad
    for relative roll-pitch-yaw displacement.

    This is the status-quo loader of examples#70 (baseline B1 of the study): it
    keeps the four errors the issue lists (corner-marker origin, plate-centre
    moments, mixed frames, receive-time stamps). Use :func:`load_run` for the
    corrected pipeline.

    ``fy_sign`` multiplies the recorded plate ``Fy``. The 2023-09-20 runs need
    ``-1`` (their ``Fy`` is flipped against world y; #70 part 2, the study config
    ``plate_fy_sign``). The default ``+1`` reads the file as recorded, which is
    the status quo (baseline B1).
    """
    path = Path(path)
    if not path.is_file():
        raise FileNotFoundError(f"Vicon force-plate log not found: {path}")

    rows: list[list[str]] = []
    with path.open(newline="") as log_file:
        for row in csv.reader(log_file):
            if not row:
                continue
            if len(row) != 40:
                raise ValueError(f"expected 40 Vicon columns in {path}, got {len(row)}")
            rows.append(row)
    if len(rows) < 3:
        raise ValueError(f"Vicon force-plate log requires at least three rows: {path}")

    time = np.array(
        [datetime.fromisoformat(row[0]).timestamp() for row in rows], dtype=float
    )
    values = np.asarray([[float(value) for value in row[1:]] for row in rows])
    return status_quo_from_arrays(time, values, source=str(path), fy_sign=fy_sign)


load_status_quo = load_vicon_forceplate_trajectory


def status_quo_from_arrays(
    time: np.ndarray, values: np.ndarray, source: str = "array", fy_sign: float = 1.0
) -> SuspensionTrajectory:
    """The status-quo pipeline on ``time`` (receive stamps) and the 39 values."""
    if fy_sign not in (1, -1):
        raise ValueError(fy_sign)
    if np.any(np.diff(time) <= 0.0):
        raise ValueError(f"Vicon timestamps must be strictly increasing: {source}")
    if not np.all(np.isfinite(values)):
        raise ValueError(f"Vicon log contains non-finite measurements: {source}")

    base_1 = values[:, 0:3]
    base_2 = values[:, 3:6]
    base_3 = values[:, 6:9]
    rotation = _base_marker_rotation(base_1, base_2, base_3)
    relative_rotation = np.einsum("ji,njk->nik", rotation[0], rotation)
    orientation = np.unwrap(_rotation_to_rpy(relative_rotation), axis=0)
    displacement = np.column_stack((base_1 - base_1[0], orientation))
    velocity = np.gradient(displacement, time, axis=0, edge_order=2)
    wrench = values[:, 30:36] * np.array([1.0, fy_sign, 1.0, 1.0, 1.0, 1.0])
    wrench = wrench - np.mean(wrench[: min(100, len(wrench))], axis=0)

    return SuspensionTrajectory(
        time=time,
        base_displacement=displacement,
        base_velocity=velocity,
        wrench=wrench,
        frame="vicon_world",
    )


def _base_marker_rotation(
    base_1: np.ndarray, base_2: np.ndarray, base_3: np.ndarray
) -> np.ndarray:
    x_axis = _normalize_rows(base_2 - base_1, "base 1 to base 2")
    z_axis = _normalize_rows(np.cross(x_axis, base_3 - base_1), "base marker triad")
    y_axis = _normalize_rows(np.cross(z_axis, x_axis), "base marker triad")
    return np.stack((x_axis, y_axis, z_axis), axis=2)


def _normalize_rows(vectors: np.ndarray, name: str) -> np.ndarray:
    norms = np.linalg.norm(vectors, axis=1)
    if np.any(norms <= np.finfo(float).eps):
        raise ValueError(f"degenerate {name} marker geometry")
    return vectors / norms[:, None]


def _rotation_to_rpy(rotation: np.ndarray) -> np.ndarray:
    pitch = np.arcsin(np.clip(-rotation[:, 2, 0], -1.0, 1.0))
    roll = np.arctan2(rotation[:, 2, 1], rotation[:, 2, 2])
    yaw = np.arctan2(rotation[:, 1, 0], rotation[:, 0, 0])
    return np.column_stack((roll, pitch, yaw))


# ---------------------------------------------------------------------------
# Corrected pipeline (examples#70, study part 1)
#
# Every default below is the value frozen in
# examples/tiago/config/tiago_suspension_study_config.yaml; the study module
# passes the config values explicitly.
# ---------------------------------------------------------------------------

RATE_HZ = 100.0
PLATE_CENTRE = (0.900, 0.450, 0.0)  # m, Vicon world; M = (COP - c) x F (measured)
STATIC_WINDOW_S = 1.0
GUARD_FRAMES = 5
EDGE_S = 0.5
CUTOFF_HZ = 5.0  # revision of #70 (was 10.0)
FILTER_ORDER = 4


def read_raw(path: str | Path) -> tuple[np.ndarray, np.ndarray]:
    """``(t_receive, values)``: host receive time (s) and the 39 numeric columns.

    Accepts the shipped layout (no header) and the raw layout (a header line of
    20 or 40 names); columns are selected by name, so a header in any of the
    two forms gives the same arrays. Strictly 40 columns per row.
    """
    path = Path(path)
    if not path.is_file():
        raise FileNotFoundError(f"Vicon force-plate log not found: {path}")
    with path.open(newline="") as f:
        rows = [r for r in csv.reader(f) if r]
    header = None
    if rows and rows[0][0] == "time":
        header, rows = rows[0], rows[1:]
    names = full_header(header)
    if sorted(names) != sorted(COLUMNS) or len(set(names)) != 40:
        raise ValueError(f"{path}: columns are not the CREPS layout")
    index = [names.index(c) for c in COLUMNS]
    for row in rows:
        if len(row) != 40:
            raise ValueError(f"expected 40 Vicon columns in {path}, got {len(row)}")
    if len(rows) < 3:
        raise ValueError(f"Vicon force-plate log requires at least three rows: {path}")
    t = np.array(
        [datetime.fromisoformat(r[index[0]]).timestamp() for r in rows], dtype=float
    )
    values = np.asarray([[float(r[i]) for i in index[1:]] for r in rows])
    return t, values


@dataclass(frozen=True)
class Clock:
    """Frame numbers on the Vicon grid for the receive-stamped rows."""

    frame: np.ndarray  # int, strictly increasing, one per kept row
    keep: np.ndarray  # bool over the input rows (False: duplicate dropped)
    latency: np.ndarray  # s, >= 0, per kept row
    rate: float

    @property
    def n_frames(self) -> int:
        return int(self.frame[-1]) + 1

    @property
    def duplicates(self) -> int:
        return int((~self.keep).sum())

    @property
    def drops(self) -> int:
        return int(np.sum(np.diff(self.frame) - 1))

    def stats(self) -> Dict[str, float]:
        lat = self.latency
        gaps = np.diff(self.frame)
        return {
            "rows": int(self.keep.size),
            "duplicates": self.duplicates,
            "duplicate_pct": 100.0 * self.duplicates / self.keep.size,
            "frames": self.n_frames,
            "drops": self.drops,
            "drop_pct": 100.0 * self.drops / self.n_frames,
            "max_gap_frames": int(gaps.max() - 1) if gaps.size else 0,
            "latency_mean_ms": 1e3 * float(lat.mean()),
            "latency_sd_ms": 1e3 * float(lat.std()),
            "latency_p95_ms": 1e3 * float(np.percentile(lat, 95)),
            "latency_max_ms": 1e3 * float(lat.max()),
        }


def rebuild_clock(
    t: np.ndarray, values: Optional[np.ndarray] = None, rate: float = RATE_HZ
) -> Clock:
    """Rebuild integer frame numbers on the ``rate`` grid from receive stamps.

    Duplicates are rows whose values are identical to the previous row (only
    when ``values`` is given): dropped and counted. For the kept rows the
    receive time is ``t_k = t_0 + (n_k + l_k) / rate`` with a latency ``l_k``
    >= 0 (a frame cannot arrive before it was made). ``n_k`` is the largest
    non-decreasing integer sequence compatible with that: with
    ``d_k = (t_k - t_0) rate - k`` the cumulative drop count is
    ``D_k = floor(min_{j >= k} d_j)`` and ``n_k = k + D_k - D_0``. The latency
    is then the minimum-spread one (lower envelope), ``(d_k - D_k) / rate``,
    defined up to the lowest latency of the run. A drop is seen where the
    envelope of ``d`` rises through an integer: its row can be off by the few
    rows the local latency needs to show a minimum, hence the guard rows
    around every gap in :func:`load_run`.
    """
    t = np.asarray(t, dtype=float)
    keep = np.ones(t.size, dtype=bool)
    if values is not None:
        values = np.asarray(values)
        keep[1:] = ~np.all(values[1:] == values[:-1], axis=1)
    tk = t[keep]
    if tk.size < 2 or np.any(np.diff(tk) <= 0.0):
        raise ValueError("receive times must be strictly increasing")
    d = (tk - tk[0]) * rate - np.arange(tk.size)
    envelope = np.minimum.accumulate(d[::-1])[::-1]
    drops = np.floor(envelope + 1e-9).astype(int)
    frame = np.arange(tk.size) + drops - drops[0]
    latency = (d - drops) / rate
    return Clock(frame=frame, keep=keep, latency=latency, rate=rate)


def to_grid(clock: Clock, arrays: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Arrays of the kept rows on the integer frame grid.

    ``(grid_arrays, interpolated)``: dropped frames are linearly interpolated
    and flagged. ``arrays`` has the length of the *input* rows.
    """
    arrays = np.asarray(arrays, dtype=float)[clock.keep]
    n = np.arange(clock.n_frames)
    out = np.column_stack(
        [np.interp(n, clock.frame, arrays[:, i]) for i in range(arrays.shape[1])]
    )
    seen = np.zeros(clock.n_frames, dtype=bool)
    seen[clock.frame] = True
    return out, ~seen


def lowpass(x: np.ndarray, rate: float, cutoff: float, order: int = FILTER_ORDER):
    """Zero-phase Butterworth low-pass along axis 0 (identity if cutoff is None)."""
    if cutoff is None:
        return np.asarray(x, dtype=float)
    b, a = butter(order, cutoff / (0.5 * rate))
    return filtfilt(b, a, np.asarray(x, dtype=float), axis=0)


def base_frame(ref: np.ndarray) -> np.ndarray:
    """Right-handed, z-up base frame of the rest triad ``ref`` (3, 3).

    Columns are the frame axes in world coordinates: x along base1 to base2
    (made horizontal), z up (the marker-plane normal, sign-fixed: the normal
    ``(b2-b1) x (b3-b1)`` of the status-quo loader points down), y = z x x.
    """
    ref = np.asarray(ref, dtype=float)
    normal = np.cross(ref[1] - ref[0], ref[2] - ref[0])
    if normal[2] < 0.0:
        normal = -normal
    z = normal / np.linalg.norm(normal)
    x = (ref[1] - ref[0]) - np.dot(ref[1] - ref[0], z) * z
    x = x / np.linalg.norm(x)
    return np.column_stack((x, np.cross(z, x), z))


def frame_yaw_deg(ref: np.ndarray) -> float:
    """Yaw of :func:`base_frame` about world z, degrees."""
    r = base_frame(ref)
    return float(np.degrees(np.arctan2(r[1, 0], r[0, 0])))


def triad_pose(markers: np.ndarray, ref: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """``(translation, rotvec)`` of a marker triad relative to the rest pose.

    ``markers`` (n, 3, 3) and ``ref`` (3, 3). The origin is the triad centroid
    (a rotation about any axis through it gives zero translation); the rotation
    is the Kabsch fit of all three markers, as a world-frame rotation vector.
    """
    markers = np.asarray(markers, dtype=float)
    ref = np.asarray(ref, dtype=float)
    p0 = ref - ref.mean(axis=0)
    centroid = markers.mean(axis=1)
    p = markers - centroid[:, None, :]
    h = np.einsum("ia,nib->nab", p0, p)  # sum_i p0_i p_i^T
    u, _, vt = np.linalg.svd(h)
    v = np.swapaxes(vt, 1, 2)
    sign = np.sign(np.linalg.det(np.einsum("nab,ncb->nac", v, u)))
    d = np.zeros((markers.shape[0], 3, 3))
    d[:, 0, 0] = d[:, 1, 1] = 1.0
    d[:, 2, 2] = sign
    rot = np.einsum("nab,nbc,ndc->nad", v, d, u)  # V D U^T maps p0 -> p
    return centroid - ref.mean(axis=0), Rotation.from_matrix(rot).as_rotvec()


def marker_frames(markers: np.ndarray) -> np.ndarray:
    """(n, 3, 3) frames of marker triads ``(n, 3, 3)``: columns x, y, z in world.

    x from marker 1 to marker 2, z = x cross (marker 3 - marker 1), y = z cross
    x, as in the data README (for the base triad z points down).
    """
    markers = np.asarray(markers, dtype=float)
    return _base_marker_rotation(markers[:, 0], markers[:, 1], markers[:, 2])


def footprint_rest(
    ref: np.ndarray, offset, rotvec, min_up: float = 0.8
) -> tuple[np.ndarray, np.ndarray]:
    """``(point, axes)`` of the base_footprint frame at the rest triad ``ref`` (3, 3).

    ``offset`` is the footprint origin in the triad frame (robot-fixed) and
    ``rotvec`` the rotation from the triad frame to the footprint axes (both
    from the config, derived by ``suspension_model.footprint_derivation``);
    ``point`` and ``axes`` (columns x, y, z) are in world. The footprint z must
    point up (``axes[2, 2] >= min_up``): the triad z points down and the
    rotation to the footprint axes turns it up.
    """
    ref = np.asarray(ref, dtype=float)
    r_tri = marker_frames(ref[None])[0]
    point = ref.mean(axis=0) + r_tri @ np.asarray(offset, dtype=float)
    axes = r_tri @ Rotation.from_rotvec(np.asarray(rotvec, dtype=float)).as_matrix()
    if axes[2, 2] < min_up:
        raise ValueError(f"footprint z axis is not up (z.z = {axes[2, 2]:.3f})")
    return point, axes


def base_pose(
    markers: np.ndarray,
    ref: np.ndarray,
    *,
    origin: str = "centroid",
    frame: str = "world",
    footprint: Optional[dict] = None,
):
    """``(trans, angle, point, axes)`` of the base triad on its rest pose ``ref``.

    ``origin``: ``footprint`` (rigid with the triad, ``footprint['offset']`` and
    ``['rotvec']``), ``centroid`` or ``corner`` (marker base1). ``frame``:
    ``footprint`` (axes of the rest footprint frame), ``base`` (:func:`base_frame`),
    ``world`` (also ``legacy``, whose angle the caller replaces). ``trans`` is
    the translation of ``point`` and ``angle`` the rotation vector, both in the
    axes ``axes`` (world components times ``axes``).
    """
    if origin not in ("footprint", "centroid", "corner"):
        raise ValueError(origin)
    if frame not in ("footprint", "base", "world", "legacy"):
        raise ValueError(frame)
    if "footprint" in (origin, frame) and footprint is None:
        raise ValueError("the footprint offset and rotation are required")
    markers = np.asarray(markers, dtype=float)
    ref = np.asarray(ref, dtype=float)
    centre = ref.mean(axis=0)
    trans, rotvec = triad_pose(markers, ref)
    if origin == "footprint":
        point = footprint_rest(ref, **footprint)[0]
        rot = Rotation.from_rotvec(rotvec).as_matrix() - np.eye(3)
        trans = trans + rot @ (point - centre)
    elif origin == "corner":
        point, trans = ref[0], markers[:, 0, :] - ref[0]
    else:
        point = centre
    axes = np.eye(3)
    if frame == "footprint":
        axes = footprint_rest(ref, **footprint)[1]
    elif frame == "base":
        axes = base_frame(ref)
    return trans @ axes, rotvec @ axes, point, axes


def transport_moment(
    moment: np.ndarray, force: np.ndarray, centre, point
) -> np.ndarray:
    """Moment about ``point`` from the moment about ``centre``: ``M + (c - p) x F``."""
    lever = np.asarray(centre, dtype=float) - np.asarray(point, dtype=float)
    return np.asarray(moment, dtype=float) + np.cross(lever, force)


def plate_centre_fit(
    force: np.ndarray, moment: np.ndarray, cop: np.ndarray
) -> tuple[np.ndarray, float]:
    """Plate centre ``c`` (and Mx, My residual rms) from ``M = (COP - c) x F``.

    The relation holds for Mx and My only (Mz is the free moment). A check of
    the plate convention, not a fit of the suspension.
    """
    f = np.asarray(force, dtype=float)
    rhs = np.asarray(moment, dtype=float) - np.cross(cop, f)
    skew = np.zeros((f.shape[0], 3, 3))
    skew[:, 0, 1], skew[:, 0, 2] = -f[:, 2], f[:, 1]
    skew[:, 1, 0], skew[:, 1, 2] = f[:, 2], -f[:, 0]
    a = skew[:, :2, :].reshape(-1, 3)  # F x c, rows x and y
    sol = np.linalg.lstsq(a, rhs[:, :2].reshape(-1), rcond=None)[0]
    res = rhs[:, :2].reshape(-1) - a @ sol
    return sol, float(np.sqrt(np.mean(res**2)))


@dataclass
class SuspensionRun:
    """One run on the 100 Hz frame grid, in one frame, about one origin.

    ``x`` (displacement: translation of the origin m, rotation vector rad),
    ``v`` its derivative, ``w`` the measured wrench of the force plate
    (action on the plate, about the origin) minus its static offset. ``valid``
    marks the rows a fit may use (not edge, not within ``guard`` frames of an
    interpolated frame). ``grip`` and ``grip_rot`` are the centroid and the
    frame (:func:`marker_frames`) of the gripper-marker triad, ``shoulder`` the
    shoulder-cluster centroid (world, filtered like ``x``); the arm baseline B2
    uses the gripper pair. ``origin`` is the world point the wrench is about
    and ``axes`` (columns) the axes of ``x``, ``v`` and ``w`` in world.
    """

    t: np.ndarray
    x: np.ndarray
    v: np.ndarray
    w: np.ndarray
    valid: np.ndarray
    interpolated: np.ndarray
    grip: np.ndarray
    grip_rot: np.ndarray
    shoulder: np.ndarray
    origin: np.ndarray
    ref: np.ndarray
    axes: np.ndarray
    frame: str
    stats: Dict[str, float] = field(default_factory=dict)

    @property
    def n(self) -> int:
        return self.t.size


def process_raw(
    t: np.ndarray,
    values: np.ndarray,
    *,
    rate: float = RATE_HZ,
    static_window_s: float = STATIC_WINDOW_S,
    guard_frames: int = GUARD_FRAMES,
    edge_s: float = EDGE_S,
    cutoff_hz: Optional[float] = CUTOFF_HZ,
    plate_centre=PLATE_CENTRE,
    frame: str = "world",
    origin: str = "centroid",
    footprint: Optional[dict] = None,
    transport: bool = True,
    clock: str = "grid",
    mz_about: str = "centre",
    plate_fy_sign: float = 1.0,
) -> SuspensionRun:
    """Corrected features of raw receive-stamped rows (``t``, 39 ``values``).

    Switches (``origin``, ``transport``, ``frame``, ``clock``) restore one
    status-quo error each for the ablation of the study; the defaults are the
    corrected pipeline of the study (``footprint`` origin and frame, from the
    config); the plain defaults are the triad centroid in world axes.
    ``origin``: ``footprint`` (needs ``footprint`` = ``{offset, rotvec}``, see
    :func:`footprint_rest`), ``centroid`` or ``corner`` (marker base1).
    ``frame``: ``footprint`` (axes of the rest footprint frame), ``world`` or
    ``base`` (axes of :func:`base_frame`), ``legacy`` for the status-quo
    relative RPY in the down-pointing marker frame (translation and wrench stay
    in world axes).
    ``clock``: ``grid`` (rebuilt frame numbers) or ``receive`` (rows as
    recorded, differentiated on the receive stamps).
    ``mz_about`` (#70 part 2, O7): ``centre`` reads the recorded ``Mz`` as the
    moment about the plate centre ``c`` (the convention of every other stage);
    ``cop`` reads it as the free moment about the COP, so ``(COP - c) x F``
    (z component) is added to bring it to ``c``. ``Mx`` and ``My`` are unchanged.
    ``plate_fy_sign`` (#70 part 2 revision) multiplies the recorded ``Fy`` before
    any other use; ``-1`` on the real runs, whose plate ``Fy`` is flipped against
    world y.
    """
    if clock not in ("grid", "receive"):
        raise ValueError(clock)
    if mz_about not in ("centre", "cop"):
        raise ValueError(mz_about)
    if plate_fy_sign not in (1, -1):
        raise ValueError(plate_fy_sign)
    t = np.asarray(t, dtype=float)
    values = np.asarray(values, dtype=float)
    if clock == "grid":
        clk = rebuild_clock(t, values, rate)
        data, interp = to_grid(clk, values)
        tt = np.arange(clk.n_frames) / rate
        stats = clk.stats()
    else:
        clk = None
        data, interp = values, np.zeros(values.shape[0], dtype=bool)
        tt = t - t[0]
        stats = {"rows": int(values.shape[0])}
    n = data.shape[0]
    nw = max(2, int(round(static_window_s * rate)))

    markers = data[:, 0:9].reshape(n, 3, 3)
    ref = markers[:nw].mean(axis=0)
    trans, angle, p_b, axes = base_pose(
        markers, ref, origin=origin, frame=frame, footprint=footprint
    )
    if frame == "legacy":
        rot = marker_frames(markers)
        rel = np.einsum("ji,njk->nik", rot[0], rot)
        angle = np.unwrap(_rotation_to_rpy(rel), axis=0)

    force = data[:, 30:33] * np.array([1.0, plate_fy_sign, 1.0])
    moment = data[:, 33:36]
    if mz_about == "cop":
        c = np.asarray(plate_centre, dtype=float)
        cop = data[:, 36:39]
        moment = moment.copy()
        moment[:, 2] += (cop[:, 0] - c[0]) * force[:, 1] - (cop[:, 1] - c[1]) * force[:, 0]
    if transport:
        moment = transport_moment(moment, force, plate_centre, p_b)
    wrench = np.column_stack((force @ axes, moment @ axes))

    grip_markers = data[:, 21:30].reshape(n, 3, 3)
    grip = grip_markers.mean(axis=1)
    grip_rot = marker_frames(grip_markers).reshape(n, 9)
    shoulder = data[:, 9:21].reshape(n, 4, 3).mean(axis=1)
    x = np.column_stack((trans, angle))
    if clock == "grid":
        x, wrench = lowpass(x, rate, cutoff_hz), lowpass(wrench, rate, cutoff_hz)
        grip, shoulder = lowpass(grip, rate, cutoff_hz), lowpass(
            shoulder, rate, cutoff_hz
        )
        grip_rot = lowpass(grip_rot, rate, cutoff_hz)
        v = np.gradient(x, 1.0 / rate, axis=0)
    else:
        v = np.gradient(x, t, axis=0, edge_order=2)
    wrench = wrench - wrench[:nw].mean(axis=0)

    valid = np.ones(n, dtype=bool)
    edge = int(round(edge_s * rate)) if clock == "grid" else 0
    if edge:
        valid[:edge] = valid[-edge:] = False
    if interp.any():
        near = np.flatnonzero(interp)
        for k in range(-guard_frames, guard_frames + 1):
            valid[np.clip(near + k, 0, n - 1)] = False
    stats.update(
        {
            "origin": origin,
            "frame": frame,
            "origin_world_z": float(p_b[2]),
            "clock": clock,
            "mz_about": mz_about,
            "plate_fy_sign": plate_fy_sign,
            "frame_yaw_deg": frame_yaw_deg(ref),
            "valid_rows": int(valid.sum()),
        }
    )
    return SuspensionRun(
        t=tt,
        x=x,
        v=v,
        w=wrench,
        valid=valid,
        interpolated=interp,
        grip=grip,
        grip_rot=grip_rot.reshape(n, 3, 3),
        shoulder=shoulder,
        origin=p_b,
        ref=ref,
        axes=axes,
        frame=frame,
        stats=stats,
    )


def load_run(path: str | Path, **kwargs) -> SuspensionRun:
    """Corrected :class:`SuspensionRun` of one CREPS log (see :func:`process_raw`)."""
    t, values = read_raw(path)
    return process_raw(t, values, **kwargs)
