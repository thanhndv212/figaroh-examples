"""Normalise the CREPS Vicon and force-plate logs to one column layout.

The 2023-09-20 CREPS session recorded eleven Vicon logs (ten markers and force
plate 4, about 100 Hz). One of them, ``tiago_xyz_vicon_1640.csv``, was shipped
without a header line. The others have one, in two forms: 20 names, with one
name per marker triple (``base1``) and one per plate value, over 40 values per
row; and 40 names (``base1_x``, ``base1_y``, ``base1_z``) in the two postures
runs. The rows also end in CRLF where the shipped file uses LF.

:func:`normalise` writes every log in the layout of the shipped file: no
header, 40 columns in the order of :data:`COLUMNS`, LF line ends. Columns are
selected by name; every field is copied as the text found, so no value is
re-formatted or changed. The first column is the host receive time of the
frame (``YYYY-MM-DD_HH:MM:SS.ffffff``), not the Vicon frame time. The logs hold
no frame-index column: the row order is the frame order, and frames dropped or
duplicated by the recorder cannot be told apart from the receive times alone.

The original recordings are not distributed. Use::

    python -m examples.tiago.utils.suspension_extraction <raw.csv> <out.csv>
"""

from __future__ import annotations

import csv
from typing import List, Optional, Sequence, Tuple

MARKERS = (
    "base1",
    "base2",
    "base3",
    "shoulder1",
    "shoulder2",
    "shoulder3",
    "shoulder4",
    "gripper1",
    "gripper2",
    "gripper3",
)
PLATE = ("Fx4", "Fy4", "Fz4", "Mx4", "My4", "Mz4", "COPx4", "COPy4", "COPz4")
COLUMNS: Tuple[str, ...] = (
    ("time",) + tuple(f"{m}_{a}" for m in MARKERS for a in "xyz") + PLATE
)


def read_log(path: str) -> Tuple[Optional[List[str]], List[List[str]]]:
    """Return ``(header or None, rows)`` with every field as text."""
    with open(path, newline="") as f:
        rows = [r for r in csv.reader(f) if r]
    if rows and rows[0][0] == "time":
        return rows[0], rows[1:]
    return None, rows


def full_header(header: Optional[Sequence[str]]) -> List[str]:
    """The 40 column names of a log, whichever header form it has.

    No header: the historical order of :data:`COLUMNS`. 20 names: each marker
    name stands for its x, y, z triple. 40 names: as written.
    """
    if header is None:
        return list(COLUMNS)
    if len(header) == 40:
        return list(header)
    if len(header) == 20:
        out: List[str] = []
        for name in header:
            out += [f"{name}_{a}" for a in "xyz"] if name in MARKERS else [name]
        return out
    raise ValueError(f"unexpected header with {len(header)} names")


def normalise(path: str) -> str:
    """The log in the shipped layout: no header, canonical columns, LF."""
    header, rows = read_log(path)
    names = full_header(header)
    if sorted(names) != sorted(COLUMNS) or len(set(names)) != 40:
        raise ValueError(f"{path}: columns are not the CREPS layout: {names}")
    index = [names.index(c) for c in COLUMNS]
    out = []
    for n, row in enumerate(rows):
        if len(row) != 40:
            raise ValueError(f"{path}: row {n} has {len(row)} fields, expected 40")
        out.append(",".join(row[i] for i in index) + "\n")
    return "".join(out)


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("raw_csv")
    parser.add_argument("out_csv")
    args = parser.parse_args()
    text = normalise(args.raw_csv)
    with open(args.out_csv, "w", newline="") as f:
        f.write(text)
    print(f"{text.count(chr(10))} rows")
