"""Extraction rules for the TIAGo PAL introspection exports (#109, #110).

The original recordings are not distributed; the synthetic cases pin the file
layout written by examples/tiago/utils/introspection_extraction.py. The slow
tests re-create shipped files when TIAGO_RECORDINGS_ROOT points at the
original recordings.
"""

import os
from pathlib import Path

import pytest
import yaml

from examples.tiago.utils.introspection_extraction import (
    backlash_channels,
    channel_table,
    names_csv,
    read_introspection,
    read_tf_csv,
    trim,
    values_csv,
)
from examples.tiago.utils.mocap_extraction import file_sha256

NAMES = (1690205323100213486, 1, 1690202869, 475955572, 2, ["a", "b", "c"])
ROWS = [
    (1690205323106335517, 63795, 1690205323, 105899893, 2, [1.0, 0.048, float("nan")]),
    (1690205323116335517, 63796, 1690205323, 115899893, 2, [2.0, 6.5e-06, -0.5]),
]


def test_names_file_keeps_the_first_name_in_the_cell_and_the_rest_in_the_header():
    text = names_csv(NAMES)
    head, row, tail = text.split("\n")
    assert tail == ""
    assert head == (
        "rosbagTimestamp,header,seq,stamp,secs,nsecs,frame_id,names,- b,- c,"
        "names_version"
    )
    assert row == "1690205323100213486,,1,,1690202869,475955572,'',- a,2"


def test_values_file_writes_the_list_with_repr_and_nan():
    lines = values_csv(ROWS).split("\n")
    assert lines[0] == (
        "rosbagTimestamp,header,seq,stamp,secs,nsecs,frame_id,values,names_version"
    )
    assert lines[1] == (
        "1690205323106335517,,63795,,1690205323,105899893,''," '"[1.0, 0.048, nan]",2'
    )
    assert lines[2].endswith('"[2.0, 6.5e-06, -0.5]",2')


def test_trim_keeps_values_unchanged_and_in_recorded_order():
    names, rows = trim(NAMES, ROWS, ["c", "a"])
    assert names[5] == ["a", "c"]
    assert rows[1][5] == [2.0, -0.5]
    with pytest.raises(KeyError):
        trim(NAMES, ROWS, ["zzz"])


def test_backlash_channels_are_the_loader_signals_after_the_leading_channel():
    ch = backlash_channels()
    assert len(ch) == len(set(ch)) == 33
    assert ch[0].startswith("topic_stats.")
    assert "arm_5_joint_absolute_encoder_position" in ch
    assert "torso_lift_joint_effort" in ch


def test_channel_table_puts_the_selected_channels_on_the_header_clock():
    text = channel_table(NAMES, ROWS, [("x", "c"), ("y", "a")])
    lines = text.strip().split("\n")
    assert lines[0] == "t,x,y"
    assert lines[1].split(",")[0] == "0.0"
    assert float(lines[2].split(",")[0]) == pytest.approx(0.01)
    assert lines[2].endswith(",-0.5,2.0")
    with pytest.raises(KeyError):
        channel_table(NAMES, ROWS, [("x", "zzz")])


def test_tf_csv_reader_follows_the_shifted_17_field_blocks(tmp_path):
    def block(sec, nsec, parent, child, x):
        return [
            "",
            "0",
            "",
            str(sec),
            str(nsec),
            f'"{parent}"',
            f'"{child}"',
            "",
            "",
            str(x),
            "0.0",
            "0.0",
            "",
            "0.0",
            "0.0",
            "0.0",
            "1.0",
        ]

    first = ["170", "", ""] + block(1, 2, "a", "b", 0.5)[1:]
    row = first + block(1, 3, "b", "c", 0.25)
    path = tmp_path / "tf.csv"
    path.write_text("rosbagTimestamp,transforms,-,header\n" + ",".join(row) + "\n")
    ((t, tfs),) = read_tf_csv(str(path))
    assert t == 170
    assert [(c[2], c[3]) for c in tfs] == [("a", "b"), ("b", "c")]
    assert tfs[0][4] == (0.5, 0.0, 0.0)
    assert tfs[1][1] == 3 and tfs[1][5] == (0.0, 0.0, 0.0, 1.0)


def _find(root: Path, entry: dict) -> Path:
    for path in root.rglob(entry["name"]):
        if file_sha256(path) == entry["sha256"]:
            return path
    raise AssertionError(f"{entry['name']} (sha256 {entry['sha256']}) not under {root}")


@pytest.mark.slow
@pytest.mark.skipif(
    not os.environ.get("TIAGO_RECORDINGS_ROOT"),
    reason="needs the original recordings: set TIAGO_RECORDINGS_ROOT",
)
def test_backlash_files_are_re_created_byte_for_byte():
    data = Path(__file__).resolve().parents[1] / "examples/tiago/data/backlash"
    manifest = yaml.safe_load((data / "protocol.yaml").read_text())
    root = Path(os.environ["TIAGO_RECORDINGS_ROOT"])
    for s in manifest["sessions"]:
        src = s["source"]
        if "bag" in src:
            names_msg, rows = read_introspection(str(_find(root, src["bag"])))
            names_msg, rows = trim(names_msg, rows, backlash_channels())
            out = {
                "introspection_datanames.csv": names_csv(names_msg),
                "introspection_datavalues.csv": values_csv(rows),
            }
            for name, text in out.items():
                (path,) = [p for p in s["files"] if p.endswith(name)]
                assert (data / path).read_text() == text, path
        elif "tf_csv" in src:
            (path,) = s["files"]
            assert file_sha256(_find(root, src["tf_csv"])) == s["files"][path]
