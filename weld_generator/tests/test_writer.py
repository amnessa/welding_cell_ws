"""`writer.py` — the parts where a mistake costs data rather than accuracy."""

from __future__ import annotations

import json


def test_a_second_generate_run_merges_the_index_instead_of_truncating_it(tmp_path):
    """Seeds are not backfilled inside a run, so reaching a target count means a second range.

    Scene *directories* are written per scene; the index is written once at the end. Before
    this merged, a top-up run into the same `--out` left the directory holding every scene
    and the index listing only the second batch — and `scene_dirs` reads the index, so
    two-thirds of a corpus silently stopped existing for every downstream tool. Caught while
    topping the benchmark corpus up from 37 butt scenes to 50.
    """
    from weldgen.writer import write_index

    first = [{"emitted": True, "seed": s, "config_id": "abc"} for s in (1, 2, 3)]
    second = [{"emitted": True, "seed": s, "config_id": "abc"} for s in (7, 8)]
    write_index(tmp_path, first)
    write_index(tmp_path, second)

    rows = [json.loads(ln) for ln in (tmp_path / "index.jsonl").read_text().splitlines()]
    assert [r["seed"] for r in rows] == [1, 2, 3, 7, 8]


def test_rerunning_the_same_seed_range_replaces_its_rows_rather_than_doubling_them(tmp_path):
    """Merge is keyed on `(config_id, seed)`, so the index stays a function of what exists."""
    from weldgen.writer import write_index

    write_index(tmp_path, [{"emitted": True, "seed": 1, "config_id": "abc", "n": 1}])
    write_index(tmp_path, [{"emitted": True, "seed": 1, "config_id": "abc", "n": 2}])
    rows = [json.loads(ln) for ln in (tmp_path / "index.jsonl").read_text().splitlines()]
    assert len(rows) == 1 and rows[0]["n"] == 2

    # A different config in the same directory is a different corpus, and both survive.
    write_index(tmp_path, [{"emitted": True, "seed": 1, "config_id": "xyz", "n": 3}])
    rows = [json.loads(ln) for ln in (tmp_path / "index.jsonl").read_text().splitlines()]
    assert len(rows) == 2
