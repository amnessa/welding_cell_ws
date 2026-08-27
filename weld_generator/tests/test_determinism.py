"""The Phase 1 gate (D15) — and the RNG-substream property it depends on.

If this file fails, everything downstream is unreproducible and the
release-as-a-program argument collapses. It is the one gate that, if it silently
breaks, quietly invalidates the release months later.
"""

from __future__ import annotations

import json
import subprocess
import sys
from pathlib import Path

import numpy as np
import pytest

from weldgen.config import geometry_config, load_config
from weldgen.hashing import content_hash
from weldgen.scene import generate_scene

ROOT = Path(__file__).resolve().parents[1]
SEEDS = [8412337, 99, 123456789]


@pytest.fixture(scope="module")
def cfg():
    c = load_config(str(ROOT / "configs" / "smoke.yaml"))
    # D31 acceptance is exercised in test_class_disjointness; disabled here because the
    # canonical seed 8412337 is (correctly) a class-boundary rejection under it, and a
    # rejected seed has no arrays to hash.
    c["class_disjoint"] = False
    return c


@pytest.mark.parametrize("seed", SEEDS)
def test_same_process_repeatable(cfg, seed):
    a_s, a_a = generate_scene(cfg, seed)
    b_s, b_a = generate_scene(cfg, seed)
    assert content_hash(a_s, a_a) == content_hash(b_s, b_a)


def test_separate_processes_agree(cfg):
    """The real gate: a fresh interpreter must reproduce the same hashes.

    Catches anything that leaks process state into the output — a global RNG, dict
    ordering, a cached module-level array.
    """
    code = (
        "from weldgen.config import load_config;"
        "from weldgen.scene import generate_scene;"
        "from weldgen.hashing import content_hash;"
        f"cfg=load_config({str(ROOT / 'configs' / 'smoke.yaml')!r});"
        "cfg['class_disjoint']=False;"
        f"print([content_hash(*generate_scene(cfg,s)) for s in {SEEDS}])"
    )
    runs = [
        subprocess.run([sys.executable, "-c", code], cwd=ROOT,
                       capture_output=True, text=True, check=True).stdout
        for _ in range(2)
    ]
    assert runs[0] == runs[1]
    assert len(json.loads(runs[0].replace("'", '"'))) == len(SEEDS)


def test_provenance_is_excluded_from_the_hash(cfg):
    """Re-running next month must not 'fail' a gate it actually passes."""
    scene, arrays = generate_scene(cfg, SEEDS[0])
    before = content_hash(scene, arrays)
    scene["provenance"]["created_utc"] = "1999-01-01T00:00:00Z"
    scene["provenance"]["git_commit"] = "deadbeef"
    assert content_hash(scene, arrays) == before


def test_hash_is_sensitive_to_geometry(cfg):
    """A hash that never changes is not a hash. Perturb one array, expect a new digest."""
    scene, arrays = generate_scene(cfg, SEEDS[0])
    before = content_hash(scene, arrays)
    arrays = dict(arrays)
    xyz = arrays["cloud.npz:xyz"].copy()
    xyz[0, 0] += np.float32(1e-3)
    arrays["cloud.npz:xyz"] = xyz
    assert content_hash(scene, arrays) != before


def test_hash_is_sensitive_to_dtype(cfg):
    """dtype is hashed explicitly, so a silent float32->float64 widening is caught."""
    scene, arrays = generate_scene(cfg, SEEDS[0])
    before = content_hash(scene, arrays)
    arrays = dict(arrays)
    arrays["cloud.npz:xyz"] = arrays["cloud.npz:xyz"].astype(np.float64)
    assert content_hash(scene, arrays) != before


def test_npz_roundtrip_preserves_content_hash(cfg, tmp_path):
    """np.savez writes a zip whose entries embed a timestamp, so the file is NOT
    byte-reproducible. The content hash must survive the round trip anyway — that is
    the whole reason D15 is defined on content rather than on bytes.
    """
    from weldgen.writer import split_arrays, write_scene

    scene, arrays = generate_scene(cfg, SEEDS[0])
    want = content_hash(scene, arrays)
    d = write_scene(tmp_path, scene, arrays)

    reloaded = {}
    for npz in sorted(d.glob("*.npz")):
        with np.load(npz) as z:
            for k in z.files:
                reloaded[f"{npz.name}:{k}"] = z[k]
    got = content_hash(json.loads((d / "scene.json").read_text()), reloaded)
    assert got == want == (d / "scene.sha256").read_text().strip()


def test_geometry_substreams_are_isolated(cfg):
    """SCHEMA.md §6.1/§6.4: substreams 0-2 fix geometry, 3-6 are ablation axes.

    Changing a sensor or density setting must not disturb one bit of seam truth. This is
    what lets Phase 3 add camera sampling without regenerating Phase 1-2 geometry.
    """
    other = dict(cfg)
    other["sensor_profiles"] = ["stereo_poor"]
    other["density_per_mm2"] = [3.0, 3.0]

    for seed in SEEDS:
        a, _ = generate_scene(cfg, seed)
        b, _ = generate_scene(other, seed)
        assert a["twin_key"] == b["twin_key"]
        assert geometry_config(cfg) == geometry_config(other)
        assert a["objects"] == b["objects"]
        assert [s["parametric"] for s in a["seams"]] == [s["parametric"] for s in b["seams"]]
        assert a["fit"] == b["fit"]
        # ...while the ablation axis genuinely moved.
        assert a["cloud"]["density_per_mm2"] != b["cloud"]["density_per_mm2"]


def test_twin_key_changes_with_geometry(cfg):
    """A twin_key that never changes would silently join unrelated scenes."""
    other = dict(cfg)
    other["thickness_mm"] = [7.0, 7.0]
    a, _ = generate_scene(cfg, SEEDS[0])
    b, _ = generate_scene(other, SEEDS[0])
    assert a["twin_key"] != b["twin_key"]


def test_hash_is_byte_order_independent():
    """The gate claims "separate processes, any machine", so answer for byte order.

    `str(np.dtype("float64"))` is `"float64"` on a little- and a big-endian machine alike,
    so the dtype tag would match while `tobytes()` did not: the hash would silently differ
    across architectures while the gate claimed to hold. Arrays are byte-swapped to
    little-endian before hashing. Almost nothing runs big-endian any more - but a claim
    that invites the question should answer it.
    """
    import numpy as np

    from weldgen.hashing import content_hash

    little = np.arange(12, dtype="<f4").reshape(4, 3)
    big = little.astype(">f4")
    assert np.array_equal(little, big)
    assert content_hash({}, {"cloud.npz:xyz": little}) \
        == content_hash({}, {"cloud.npz:xyz": big})

    # ...and the dtype must still be distinguished, or the guard has gone too far.
    assert content_hash({}, {"cloud.npz:xyz": little}) \
        != content_hash({}, {"cloud.npz:xyz": little.astype("<f8")})
