"""The Phase 5 annotation toolchain, validated the same way the harness was: with a fake
annotator of KNOWN error, before any real click flows through.

If the scorer cannot recover an injected 0,8 mm lateral sigma, a planted phantom seam and
a planted miss, no number it produces about Anil can be trusted.
"""

from __future__ import annotations

import json
import pathlib
import subprocess
import sys

import numpy as np
import pytest

ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))


@pytest.fixture(scope="module")
def export(tmp_path_factory):
    tmp = tmp_path_factory.mktemp("annot")
    r = subprocess.run(
        [sys.executable, str(ROOT / "scripts" / "annotation" / "export_for_annotation.py"),
         "--out", str(tmp / "export"), "--manifest", str(tmp / "manifest.json"),
         "--per-type", "1", "--seed", "0"],
        capture_output=True, text=True)
    if "corpus is not balanced" in r.stderr or r.returncode != 0:
        pytest.skip(f"export failed (no corpus?): {r.stderr[-200:]}")
    return tmp


def test_the_export_ships_no_truth_and_the_manifest_stays_outside(export):
    """The annotator's directory must contain clouds and the briefing - nothing else.

    Contamination by packaging is the failure this pins: a seams file, a scene.json or the
    manifest inside the export directory would hand the annotator the answer.
    """
    files = sorted(p.name for p in (export / "export").iterdir())
    assert "BRIEFING.md" in files
    assert all(f.endswith(".ply") or f == "BRIEFING.md" for f in files)
    assert (export / "manifest.json").exists()
    manifest = json.loads((export / "manifest.json").read_text())
    assert len(manifest) == 5
    # anonymous ids leak nothing about scene identity or joint type
    for anon in manifest:
        assert anon.startswith("scene_") and anon[6:].isdigit()


def test_a_fake_annotator_of_known_error_is_recovered(export):
    """Clicks = truth resampled at ~25 mm + N(0, 0,8 mm), one seam skipped, one invented.

    The scorer must report: lateral RMSE ~= the perpendicular component of the injected
    noise, exactly one miss, exactly one extra - and keep selection and localization
    apart (the phantom must not degrade the localization figure).
    """
    from baselines import ground_truth, load_scene
    from baselines.harness import _coarsen

    manifest = json.loads((export / "manifest.json").read_text())
    ann = export / "annotations" / "fake"
    ann.mkdir(parents=True)
    rng = np.random.default_rng(0)
    sigma = 0.8
    planted_miss = planted_extra = False
    for anon, meta in sorted(manifest.items()):
        scene, arrays = load_scene(meta["path"])
        gt = ground_truth(scene, arrays, primary_only=True)
        k = 0
        for gi, g in enumerate(gt):
            if not planted_miss and len(gt) > 1:
                planted_miss = True                    # skip one real seam, once
                continue
            clicks = _coarsen(np.asarray(g, float), 25.0)
            clicks = clicks + rng.normal(0, sigma, clicks.shape)
            np.savetxt(ann / f"{anon}_seam{k}.txt",
                       np.column_stack([np.arange(len(clicks)), clicks]),
                       fmt="%.4f", delimiter=",")
            k += 1
        if not planted_extra:
            planted_extra = True                       # invent one seam, once
            lo = np.asarray(gt[0], float).min(axis=0)
            fake = np.stack([lo + [0, 0, 60.0], lo + [80.0, 0, 60.0]])
            np.savetxt(ann / f"{anon}_seam{k}.txt",
                       np.column_stack([np.arange(2), fake]), fmt="%.4f", delimiter=",")

    r = subprocess.run(
        [sys.executable, str(ROOT / "scripts" / "annotation" / "score_annotations.py"),
         "--manifest", str(export / "manifest.json"),
         "--annotations", str(ann), "--role", "briefed",
         "--out", str(export / "scored")],
        capture_output=True, text=True)
    assert r.returncode == 0, r.stderr[-400:]

    import pandas as pd
    df = pd.read_csv(export / "scored" / "annotation_scores.csv")
    assert df.missed.sum() == 1
    assert df.extra.sum() == 1
    # Rayleigh mean of the perpendicular component ~= sigma * sqrt(pi/2) ~= 1,0 mm;
    # chord interpolation pulls it down. The band catches unit and factor-2 errors.
    lat = df.lat_rmse.median()
    assert 0.5 < lat < 1.6, f"injected 0,8 mm came back as {lat:.2f}"

    model = json.loads((export / "scored" / "annotator_model.json").read_text())
    assert "fake" in model and model["fake"]["role"] == "briefed"
    assert model["fake"]["miss_rate"] > 0 and model["fake"]["extra_rate"] > 0


def test_the_demo_role_is_excluded_from_the_headline(export):
    """The contamination rule, enforced in code: a demo-only scoring run produces no
    headline aggregate and says to recruit the real annotator."""
    ann = export / "annotations" / "author_demo"
    ann.mkdir(parents=True, exist_ok=True)
    manifest = json.loads((export / "manifest.json").read_text())
    anon, meta = sorted(manifest.items())[0]
    from baselines import ground_truth, load_scene
    scene, arrays = load_scene(meta["path"])
    g = np.asarray(ground_truth(scene, arrays, primary_only=True)[0], float)
    np.savetxt(ann / f"{anon}_seam0.txt",
               np.column_stack([np.arange(2), g[[0, -1]]]), fmt="%.4f", delimiter=",")

    r = subprocess.run(
        [sys.executable, str(ROOT / "scripts" / "annotation" / "score_annotations.py"),
         "--manifest", str(export / "manifest.json"),
         "--annotations", str(ann), "--role", "demo",
         "--out", str(export / "scored_demo")],
        capture_output=True, text=True)
    assert r.returncode == 0, r.stderr[-400:]
    assert "only demo annotations present" in r.stdout
    assert "recruit" in r.stdout
