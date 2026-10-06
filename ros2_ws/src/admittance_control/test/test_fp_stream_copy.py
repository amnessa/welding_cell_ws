"""fp_stream.py is the wire protocol between fp_tracker_node (here, as
admittance_control/fp_stream.py) and the FoundationPose container's tracking server
(scripts/scripts_in_foundationpose/fp_stream.py, the mirror of the host's file). Both
ends must speak the same format, so the two copies must stay identical."""

import pathlib

PKG = pathlib.Path(__file__).resolve().parents[1]


def test_fp_stream_copies_are_identical():
    ours = (PKG / "admittance_control" / "fp_stream.py").read_bytes()
    mirror = (PKG / "scripts" / "scripts_in_foundationpose" / "fp_stream.py").read_bytes()
    assert ours == mirror, ("fp_stream.py differs from the FoundationPose mirror: copy the "
                            "newer one over the other (the tracker and the node must agree)")
