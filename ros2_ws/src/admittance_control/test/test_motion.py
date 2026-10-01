"""The shared TrajectoryExecutor (admittance_control/motion.py), without a robot.

Needs rclpy (a sourced ROS shell); skipped otherwise. Everything here runs in the dry
run, where no goal is sent: the gates (joint jump), the pretended position that
`where()` follows, the pretended contact, the force bias and trace, abort, and the
parameter prefix that lets the marking node ('') and the ICP node ('mv_') share it.
"""

from __future__ import annotations

import pathlib
import sys

import numpy as np
import pytest

rclpy = pytest.importorskip("rclpy")

PKG = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PKG))

from geometry_msgs.msg import WrenchStamped  # noqa: E402
from rclpy.node import Node  # noqa: E402

from admittance_control.motion import PARAMS, TrajectoryExecutor  # noqa: E402

Q0 = np.array([-0.34, -1.44, -1.56, -1.55, 1.94, -0.86])


@pytest.fixture(scope="module")
def ros():
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture
def ex(ros):
    node = Node("test_motion")
    said = []
    e = TrajectoryExecutor(node, prefix="t_", touch_force_n=1.5, say=said.append)
    e.said = said
    yield e
    node.destroy_node()


def _path(q_from, q_to, n=11, T=2.0):
    return [(q_from + s * (q_to - q_from), s * T) for s in np.linspace(0, 1, n)]


def _wrench(e, f):
    m = WrenchStamped()
    m.wrench.force.x, m.wrench.force.y, m.wrench.force.z = map(float, f)
    e._on_wrench(m)


def test_parameters_are_declared_with_the_prefix(ex):
    for name, default in PARAMS.items():
        assert ex._node.get_parameter("t_" + name).value == default
    assert ex.dry_run is True


def test_existing_parameters_are_kept(ros):
    node = Node("test_motion_existing")
    node.declare_parameter("dry_run", False)            # the owner declared it first
    e = TrajectoryExecutor(node)
    assert e.dry_run is False
    node.destroy_node()


def test_dry_run_moves_the_pretended_arm(ex):
    assert ex.where() is None
    ok, msg = ex.execute(_path(Q0, Q0), "x")
    assert not ok and "no joint states" in msg
    ex.set_sim(Q0)
    q1 = Q0 + 0.2
    ok, msg = ex.execute(_path(Q0, q1), "transit")
    assert ok and "dry run" in msg and ex.said and "transit" in ex.said[-1]
    assert np.allclose(ex.where(), q1)


def test_joint_jump_is_refused(ex):
    ex.set_sim(Q0)
    ok, msg = ex.execute(_path(Q0 + 0.5, Q0 + 0.6), "far")
    assert not ok and "refused" in msg
    assert np.allclose(ex.where(), Q0)                  # nothing pretended to move


def test_dry_run_contact_at_the_given_fraction(ex):
    ex.set_sim(Q0)
    timed = _path(Q0, Q0 + 0.1, n=11)
    ok, msg, contact = ex.execute(timed, "descent", watch_touch=True, want_contact=True,
                                  sim_contact_fraction=0.5)
    assert ok and contact is not None
    q_c, f = contact
    assert np.allclose(q_c, timed[5][0]) and f == pytest.approx(1.5)
    assert np.allclose(ex.where(), timed[5][0])


def test_force_bias_and_trace(ex):
    _wrench(ex, [1.0, -2.0, 3.0])                       # a constant offset (the tool's weight)
    ex.measure_bias(window_s=0.02)
    assert ex.force_mag() == pytest.approx(0.0, abs=1e-9)
    ex.start_trace()
    _wrench(ex, [1.0, -2.0, 5.0])
    _wrench(ex, [1.0, -2.0, 4.0])
    trace = ex.stop_trace()
    assert trace.shape == (2, 3) and np.allclose(trace[:, 2], [2.0, 1.0])
    assert ex.force_mag() == pytest.approx(1.0)
    assert np.allclose(ex.force_vector(), [0.0, 0.0, 1.0])
    assert ex.stop_trace().shape == (0, 3)              # tracing is off again


def test_abort_flag(ex):
    assert not ex.aborted
    ex.abort()                                           # no goal running: just the flag
    assert ex.aborted
    ex.clear_abort()
    assert not ex.aborted
