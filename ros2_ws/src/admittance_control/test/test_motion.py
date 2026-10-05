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


# ---------------------------------------------------------------- time_path (TOTG) -----
def _fake_totg(node, name):
    """A stand-in for totg_service_node: linear resampling of the waypoints at 0.1 rad/s."""
    from admittance_control.motion import ComputeTOTG

    def handle(req, res):
        n = req.num_joints
        q = np.asarray(req.waypoints_flat, float).reshape(-1, n)
        dense = [q[0]]
        for a, b in zip(q[:-1], q[1:]):
            k = max(1, int(np.ceil(np.abs(b - a).max() / 0.01)))
            dense += [a + (b - a) * (i / k) for i in range(1, k + 1)]
        res.timed_positions_flat = [float(v) for p in dense for v in p]
        res.timed_velocities_flat = [0.0] * (len(dense) * n)
        res.timestamps = [0.1 * i for i in range(len(dense))]
        res.num_output_points = len(dense)
        res.success = True
        return res
    return node.create_service(ComputeTOTG, name, handle)


@pytest.fixture
def totg_ex(ros):
    import threading

    from rclpy.executors import MultiThreadedExecutor
    from rclpy.parameter import Parameter
    from admittance_control.motion import ComputeTOTG
    if ComputeTOTG is None:
        pytest.skip("ComputeTOTG not built")
    server = Node("fake_totg")
    _fake_totg(server, "/test_fake_totg")
    client = Node("test_motion_totg",
                  parameter_overrides=[Parameter("t_totg_service", value="/test_fake_totg")])
    e = TrajectoryExecutor(client, prefix="t_", say=lambda s: None)
    exe = MultiThreadedExecutor(num_threads=2)
    exe.add_node(server); exe.add_node(client)
    th = threading.Thread(target=exe.spin, daemon=True); th.start()
    yield e
    exe.shutdown(); server.destroy_node(); client.destroy_node()


def test_time_path_uses_totg_when_it_answers(totg_ex):
    path = [Q0, Q0 + np.array([0.2, 0, 0, 0, 0, 0]), Q0 + np.array([0.2, 0.1, 0, 0, 0, 0])]
    timed, src = totg_ex.time_path(path, 0.3, 0.5, is_valid=lambda q: True)
    assert src == "TOTG"
    assert np.allclose(timed[0][0], path[0]) and np.allclose(timed[-1][0], path[-1])
    assert len(timed) >= 31 and timed[-1][1] == pytest.approx(0.1 * (len(timed) - 1))   # the server's samples


def test_time_path_falls_back_when_blending_leaves_free_space(totg_ex):
    path = [Q0, Q0 + np.array([0.2, 0, 0, 0, 0, 0])]
    timed, src = totg_ex.time_path(path, 0.3, 0.5, is_valid=lambda q: q[0] < Q0[0] + 0.1)
    assert src.startswith("trapezoid (TOTG blending left free space")
    from admittance_control.marking import time_trapezoid
    assert [t for _, t in timed] == [t for _, t in time_trapezoid(path, 0.3, 0.5)]


def test_time_path_without_the_service_is_a_trapezoid(ros):
    from rclpy.parameter import Parameter
    node = Node("test_motion_nototg",
                parameter_overrides=[Parameter("t_totg_service", value="/no_such_totg")])
    e = TrajectoryExecutor(node, prefix="t_", say=lambda s: None)
    timed, src = e.time_path([Q0, Q0 + 0.1], 0.3, 0.5)
    assert src.startswith("trapezoid") and timed[-1][1] > 0
    node.destroy_node()
