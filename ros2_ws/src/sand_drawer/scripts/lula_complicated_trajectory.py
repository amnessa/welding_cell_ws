#!/usr/bin/env python3

import argparse
import json
import os
from typing import List

import numpy as np


def _load_plane_points(plane_json_path: str) -> List[np.ndarray]:
    with open(plane_json_path, "r", encoding="utf-8") as handle:
        payload = json.load(handle)

    if "projected_vector_trajectory" in payload and len(payload["projected_vector_trajectory"]) >= 4:
        return [np.array(item["position"], dtype=float) for item in payload["projected_vector_trajectory"]]

    if "square_trajectory" in payload and len(payload["square_trajectory"]) >= 4:
        return [np.array(item["position"], dtype=float) for item in payload["square_trajectory"]]

    raise RuntimeError(
        "Plane JSON must contain projected_vector_trajectory or square_trajectory with at least 4 points."
    )


def _default_ur5e_config_paths():
    from isaacsim.core.utils.extensions import get_extension_path_from_name

    mg_extension_path = get_extension_path_from_name("isaacsim.robot_motion.motion_generation")
    rmp_config_dir = os.path.join(mg_extension_path, "motion_policy_configs", "universal_robots", "ur5e")

    robot_description_path = os.path.join(rmp_config_dir, "rmpflow", "ur5e_robot_description.yaml")
    urdf_path = os.path.join(rmp_config_dir, "ur5e_robot.urdf")

    return robot_description_path, urdf_path


def _build_complicated_path_spec(lula, points: List[np.ndarray], keep_orientation: bool = True):
    if len(points) < 4:
        raise ValueError("Need at least 4 points to build a complicated path spec.")

    # Use a tool-down pose convention around +X axis.
    # You can later replace with a quaternion-derived Rotation3 if desired.
    r_tool = lula.Rotation3(np.pi, np.array([1.0, 0.0, 0.0]))

    t0 = points[0]
    t1 = points[1]
    t2 = points[2]
    t3 = points[3]

    task_space_spec = lula.create_task_space_path_spec(lula.Pose3(r_tool, t0))

    # 1) Linear segment
    task_space_spec.add_linear_path(lula.Pose3(r_tool, t1))

    # 2) Three-point arc from t1 -> t2 via midpoint
    midpoint12 = 0.5 * (t1 + t2) + np.array([0.0, 0.0, 0.02])
    task_space_spec.add_three_point_arc(t2, midpoint12, constant_orientation=keep_orientation)

    # 3) Tangent arc from t2 -> t3
    task_space_spec.add_tangent_arc(t3, constant_orientation=keep_orientation)

    # 4) Return to start with orientation target
    task_space_spec.add_linear_path(lula.Pose3(r_tool, t0))

    return task_space_spec


def generate_complicated_trajectory(
    plane_json_path: str,
    end_effector_frame: str,
    robot_description_path: str,
    urdf_path: str,
):
    import lula
    from isaacsim.robot_motion.motion_generation import LulaTaskSpaceTrajectoryGenerator

    points = _load_plane_points(plane_json_path)

    taskspace_generator = LulaTaskSpaceTrajectoryGenerator(
        robot_description_path=robot_description_path,
        urdf_path=urdf_path,
    )

    path_spec = _build_complicated_path_spec(lula, points)

    # Composite path lets us later prepend/append C-space sections.
    # Start from neutral joint guess; update this if you want a known start pose.
    initial_cspace = np.zeros(6)
    composite_spec = lula.create_composite_path_spec(initial_cspace)
    transition_mode = lula.CompositePathSpec.TransitionMode.FREE
    composite_spec.add_task_space_path_spec(path_spec, transition_mode)

    trajectory = taskspace_generator.compute_task_space_trajectory_from_path_spec(
        composite_spec,
        end_effector_frame,
    )

    return trajectory


def maybe_execute_on_articulation(trajectory, robot_prim_path: str, physics_dt: float):
    from isaacsim.core.prims import Articulation
    from isaacsim.robot_motion.motion_generation import ArticulationTrajectory

    articulation = Articulation(robot_prim_path)
    articulation.initialize()

    articulation_trajectory = ArticulationTrajectory(articulation, trajectory, physics_dt)
    actions = articulation_trajectory.get_action_sequence()

    # Teleport to first action before stepping
    if actions:
        initial_positions = np.zeros(articulation.num_dof)
        initial_positions[actions[0].joint_indices] = actions[0].joint_positions
        articulation.set_joint_positions(initial_positions)
        articulation.set_joint_velocities(np.zeros_like(initial_positions))

    return articulation, actions


def main():
    parser = argparse.ArgumentParser(
        description="Generate complicated Lula task-space trajectories from sand_drawer plane output."
    )
    parser.add_argument("--plane-json", default="/tmp/sand_drawer_plane.json")
    parser.add_argument("--end-effector-frame", default="tool0")
    parser.add_argument("--robot-description", default="")
    parser.add_argument("--urdf", default="")
    parser.add_argument("--robot-prim-path", default="")
    parser.add_argument("--physics-dt", type=float, default=1.0 / 60.0)
    parser.add_argument("--execute", action="store_true")
    args = parser.parse_args()

    if not args.robot_description or not args.urdf:
        robot_description_path, urdf_path = _default_ur5e_config_paths()
    else:
        robot_description_path, urdf_path = args.robot_description, args.urdf

    trajectory = generate_complicated_trajectory(
        plane_json_path=args.plane_json,
        end_effector_frame=args.end_effector_frame,
        robot_description_path=robot_description_path,
        urdf_path=urdf_path,
    )

    if trajectory is None:
        print("[sand_drawer] No trajectory could be computed from the given path spec.")
        return

    print("[sand_drawer] Lula complicated trajectory generated successfully.")
    print(f"[sand_drawer] robot_description: {robot_description_path}")
    print(f"[sand_drawer] urdf: {urdf_path}")

    if args.execute:
        if not args.robot_prim_path:
            raise RuntimeError("--execute requires --robot-prim-path")

        articulation, actions = maybe_execute_on_articulation(
            trajectory=trajectory,
            robot_prim_path=args.robot_prim_path,
            physics_dt=args.physics_dt,
        )
        print(f"[sand_drawer] Generated {len(actions)} articulation actions for {args.robot_prim_path}.")
        print("[sand_drawer] Apply actions in your simulation update loop.")


if __name__ == "__main__":
    main()
