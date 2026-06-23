#!/usr/bin/env python3
# Proof of concept code. Not to be used in final project.
"""Move the 2AxisArm URDF in PyBullet."""

from __future__ import annotations

import argparse
import math
import sys
import time
from dataclasses import dataclass
from pathlib import Path

try:
    import pybullet as p
except ImportError:
    print("PyBullet is not installed. Install it with: python -m pip install pybullet", file=sys.stderr)
    raise SystemExit(1)


SCRIPT_DIR = Path(__file__).resolve().parent
DEFAULT_URDF = SCRIPT_DIR / "2AxisArm_pybullet.urdf"
CONTROLLED_JOINTS = ("joint0_yaw", "joint1_shoulder", "joint2_elbow")
TOOL_LINK = "tool0"


@dataclass(frozen=True)
class Joint:
    index: int
    name: str
    lower: float
    upper: float
    max_force: float
    max_velocity: float


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Load 2AxisArm.urdf in PyBullet and command yaw, shoulder, and elbow joints."
    )
    parser.add_argument("--urdf", type=Path, default=DEFAULT_URDF, help="Path to the 2AxisArm URDF.")
    parser.add_argument("--direct", action="store_true", help="Use PyBullet DIRECT mode instead of the GUI.")
    parser.add_argument("--demo", action="store_true", help="Run a scripted joint-space sweep.")
    parser.add_argument("--duration", type=float, default=10.0, help="Demo duration in seconds.")
    parser.add_argument("--time-step", type=float, default=1.0 / 240.0, help="Simulation time step in seconds.")
    parser.add_argument("--gravity", type=float, default=0.0, help="Z gravity in m/s^2. Defaults to 0 for a stable visual demo.")
    parser.add_argument("--no-marker", action="store_true", help="Disable the red tool0 marker body.")
    parser.add_argument("--marker-radius", type=float, default=0.018, help="Radius of the red tool0 marker body.")
    parser.add_argument("--yaw", type=float, default=0.0, help="Initial yaw target in radians.")
    parser.add_argument("--shoulder", type=float, default=0.0, help="Initial shoulder target in radians.")
    parser.add_argument("--elbow", type=float, default=0.0, help="Initial elbow target in radians.")
    return parser.parse_args()


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def finite_limits(lower: float, upper: float) -> tuple[float, float]:
    if lower > upper or not math.isfinite(lower) or not math.isfinite(upper):
        return -math.pi, math.pi
    return lower, upper


def load_robot(urdf_path: Path) -> int:
    resolved_urdf = urdf_path.resolve()
    if not resolved_urdf.exists():
        raise FileNotFoundError(f"URDF not found: {resolved_urdf}")

    p.setAdditionalSearchPath(str(resolved_urdf.parent))
    flags = p.URDF_USE_INERTIA_FROM_FILE | p.URDF_MAINTAIN_LINK_ORDER
    return p.loadURDF(str(resolved_urdf), useFixedBase=True, flags=flags)


def collect_joints(robot_id: int) -> tuple[dict[str, Joint], dict[str, int]]:
    joints: dict[str, Joint] = {}
    links: dict[str, int] = {}

    for index in range(p.getNumJoints(robot_id)):
        info = p.getJointInfo(robot_id, index)
        name = info[1].decode("utf-8")
        joint_type = info[2]
        lower, upper = finite_limits(float(info[8]), float(info[9]))
        max_force = float(info[10]) if info[10] > 0 else 10.0
        max_velocity = float(info[11]) if info[11] > 0 else 1.0
        link_name = info[12].decode("utf-8")

        links[link_name] = index
        if joint_type in (p.JOINT_REVOLUTE, p.JOINT_PRISMATIC):
            joints[name] = Joint(index, name, lower, upper, max_force, max_velocity)

    missing = [name for name in CONTROLLED_JOINTS if name not in joints]
    if missing:
        available = ", ".join(sorted(joints)) or "none"
        raise RuntimeError(f"Missing controlled joint(s): {', '.join(missing)}. Available movable joints: {available}")

    if TOOL_LINK not in links:
        available_links = ", ".join(sorted(links)) or "none"
        raise RuntimeError(f"Missing tool link '{TOOL_LINK}'. Available child links: {available_links}")

    return joints, links


def print_robot_summary(robot_id: int, joints: dict[str, Joint], links: dict[str, int]) -> None:
    print(f"Loaded robot with {p.getNumJoints(robot_id)} joints.")
    for name in CONTROLLED_JOINTS:
        joint = joints[name]
        print(
            f"  {name}: index={joint.index}, "
            f"limit=[{joint.lower:.4f}, {joint.upper:.4f}], "
            f"force={joint.max_force:.2f}, velocity={joint.max_velocity:.2f}"
        )
    print(f"  {TOOL_LINK}: link_index={links[TOOL_LINK]}")


def command_positions(robot_id: int, joints: dict[str, Joint], targets: dict[str, float]) -> dict[str, float]:
    ordered_joints = [joints[name] for name in CONTROLLED_JOINTS]
    clamped_targets = {
        joint.name: clamp(float(targets[joint.name]), joint.lower, joint.upper) for joint in ordered_joints
    }

    p.setJointMotorControlArray(
        robot_id,
        jointIndices=[joint.index for joint in ordered_joints],
        controlMode=p.POSITION_CONTROL,
        targetPositions=[clamped_targets[joint.name] for joint in ordered_joints],
        forces=[joint.max_force for joint in ordered_joints],
        targetVelocities=[0.0 for _ in ordered_joints],
        positionGains=[0.08 for _ in ordered_joints],
        velocityGains=[1.0 for _ in ordered_joints],
    )
    return clamped_targets


def create_sliders(joints: dict[str, Joint], initial_targets: dict[str, float]) -> dict[str, int]:
    sliders: dict[str, int] = {}
    for name in CONTROLLED_JOINTS:
        joint = joints[name]
        sliders[name] = p.addUserDebugParameter(
            name,
            joint.lower,
            joint.upper,
            clamp(initial_targets[name], joint.lower, joint.upper),
        )
    return sliders


def read_sliders(sliders: dict[str, int]) -> dict[str, float]:
    return {name: p.readUserDebugParameter(slider_id) for name, slider_id in sliders.items()}


def demo_targets(t: float, joints: dict[str, Joint]) -> dict[str, float]:
    phases = {
        "joint0_yaw": 0.0,
        "joint1_shoulder": 0.8,
        "joint2_elbow": 1.6,
    }
    amplitudes = {
        "joint0_yaw": 0.55,
        "joint1_shoulder": 0.45,
        "joint2_elbow": 0.50,
    }

    targets: dict[str, float] = {}
    for name in CONTROLLED_JOINTS:
        joint = joints[name]
        midpoint = 0.5 * (joint.lower + joint.upper)
        half_range = 0.5 * (joint.upper - joint.lower)
        amplitude = min(amplitudes[name], half_range * 0.75)
        targets[name] = midpoint + amplitude * math.sin((2.0 * math.pi * 0.2 * t) + phases[name])
    return targets


def tool_position(robot_id: int, tool_link_index: int) -> tuple[float, float, float]:
    state = p.getLinkState(robot_id, tool_link_index, computeForwardKinematics=True)
    return tuple(float(value) for value in state[4])


def create_tool_marker(position: tuple[float, float, float], radius: float) -> int:
    visual_id = p.createVisualShape(
        p.GEOM_SPHERE,
        radius=radius,
        rgbaColor=(1.0, 0.05, 0.05, 1.0),
    )
    return p.createMultiBody(
        baseMass=0.0,
        baseCollisionShapeIndex=-1,
        baseVisualShapeIndex=visual_id,
        basePosition=position,
    )


def update_tool_marker(marker_id: int, position: tuple[float, float, float]) -> None:
    if marker_id >= 0:
        p.resetBasePositionAndOrientation(marker_id, position, (0.0, 0.0, 0.0, 1.0))


def configure_debug_camera() -> None:
    p.resetDebugVisualizerCamera(
        cameraDistance=0.85,
        cameraYaw=45.0,
        cameraPitch=-30.0,
        cameraTargetPosition=(0.2, 0.0, 0.15),
    )


def run() -> int:
    args = parse_args()
    connection_mode = p.DIRECT if args.direct else p.GUI
    client_id = p.connect(connection_mode)
    if client_id < 0:
        raise RuntimeError("Could not connect to PyBullet.")

    try:
        p.setTimeStep(args.time_step)
        p.setGravity(0.0, 0.0, args.gravity)
        p.configureDebugVisualizer(p.COV_ENABLE_GUI, 1)
        p.configureDebugVisualizer(p.COV_ENABLE_RGB_BUFFER_PREVIEW, 0)
        p.configureDebugVisualizer(p.COV_ENABLE_DEPTH_BUFFER_PREVIEW, 0)
        p.configureDebugVisualizer(p.COV_ENABLE_SEGMENTATION_MARK_PREVIEW, 0)

        robot_id = load_robot(args.urdf)
        joints, links = collect_joints(robot_id)
        print_robot_summary(robot_id, joints, links)

        initial_targets = {
            "joint0_yaw": args.yaw,
            "joint1_shoulder": args.shoulder,
            "joint2_elbow": args.elbow,
        }
        command_positions(robot_id, joints, initial_targets)

        if not args.direct:
            configure_debug_camera()

        sliders = {} if args.direct or args.demo else create_sliders(joints, initial_targets)
        tool_link_index = links[TOOL_LINK]
        marker_id = -1 if args.no_marker else create_tool_marker(tool_position(robot_id, tool_link_index), args.marker_radius)

        start_time = time.monotonic()
        last_print = start_time

        while True:
            elapsed = time.monotonic() - start_time
            if args.demo:
                targets = demo_targets(elapsed, joints)
            elif sliders:
                targets = read_sliders(sliders)
            else:
                targets = initial_targets

            clamped_targets = command_positions(robot_id, joints, targets)
            p.stepSimulation()
            update_tool_marker(marker_id, tool_position(robot_id, tool_link_index))

            now = time.monotonic()
            if now - last_print >= 1.0:
                tool = tool_position(robot_id, tool_link_index)
                print(
                    "targets="
                    + ", ".join(f"{name}={clamped_targets[name]:.3f}" for name in CONTROLLED_JOINTS)
                    + f" tool0=({tool[0]:.3f}, {tool[1]:.3f}, {tool[2]:.3f})"
                )
                last_print = now

            if args.direct or args.demo:
                if elapsed >= args.duration:
                    break
            time.sleep(args.time_step)

        final_tool = tool_position(robot_id, tool_link_index)
        print(f"Final {TOOL_LINK} position: ({final_tool[0]:.4f}, {final_tool[1]:.4f}, {final_tool[2]:.4f})")
    finally:
        p.disconnect()

    return 0


if __name__ == "__main__":
    raise SystemExit(run())
