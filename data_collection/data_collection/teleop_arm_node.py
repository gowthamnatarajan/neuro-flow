#!/usr/bin/env python3
"""Cartesian-space keyboard teleoperation for the JetAuto Pro arm.

Keys move the end-effector in X/Y/Z. PyKDL (already installed with ROS 2)
solves the inverse kinematics to find the required joint angles, which are
then converted to servo pulse commands and sent to the robot.

Key layout
----------
  i / k  : EE forward / back   (+X / −X)
  j / l  : EE left  / right    (+Y / −Y)
  u / o  : EE up    / down     (+Z / −Z)
  n / m  : wrist roll ±step    (joint5 direct)
  p / ;  : gripper close / open
  SPACE  : toggle recording
  ENTER  : save episode
  x      : discard episode
  Ctrl-C : exit
"""

import select
import sys
import termios
import threading
import tty

import numpy as np
import PyKDL as kdl
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile
from ros_robot_controller_msgs.msg import ServoPosition, ServosPosition
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray, String

# ── Arm geometry (from jetauto_description URDF) ─────────────────────────────
# Link lengths along Z between consecutive joints
L1 = 0.0339   # joint1 → joint2 (servo holder height)
L2 = 0.1294   # joint2 → joint3 (upper arm)
L3 = 0.1294   # joint3 → joint4 (forearm)
L4 = 0.0545   # joint4 → joint5 (wrist holder)
L5 = 0.0800   # joint5 → end-effector

JOINT_LIMITS = 2.09   # ±2.09 rad (±120°) for all 5 arm joints

# ── Servo calibration ────────────────────────────────────────────────────────
# Hiwonder LX servos: pulse 0–1000 spans 240° (±120° around centre 500)
PULSE_PER_RAD = 500.0 / JOINT_LIMITS   # ≈ 238.7 pulses per radian
SERVO_IDS = [1, 2, 3, 4, 5]            # servo IDs for joint1-5
GRIPPER_ID = 10                         # confirmed from robot: ID 10 = r_joint
GRIPPER_OPEN_PULSE  = 200
GRIPPER_CLOSE_PULSE = 580   # partial close — leaves ~2 cm gap
GRIPPER_DEFAULT_PULSE = 500

# servo_controller.yaml has min:1000 max:0 for all joints, so:
#   q_robot = -(pulse - 500) * (2.09/500)  →  JOINT_SIGN must be -1
JOINT_SIGN = [-1, -1, -1, -1, -1]

# ── Teleoperation parameters ──────────────────────────────────────────────────
EE_STEP       = 0.010   # metres per keypress
ROTATION_STEP = 0.05    # radians per keypress for EE orientation
HEARTBEAT_HZ  = 25
SAFE_PULSE_MIN = 100
SAFE_PULSE_MAX = 900

HELP = """
╔══════════════════════════════════════════╗
║   JetAuto 6-DOF Arm Teleop               ║
╠══════════════════════════════════════════╣
║  Translation                             ║
║   W / S   Forward / Back   (+X / −X)     ║
║   A / D   Yaw left / right (joint1)      ║
║   Q / E   Up      / Down   (+Z / −Z)     ║
╠══════════════════════════════════════════╣
║  Wrist (direct joint control)            ║
║   I / K   joint4 pitch  +/−              ║
║   J / L   joint5 roll   +/−              ║
╠══════════════════════════════════════════╣
║  SPACE    Gripper toggle (open ↔ close)  ║
║  R        Toggle recording               ║
║  ENTER    Save episode                   ║
║  X        Discard episode                ║
║  Ctrl-C   Exit                           ║
╚══════════════════════════════════════════╝
"""


# ── KDL chain & solvers ───────────────────────────────────────────────────────

def _build_chain() -> kdl.Chain:
    """Build the JetAuto arm kinematic chain from known URDF geometry."""
    chain = kdl.Chain()
    chain.addSegment(kdl.Segment(
        kdl.Joint("j1", kdl.Vector(0, 0, 0), kdl.Vector(0, 0, -1), kdl.Joint.RotAxis),
        kdl.Frame(kdl.Vector(0, 0, L1))))
    chain.addSegment(kdl.Segment(
        kdl.Joint("j2", kdl.Vector(0, 0, 0), kdl.Vector(0, 1, 0), kdl.Joint.RotAxis),
        kdl.Frame(kdl.Vector(0, 0, L2))))
    chain.addSegment(kdl.Segment(
        kdl.Joint("j3", kdl.Vector(0, 0, 0), kdl.Vector(0, 1, 0), kdl.Joint.RotAxis),
        kdl.Frame(kdl.Vector(0, 0, L3))))
    chain.addSegment(kdl.Segment(
        kdl.Joint("j4", kdl.Vector(0, 0, 0), kdl.Vector(0, 1, 0), kdl.Joint.RotAxis),
        kdl.Frame(kdl.Vector(0, 0, L4))))
    chain.addSegment(kdl.Segment(
        kdl.Joint("j5", kdl.Vector(0, 0, 0), kdl.Vector(0, 0, -1), kdl.Joint.RotAxis),
        kdl.Frame(kdl.Vector(0, 0, L5))))
    return chain


_CHAIN = _build_chain()
_FK_SOLVER = kdl.ChainFkSolverPos_recursive(_CHAIN)
_JAC_SOLVER = kdl.ChainJntToJacSolver(_CHAIN)


def _jnt(q_list: list) -> kdl.JntArray:
    a = kdl.JntArray(5)
    for i, v in enumerate(q_list):
        a[i] = v
    return a


def fk(q: list) -> kdl.Frame:
    """Forward kinematics: joint angles (rad) → end-effector KDL Frame."""
    frame = kdl.Frame()
    _FK_SOLVER.JntToCart(_jnt(q), frame)
    return frame


_DLS_LAMBDA = 0.05   # damping factor; prevents blowup near singularities
_MAX_DQ     = 0.20   # max joint delta per keypress (rad) — safety clamp

def ik_step(q: list, twist: list) -> list:
    """DLS IK step using the full 6×5 Jacobian.

    twist: [vx, vy, vz, wx, wy, wz] — set unused components to 0.
    DLS: dq = J^T (J J^T + λ²I)⁻¹ × twist
    """
    jac = kdl.Jacobian(5)
    _JAC_SOLVER.JntToJac(_jnt(q), jac)

    J = np.array([[jac[r, c] for c in range(5)] for r in range(6)])

    lam2 = _DLS_LAMBDA ** 2
    J_pinv = J.T @ np.linalg.inv(J @ J.T + lam2 * np.eye(6))
    dq = J_pinv @ np.array(twist)

    max_step = np.max(np.abs(dq))
    if max_step > _MAX_DQ:
        dq = dq * (_MAX_DQ / max_step)

    return [
        max(-JOINT_LIMITS, min(JOINT_LIMITS, q[i] + dq[i]))
        for i in range(5)
    ]


# ── Conversion helpers ────────────────────────────────────────────────────────

def angle_to_pulse(q_rad: float, joint_idx: int) -> int:
    pulse = 500.0 + JOINT_SIGN[joint_idx] * q_rad * PULSE_PER_RAD
    return int(max(SAFE_PULSE_MIN, min(SAFE_PULSE_MAX, pulse)))


# ── ROS 2 node ────────────────────────────────────────────────────────────────

JOINT_NAMES = ["joint1", "joint2", "joint3", "joint4", "joint5"]


class TeleopArmNode(Node):
    def __init__(self):
        super().__init__("teleop_arm_node")

        self._gripper_pulse = GRIPPER_DEFAULT_PULSE
        self._recording = False
        self._dirty = False   # True only after a key changes state

        qos = QoSProfile(depth=10)
        self._servo_pub = self.create_publisher(
            ServosPosition, "/ros_robot_controller/bus_servo/set_position", qos
        )
        self._state_pub = self.create_publisher(
            Float32MultiArray, "/neuro_flow/target_state", qos
        )
        self._cmd_pub = self.create_publisher(
            String, "/neuro_flow/record_cmd", qos
        )

        # Read current joint angles from robot before first publish
        self._q = self._read_current_joints()
        self.create_timer(1.0 / HEARTBEAT_HZ, self._publish)

    def _read_current_joints(self) -> list:
        """Spin briefly to receive one JointState; return joint angles in rad."""
        q = [0.0, -0.5, 0.5, 0.3, 0.0]  # fallback
        got_it = [False]

        def _cb(msg: JointState):
            if got_it[0]:
                return
            name_to_pos = dict(zip(msg.name, msg.position))
            if all(n in name_to_pos for n in JOINT_NAMES):
                for i, name in enumerate(JOINT_NAMES):
                    q[i] = float(name_to_pos[name])
                got_it[0] = True

        sub = self.create_subscription(JointState, "/controller_manager/joint_states", _cb, 1)
        print("Waiting for current joint state...", flush=True)
        deadline = self.get_clock().now().nanoseconds + 3_000_000_000
        while not got_it[0] and self.get_clock().now().nanoseconds < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
        self.destroy_subscription(sub)
        if not got_it[0]:
            print("Warning: no joint state received, using default pose", flush=True)
        return q

    def _publish(self):
        if not self._dirty:
            return
        self._dirty = False

        pulses = [angle_to_pulse(self._q[i], i) for i in range(5)]

        servo_msg = ServosPosition()
        servo_msg.duration = 1.0 / HEARTBEAT_HZ
        servo_msg.position = [
            ServoPosition(id=SERVO_IDS[i], position=pulses[i]) for i in range(5)
        ] + [ServoPosition(id=GRIPPER_ID, position=self._gripper_pulse)]
        self._servo_pub.publish(servo_msg)

        # Publish joint angles in radians for the collector
        # Format: [q1, q2, q3, q4, q5, gripper_normalised]
        gripper_norm = (self._gripper_pulse - GRIPPER_OPEN_PULSE) / (
            GRIPPER_CLOSE_PULSE - GRIPPER_OPEN_PULSE
        )
        state_msg = Float32MultiArray()
        state_msg.data = [float(v) for v in self._q] + [float(gripper_norm)]
        self._state_pub.publish(state_msg)

    def _move_ee(self, twist: list):
        """Apply a 6D twist [vx,vy,vz, wx,wy,wz] via DLS IK."""
        self._q = ik_step(self._q, twist)
        self._dirty = True

    def _print_cmd(self, label: str):
        pulses = [angle_to_pulse(self._q[i], i) for i in range(5)]
        ee = fk(self._q)
        pos_str = "  ".join(f"id{SERVO_IDS[i]}={pulses[i]}" for i in range(5))
        print(
            f"\n[{label}]  EE=({ee.p.x():.3f},{ee.p.y():.3f},{ee.p.z():.3f})  "
            f"{pos_str}  id{GRIPPER_ID}={self._gripper_pulse}",
            flush=True,
        )

    def apply_key(self, key: str) -> bool:
        """Returns False to signal exit."""
        d = EE_STEP
        r = ROTATION_STEP

        # ── Translation ──────────────────────────────────────────────────────
        if key == "w":
            self._move_ee([d, 0, 0,  0, 0, 0]); self._print_cmd("+X")
        elif key == "s":
            self._move_ee([-d, 0, 0, 0, 0, 0]); self._print_cmd("-X")
        elif key == "a":
            # joint1 is the yaw — direct control is the only reliable Y-motion
            self._q[0] = max(-JOINT_LIMITS, self._q[0] + r); self._dirty = True; self._print_cmd("yaw+")
        elif key == "d":
            self._q[0] = min( JOINT_LIMITS, self._q[0] - r); self._dirty = True; self._print_cmd("yaw-")
        elif key == "q":
            self._move_ee([0, 0, d,  0, 0, 0]); self._print_cmd("+Z")
        elif key == "e":
            self._move_ee([0, 0, -d, 0, 0, 0]); self._print_cmd("-Z")
        # ── Wrist: direct joint control (bypasses IK) ────────────────────────
        # joint4 = wrist pitch (revolute, Y-axis)
        # joint5 = wrist roll  (revolute, −Z-axis)
        elif key == "i":
            self._q[3] = max(-JOINT_LIMITS, self._q[3] + r); self._dirty = True; self._print_cmd("j4+")
        elif key == "k":
            self._q[3] = min( JOINT_LIMITS, self._q[3] - r); self._dirty = True; self._print_cmd("j4-")
        elif key == "j":
            self._q[4] = max(-JOINT_LIMITS, self._q[4] + r); self._dirty = True; self._print_cmd("j5+")
        elif key == "l":
            self._q[4] = min( JOINT_LIMITS, self._q[4] - r); self._dirty = True; self._print_cmd("j5-")
        elif key in ("u", "o"):
            pass  # only 2 wrist DOF on this arm
        # ── Gripper ──────────────────────────────────────────────────────────
        elif key == " ":
            if self._gripper_pulse == GRIPPER_OPEN_PULSE:
                self._gripper_pulse = GRIPPER_CLOSE_PULSE
                print("\n[Gripper: CLOSE]", flush=True)
            else:
                self._gripper_pulse = GRIPPER_OPEN_PULSE
                print("\n[Gripper: OPEN]", flush=True)
            self._dirty = True
        # ── Recording ────────────────────────────────────────────────────────
        elif key == "r":
            self._recording = not self._recording
            cmd = "start" if self._recording else "pause"
            self._cmd_pub.publish(String(data=cmd))
            status = "RECORDING" if self._recording else "PAUSED"
            print(f"\n[{status}]", flush=True)
        elif key in ("\r", "\n"):
            if self._recording:
                self._recording = False
                self._cmd_pub.publish(String(data="save"))
                print("\n[SAVED]", flush=True)
        elif key == "x":
            self._recording = False
            self._cmd_pub.publish(String(data="discard"))
            print("\n[DISCARDED]", flush=True)
        elif key == "\x03":
            return False
        return True


# ── Main ──────────────────────────────────────────────────────────────────────

def _get_key(settings) -> str:
    tty.setraw(sys.stdin.fileno())
    readable, _, _ = select.select([sys.stdin], [], [], 0.05)
    key = sys.stdin.read(1) if readable else ""
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def main():
    rclpy.init()
    node = TeleopArmNode()

    executor = MultiThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    settings = termios.tcgetattr(sys.stdin)
    print(HELP, flush=True)

    # Print initial EE position
    ee = fk(node._q)
    print(f"Initial EE position: x={ee.p.x():.3f}  y={ee.p.y():.3f}  z={ee.p.z():.3f}\n", flush=True)

    try:
        while rclpy.ok():
            key = _get_key(settings)
            if key and not node.apply_key(key):
                break
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
