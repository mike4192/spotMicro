#!/usr/bin/env python3
"""MuJoCo simulation bridge for SpotMicro robot."""

import math
import os
import shutil
import subprocess
from typing import Dict, Optional

import mujoco
import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from i2cpwm_board.msg import ServoArray
from i2cpwm_board.srv import ServosConfig
from PIL import Image
from rclpy.node import Node
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster


class SpotMicroMujocoSim(Node):
    """ROS2 node that bridges SpotMicro servo commands to MuJoCo simulation."""

    # Default servo configuration extracted from spot_micro_motion_cmd.yaml.
    # center_angle_deg: kinematic angle (rad) that corresponds to proportional=0.
    # direction: hardware direction multiplier (-1 or 1).
    DEFAULT_SERVO_CONFIG: Dict[str, Dict] = {
        "RF_1": {"num": 3,  "center_angle_deg": -5.4,  "direction": -1},
        "RF_2": {"num": 2,  "center_angle_deg": -27.6, "direction":  1},
        "RF_3": {"num": 1,  "center_angle_deg": 88.2,  "direction":  1},
        "RB_1": {"num": 6,  "center_angle_deg": -4.4,  "direction":  1},
        "RB_2": {"num": 5,  "center_angle_deg": -35.4, "direction":  1},
        "RB_3": {"num": 4,  "center_angle_deg": 85.8,  "direction":  1},
        "LB_1": {"num": 9,  "center_angle_deg": -0.4,  "direction": -1},
        "LB_2": {"num": 8,  "center_angle_deg": 38.7,  "direction":  1},
        "LB_3": {"num": 7,  "center_angle_deg": -73.9, "direction":  1},
        "LF_1": {"num": 10, "center_angle_deg": -7.6,  "direction":  1},
        "LF_2": {"num": 11, "center_angle_deg": 38.6,  "direction":  1},
        "LF_3": {"num": 12, "center_angle_deg": -82.8, "direction":  1},
    }

    # Kinematic -> URDF joint angle signs.  These must match the TF convention in
    # spot_micro_motion_cmd.cpp:
    #   Right side: shoulder=+ang1, leg=-ang2, foot=-ang3
    #   Left side:  shoulder=-ang1, leg=+ang2, foot=+ang3
    SERVO_TO_JOINT: Dict[str, str] = {
        "RF_1": "front_right_shoulder",
        "RF_2": "front_right_leg",
        "RF_3": "front_right_foot",
        "RB_1": "rear_right_shoulder",
        "RB_2": "rear_right_leg",
        "RB_3": "rear_right_foot",
        "LF_1": "front_left_shoulder",
        "LF_2": "front_left_leg",
        "LF_3": "front_left_foot",
        "LB_1": "rear_left_shoulder",
        "LB_2": "rear_left_leg",
        "LB_3": "rear_left_foot",
    }

    # Default pose angles (all zero) — matches the MuJoCo model's default
    # configuration where legs hang straight down.
    DEFAULT_ANGLES_DEG: Dict[str, float] = {
        "RF_1": 0.0, "RF_2": 0.0, "RF_3": 0.0,
        "RB_1": 0.0, "RB_2": 0.0, "RB_3": 0.0,
        "LF_1": 0.0, "LF_2": 0.0, "LF_3": 0.0,
        "LB_1": 0.0, "LB_2": 0.0, "LB_3": 0.0,
    }

    def __init__(self):
        super().__init__("spot_micro_mujoco_sim")

        # Parameters
        self.declare_parameter("model_path", "")
        self.declare_parameter("servo_max_angle_deg", 82.5)
        self.declare_parameter("sim_rate_hz", 500.0)
        self.declare_parameter("publish_rate_hz", 50.0)
        self.declare_parameter("initial_body_z", 0.25)
        self.declare_parameter("initial_body_quat", [1.0, 0.0, 0.0, 0.0])

        # Video recording parameters
        self.declare_parameter("record_video", False)
        self.declare_parameter("video_fps", 30.0)
        self.declare_parameter("video_width", 1280)
        self.declare_parameter("video_height", 720)
        self.declare_parameter("video_output_path", "/tmp/spotmicro_sim_video.mp4")

        default_signs = {
            "RF_1": 1.0,  "RF_2": -1.0, "RF_3": -1.0,
            "RB_1": 1.0,  "RB_2": -1.0, "RB_3": -1.0,
            "LF_1": -1.0, "LF_2": 1.0,  "LF_3": 1.0,
            "LB_1": -1.0, "LB_2": 1.0,  "LB_3": 1.0,
        }
        for servo_name in self.DEFAULT_SERVO_CONFIG:
            self.declare_parameter(f"joint_signs.{servo_name}", default_signs.get(servo_name, 1.0))

        model_path = self.get_parameter("model_path").value
        if not model_path or not os.path.exists(model_path):
            pkg_share = os.path.join(
                os.path.dirname(__file__), "..", "..", "share", "spot_micro_mujoco_sim"
            )
            model_path = os.path.join(pkg_share, "models", "spot_micro_sim.xml")
            if not os.path.exists(model_path):
                model_path = "/tmp/spotmicro_mujoco/spot_micro_sim.xml"
            self.get_logger().info(f"Using default model path: {model_path}")

        if not os.path.exists(model_path):
            self.get_logger().error(f"MuJoCo model not found: {model_path}")
            raise FileNotFoundError(f"Model not found: {model_path}")

        # Video recording setup
        self.record_video = self.get_parameter("record_video").value
        self.video_fps = self.get_parameter("video_fps").value
        self.video_width = self.get_parameter("video_width").value
        self.video_height = self.get_parameter("video_height").value
        self.video_output_path = self.get_parameter("video_output_path").value
        self._frame_dir = "/tmp/spotmicro_video_frames"
        self._frame_count = 0
        self._last_frame_time = 0.0
        self._renderer = None
        self._camera = None

        if self.record_video:
            # Inject visual/global settings into MJCF so offscreen framebuffer is large enough
            with open(model_path, "r") as f:
                xml_src = f.read()
            # Add <visual><global offwidth="..." offheight="..."/></visual> if not present
            if "<visual>" not in xml_src:
                # Find the opening <mujoco ...> tag and insert visual block after it
                import re
                xml_src = re.sub(
                    r'(<mujoco[^>]*>)',
                    rf'\1\n  <visual>\n    <global offwidth="{self.video_width}" offheight="{self.video_height}"/>\n  </visual>',
                    xml_src,
                    count=1,
                )
            model_path_mod = "/tmp/spotmicro_mujoco/spot_micro_sim_with_visual.xml"
            with open(model_path_mod, "w") as f:
                f.write(xml_src)
            model_path = model_path_mod

            os.makedirs(self._frame_dir, exist_ok=True)
            # Clean old frames
            for f in os.listdir(self._frame_dir):
                os.remove(os.path.join(self._frame_dir, f))
            self.get_logger().info(
                f"Video recording enabled: {self.video_width}x{self.video_height} @ {self.video_fps} FPS -> {self.video_output_path}"
            )

        # Load MuJoCo model
        self.model = mujoco.MjModel.from_xml_path(model_path)
        self.data = mujoco.MjData(self.model)

        self.get_logger().info(
            f"MuJoCo model loaded: {self.model.njnt} joints, {self.model.nu} actuators, "
            f"{self.model.nq} qpos, {self.model.nv} qvel"
        )

        # Build name -> id lookups
        self.joint_ids: Dict[str, int] = {}
        self.joint_qposadr: Dict[str, int] = {}
        self.actuator_ids: Dict[str, int] = {}

        for i in range(self.model.njnt):
            name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_JOINT, i)
            self.joint_ids[name] = i
            self.joint_qposadr[name] = self.model.jnt_qposadr[i]

        for i in range(self.model.nu):
            name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, i)
            self.actuator_ids[name] = i

        # Verify all required joints exist
        for servo_name, joint_name in self.SERVO_TO_JOINT.items():
            if joint_name not in self.joint_ids:
                self.get_logger().warn(f"Joint {joint_name} not found in MuJoCo model")

        # Servo state
        self.servo_commands: Dict[int, float] = {}  # servo_num -> proportional value
        self.servo_config: Dict[str, Dict] = {}
        self.servo_config_received = False

        # Simulation parameters
        self.servo_max_angle_rad = (
            self.get_parameter("servo_max_angle_deg").value * math.pi / 180.0
        )
        self.sim_dt = 1.0 / self.get_parameter("sim_rate_hz").value
        self.publish_dt = 1.0 / self.get_parameter("publish_rate_hz").value
        self.model.opt.timestep = self.sim_dt

        # Initialize simulation pose
        self._reset_simulation()

        # Setup video renderer after reset so data is valid
        if self.record_video:
            try:
                self._renderer = mujoco.Renderer(self.model, self.video_height, self.video_width)
                self._camera = mujoco.MjvCamera()
                self._camera.type = mujoco.mjtCamera.mjCAMERA_TRACKING
                self._camera.trackbodyid = self.model.body("base_link").id
                self._camera.distance = 0.8
                self._camera.azimuth = 135
                self._camera.elevation = -20
                self.get_logger().info("Offscreen renderer initialized")
            except Exception as e:
                self.get_logger().error(f"Failed to initialize renderer: {e}")
                self.record_video = False
                self._renderer = None

        # ROS interfaces
        self.servos_config_srv = self.create_service(
            ServosConfig, "config_servos", self._config_servos_callback
        )
        self.servos_proportional_sub = self.create_subscription(
            ServoArray, "servos_proportional", self._servos_proportional_callback, 10
        )
        self.servos_absolute_sub = self.create_subscription(
            ServoArray, "servos_absolute", self._servos_absolute_callback, 10
        )
        self.joint_state_pub = self.create_publisher(JointState, "joint_states", 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # Timers
        self.sim_timer = self.create_timer(self.sim_dt, self._sim_step)
        self.pub_timer = self.create_timer(self.publish_dt, self._publish_state)

        self.get_logger().info("SpotMicro MuJoCo simulation node started")

    def _reset_simulation(self):
        """Reset simulation to default pose."""
        mujoco.mj_resetData(self.model, self.data)

        initial_z = self.get_parameter("initial_body_z").value
        initial_quat = self.get_parameter("initial_body_quat").value

        # Free joint qpos is first 7 elements: [x, y, z, qw, qx, qy, qz]
        self.data.qpos[0:3] = [0.0, 0.0, initial_z]
        self.data.qpos[3:7] = initial_quat

        # Set initial joint angles to default (all zero)
        for servo_name, joint_name in self.SERVO_TO_JOINT.items():
            if joint_name not in self.joint_ids:
                continue
            qpos_adr = self.joint_qposadr[joint_name]
            self.data.qpos[qpos_adr] = 0.0

        # Set actuators to hold initial pose (all zero)
        for servo_name, joint_name in self.SERVO_TO_JOINT.items():
            if joint_name not in self.joint_ids:
                continue
            actuator_name = f"{joint_name}_actuator"
            if actuator_name in self.actuator_ids:
                act_id = self.actuator_ids[actuator_name]
                self.data.ctrl[act_id] = 0.0

        # Forward kinematics to initialize all derived quantities
        mujoco.mj_forward(self.model, self.data)
        toe_z = [
            float(self.data.geom_xpos[i][2])
            for i in range(self.model.ngeom)
            if mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_GEOM, i)
            and "toe" in (mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_GEOM, i) or "")
        ]
        self.get_logger().info(
            f"Simulation reset: body_z={self.data.qpos[2]:.3f}, toes z={toe_z}"
        )

    def _config_servos_callback(self, request, response):
        """Handle servo config service (called by spot_micro_motion_cmd at startup)."""
        self.get_logger().info(
            f"Received servo config for {len(request.servos)} servos"
        )
        for servo in request.servos:
            # Find servo name by num
            for name, cfg in self.DEFAULT_SERVO_CONFIG.items():
                if cfg["num"] == servo.servo:
                    self.servo_config[name] = {
                        "num": servo.servo,
                        "center": servo.center,
                        "range": servo.range,
                        "direction": servo.direction,
                        "center_angle_deg": cfg["center_angle_deg"],
                    }
                    break
        self.servo_config_received = True
        response.success = True
        return response

    def _get_servo_cfg(self, servo_name: str) -> Dict:
        """Return active config for a servo (service override or default)."""
        if servo_name in self.servo_config:
            return self.servo_config[servo_name]
        return self.DEFAULT_SERVO_CONFIG.get(servo_name, {})

    def _servos_proportional_callback(self, msg: ServoArray):
        """Handle incoming servo proportional commands."""
        for servo in msg.servos:
            self.servo_commands[servo.servo] = servo.value
        self.get_logger().debug(
            f"Received {len(msg.servos)} servo proportional commands"
        )

    def _servos_absolute_callback(self, msg: ServoArray):
        """Handle incoming servo absolute commands (idle state sends all zeros = off)."""
        pass

    def _proportional_to_joint_angle(self, servo_name: str, proportional: float) -> float:
        """Convert proportional servo command to URDF joint angle in radians."""
        cfg = self._get_servo_cfg(servo_name)
        if not cfg:
            return 0.0
        sign = self.get_parameter(f"joint_signs.{servo_name}").value
        direction = float(cfg.get("direction", 1))
        center_rad = cfg["center_angle_deg"] * math.pi / 180.0
        # Motion cmd computes: proportional = (cmd_kin - center) / max * direction
        # Recover kinematic angle: cmd_kin = proportional * max * direction + center
        cmd_kin_rad = proportional * self.servo_max_angle_rad * direction + center_rad
        return sign * cmd_kin_rad

    def _capture_frame(self):
        """Render current simulation state to a PNG frame."""
        if not self.record_video or self._renderer is None:
            return

        now = self.get_clock().now().nanoseconds / 1e9
        if now - self._last_frame_time < (1.0 / self.video_fps):
            return
        self._last_frame_time = now

        try:
            self._renderer.update_scene(self.data, camera=self._camera)
            frame = self._renderer.render()
            self._frame_count += 1
            path = os.path.join(self._frame_dir, f"frame_{self._frame_count:06d}.png")
            Image.fromarray(frame).save(path)
        except Exception as e:
            self.get_logger().warning(f"Frame capture failed: {e}")

    def _save_video(self):
        """Encode captured frames to MP4 using ffmpeg."""
        if not self.record_video or self._frame_count == 0:
            return

        self.get_logger().info(
            f"Encoding {self._frame_count} frames to {self.video_output_path} ..."
        )
        cmd = [
            "ffmpeg",
            "-y",
            "-framerate", str(int(self.video_fps)),
            "-i", os.path.join(self._frame_dir, "frame_%06d.png"),
            "-c:v", "libx264",
            "-pix_fmt", "yuv420p",
            "-crf", "23",
            self.video_output_path,
        ]
        try:
            result = subprocess.run(cmd, capture_output=True, text=True)
            if result.returncode == 0:
                self.get_logger().info(f"Video saved: {self.video_output_path}")
            else:
                self.get_logger().error(f"ffmpeg failed: {result.stderr}")
        except Exception as e:
            self.get_logger().error(f"ffmpeg error: {e}")
        finally:
            # Optional: clean up frames
            shutil.rmtree(self._frame_dir, ignore_errors=True)

    def _sim_step(self):
        """Step the MuJoCo simulation one timestep."""
        if hasattr(self, "_step_count"):
            self._step_count += 1
        else:
            self._step_count = 1

        if self._step_count % 500 == 0:
            self.get_logger().debug(f"Step {self._step_count}: body_z={self.data.qpos[2]:.3f}")

        # Apply servo commands to actuators
        for servo_name, joint_name in self.SERVO_TO_JOINT.items():
            cfg = self._get_servo_cfg(servo_name)
            if not cfg:
                continue
            servo_num = cfg["num"]
            proportional = self.servo_commands.get(servo_num, 0.0)
            target_angle = self._proportional_to_joint_angle(servo_name, proportional)

            actuator_name = f"{joint_name}_actuator"
            if actuator_name in self.actuator_ids:
                act_id = self.actuator_ids[actuator_name]
                self.data.ctrl[act_id] = target_angle

        # Step physics
        mujoco.mj_step(self.model, self.data)

    def _publish_state(self):
        """Publish joint states and TF transforms."""
        now = self.get_clock().now()
        stamp = now.to_msg()

        # Joint state message
        joint_state = JointState()
        joint_state.header.stamp = stamp
        joint_state.header.frame_id = "base_link"

        # Publish all actuated joints
        for servo_name, joint_name in self.SERVO_TO_JOINT.items():
            if joint_name not in self.joint_ids:
                continue
            qpos_adr = self.joint_qposadr[joint_name]
            joint_state.name.append(joint_name)
            joint_state.position.append(float(self.data.qpos[qpos_adr]))
            qvel_adr = self.model.jnt_dofadr[self.joint_ids[joint_name]]
            joint_state.velocity.append(float(self.data.qvel[qvel_adr]))
            joint_state.effort.append(0.0)

        self.joint_state_pub.publish(joint_state)

        # Publish TF for base link
        base_pos = self.data.qpos[0:3].copy()
        base_quat = self.data.qpos[3:7].copy()  # [w, x, y, z]

        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = "world"
        t.child_frame_id = "base_link"
        t.transform.translation.x = float(base_pos[0])
        t.transform.translation.y = float(base_pos[1])
        t.transform.translation.z = float(base_pos[2])
        t.transform.rotation.w = float(base_quat[0])
        t.transform.rotation.x = float(base_quat[1])
        t.transform.rotation.y = float(base_quat[2])
        t.transform.rotation.z = float(base_quat[3])
        self.tf_broadcaster.sendTransform(t)

        # Capture video frame if recording
        self._capture_frame()


def main(args=None):
    rclpy.init(args=args)
    node = SpotMicroMujocoSim()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._save_video()
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()
