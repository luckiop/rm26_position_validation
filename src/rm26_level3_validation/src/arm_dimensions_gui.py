#!/usr/bin/env python3
"""RM27 joint-limit and reach-dimension explorer using the installed MuJoCo model."""

from __future__ import annotations

import math
import json
import os
from pathlib import Path
import queue
import shutil
import subprocess
import sys
import tempfile
import threading
import tkinter as tk
from tkinter import messagebox, ttk
import xml.etree.ElementTree as ET


JOINT_LABELS = ("J0 底座", "J1 肩部", "J2 肘部", "J3 腕部", "J4 腕部", "J5 腕部", "J6 末端")
JOINTS = tuple(f"joint_{index}" for index in range(1, 8))
HOME_JOINTS = (0.0, 1.39, -1.3, 0.0, 0.0, 0.3, 0.0)
DEG = 180.0 / math.pi
RAD = math.pi / 180.0

BG = "#eef2f6"
CARD = "#ffffff"
INK = "#192635"
MUTED = "#718096"
BLUE = "#1769aa"


def find_fk_executable() -> Path | None:
    """Locate the native MuJoCo forward-kinematics helper."""
    explicit = os.environ.get("RM27_ARM_DIMENSIONS_FK")
    if explicit and Path(explicit).is_file():
        return Path(explicit)

    script = Path(sys.argv[0]).resolve()
    candidates = [script.with_name("arm_dimensions_fk")]
    for prefix in os.environ.get("AMENT_PREFIX_PATH", "").split(os.pathsep):
        if prefix:
            candidates.append(Path(prefix) / "rm26_level3_validation" / "bin" / "arm_dimensions_fk")

    here = Path.cwd()
    for root in (here, *here.parents):
        for build_name in ("build-integrated", "build", "build-master"):
            candidates.append(root / build_name / "rm26_level3_validation" / "arm_dimensions_fk")

    for candidate in candidates:
        if candidate.is_file() and os.access(candidate, os.X_OK):
            return candidate
    from_path = shutil.which("arm_dimensions_fk")
    return Path(from_path) if from_path else None


class FKClient:
    """Keep one MuJoCo model loaded for repeated joint-position queries."""

    def __init__(self, executable: Path):
        self.process = subprocess.Popen(
            [str(executable)],
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            bufsize=1,
        )

    def positions(
        self, joints: tuple[float, ...] | list[float]
    ) -> tuple[list[tuple[float, float, float]], tuple[float, float, float]]:
        if self.process.poll() is not None or self.process.stdin is None or self.process.stdout is None:
            message = "MuJoCo 正运动学进程已退出"
            if self.process.stderr is not None:
                detail = self.process.stderr.read().strip()
                if detail:
                    message += f": {detail}"
            raise RuntimeError(message)
        self.process.stdin.write(",".join(f"{value:.17g}" for value in joints) + "\n")
        self.process.stdin.flush()
        line = self.process.stdout.readline().strip()
        if not line:
            raise RuntimeError("MuJoCo 正运动学没有返回位置")
        if line.startswith("ERROR,"):
            raise RuntimeError(line[6:])
        values = [float(value) for value in line.split(",")]
        if len(values) != 27:
            raise RuntimeError(f"正运动学返回了 {len(values)} 个坐标，预期为 27 个")
        reference_offset = tuple(values[:3])
        positions = [tuple(values[index : index + 3]) for index in range(3, len(values), 3)]
        return positions, reference_offset

    def close(self) -> None:
        if self.process.stdin is not None:
            self.process.stdin.close()
        try:
            self.process.wait(timeout=1.0)
        except subprocess.TimeoutExpired:
            self.process.terminate()
            try:
                self.process.wait(timeout=1.0)
            except subprocess.TimeoutExpired:
                self.process.kill()


def read_model_limits(executable: Path) -> tuple[list[tuple[float, float]], list[float]]:
    completed = subprocess.run(
        [str(executable), "--limits"], capture_output=True, text=True, check=True, timeout=10
    )
    limits: list[tuple[float, float]] = []
    initial: list[float] = []
    for line in completed.stdout.splitlines():
        fields = line.split(",")
        if len(fields) != 4:
            raise RuntimeError(f"无法读取模型关节限位：{line}")
        name, lower, upper, position = fields
        if name != JOINTS[len(limits)]:
            raise RuntimeError(f"模型关节顺序不匹配：{name}")
        limits.append((float(lower), float(upper)))
        initial.append(float(position))
    if len(limits) != 7:
        raise RuntimeError(f"模型返回 {len(limits)} 组限位，预期为 7 组")
    return limits, initial


def read_mount_pose(executable: Path) -> tuple[float, ...]:
    """Read the planner's energy-unit transform relative to link_7."""
    result = subprocess.run([str(executable), "--mount-pose"], capture_output=True,
                            text=True, check=True, timeout=10)
    pose = tuple(float(value) for value in result.stdout.strip().split(","))
    if len(pose) != 7 or not all(math.isfinite(value) for value in pose):
        raise RuntimeError("无法读取能量单元安装坐标系")
    return pose


def arm_urdf_for_rviz() -> str:
    """Create an arm-only URDF with mesh paths RViz can resolve directly."""
    from ament_index_python.packages import get_package_share_directory

    model_dir = Path(get_package_share_directory("rm27_heavy_arm_demo")) / "model"
    urdf_path = model_dir / "urdf" / "rm27_heavy_arm_demo.urdf"
    robot = ET.parse(urdf_path).getroot()
    allowed_links = {"base_link", "tool0", *(f"link_{index}" for index in range(1, 8))}
    allowed_joints = {*JOINTS, "tool0_fixed"}

    for joint in list(robot.findall("joint")):
        if joint.attrib.get("name") not in allowed_joints:
            robot.remove(joint)
    for link in list(robot.findall("link")):
        name = link.attrib.get("name", "")
        if name not in allowed_links:
            robot.remove(link)
            continue
        for collision in list(link.findall("collision")):
            link.remove(collision)
        for mesh in link.findall(".//mesh"):
            filename = mesh.attrib.get("filename", "")
            mesh_prefix = "package://rm27_heavy_arm_demo/model/meshes/"
            if filename.startswith(mesh_prefix):
                mesh_path = model_dir / "meshes" / filename[len(mesh_prefix) :]
                mesh.attrib["filename"] = mesh_path.as_uri()
    return ET.tostring(robot, encoding="unicode")


def rod_segments(urdf: str, mount_pose: tuple[float, ...]) -> list[tuple[str, tuple[float, ...], tuple[float, ...]]]:
    """Projected rod vectors and residual offsets in each parent link frame."""
    robot = ET.fromstring(urdf)
    masks = ((1, 0, 1), (1, 0, 1), (1, 0, 1), (1, 0, 0), (1, 0, 1), (1, 0, 0))
    segments = []
    for index, mask in enumerate(masks):
        joint = robot.find(f"joint[@name='joint_{index + 2}']/origin")
        if joint is None:
            raise RuntimeError(f"模型缺少 joint_{index + 2} 原点")
        offset = tuple(float(value) for value in joint.get("xyz", "0 0 0").split())
        if len(offset) != 3 or not all(math.isfinite(value) for value in offset):
            raise RuntimeError("模型关节偏移无效")
        axial = tuple(value * component for value, component in zip(offset, mask))
        segments.append((f"link_{index + 1}", axial, offset))
    segments.append(("link_7", (mount_pose[0], 0., 0.), mount_pose[:3]))
    return segments


def find_rviz_config() -> Path:
    """Find the installed or source RViz display configuration."""
    script = Path(sys.argv[0]).resolve()
    candidates = [
        script.parent.parent / "share" / "rm26_level3_validation" / "config" / "arm_dimensions.rviz",
        script.parents[1] / "config" / "arm_dimensions.rviz",
    ]
    for candidate in candidates:
        if candidate.is_file():
            return candidate
    raise RuntimeError("找不到 config/arm_dimensions.rviz；请重新构建 rm26_level3_validation")


class RosVisualizationBridge:
    """Publish slider joint states and dimension markers for RViz."""

    def __init__(self, mount_pose: tuple[float, ...], segments: list[tuple[str, tuple[float, ...], tuple[float, ...]]]):
        import rclpy
        from geometry_msgs.msg import Point
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.node import Node
        from sensor_msgs.msg import JointState
        from visualization_msgs.msg import Marker, MarkerArray

        self.rclpy = rclpy
        self.Point = Point
        self.Marker = Marker
        self.MarkerArray = MarkerArray
        self.JointState = JointState
        self.mount_pose = mount_pose
        self.segments = segments
        if not rclpy.ok():
            rclpy.init(args=None)
        self.node = Node("rm26_arm_dimensions")
        self.joint_publisher = self.node.create_publisher(JointState, "/joint_states", 10)
        self.marker_publisher = self.node.create_publisher(MarkerArray, "/arm_dimensions/markers", 10)
        self.lock = threading.Lock()
        self.joints = [0.0] * 7
        self.endpoint = (0.0, 0.0, 0.0)
        self.reference_offset = (0.0, 0.0, 0.0)
        self.node.create_timer(0.1, self._publish_current)
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)
        self.thread = threading.Thread(target=self.executor.spin, daemon=True, name="rm26-rviz-bridge")
        self.thread.start()

    def update(
        self,
        joints: list[float],
        endpoint: tuple[float, float, float],
        reference_offset: tuple[float, float, float],
    ) -> None:
        with self.lock:
            self.joints = list(joints)
            self.endpoint = endpoint
            self.reference_offset = reference_offset

    def _publish_current(self) -> None:
        with self.lock:
            joints = list(self.joints)
            endpoint = self.endpoint
            offset = self.reference_offset

        state = self.JointState()
        state.header.stamp = self.node.get_clock().now().to_msg()
        state.name = list(JOINTS)
        state.position = joints
        self.joint_publisher.publish(state)

        foot = (offset[0] + endpoint[0], offset[1] + endpoint[1], endpoint[2])
        tip = self.Point(x=foot[0], y=foot[1], z=foot[2])
        origin = self.Point(x=offset[0], y=offset[1], z=0.0)
        floor = self.Point(x=foot[0], y=foot[1], z=0.0)
        markers = self.MarkerArray()
        markers.markers = [
            self._line_marker(0, [origin, floor], (0.08, 0.72, 0.72)),
            self._line_marker(1, [floor, tip], (0.96, 0.53, 0.12)),
            self._tool_marker(2, tip),
            self._text_marker(3, endpoint, offset),
        ]
        for marker in markers.markers:
            marker.header.stamp = state.header.stamp
            marker.header.frame_id = "base_link"
        markers.markers.extend(self._frame_markers(state.header.stamp))
        markers.markers.extend(self._rod_markers(state.header.stamp))
        self.marker_publisher.publish(markers)

    def _rod_markers(self, stamp: object) -> list[object]:
        markers = []
        for index, (frame, axial, offset) in enumerate(self.segments):
            def point(values: tuple[float, ...]) -> object:
                return self.Point(x=values[0], y=values[1], z=values[2])

            def marker(marker_id: int, marker_type: int) -> object:
                result = self.Marker()
                result.header.frame_id = frame
                result.header.stamp = stamp
                result.frame_locked = True
                result.ns = 'rod_lengths'
                result.id = index * 3 + marker_id
                result.type = marker_type
                result.action = self.Marker.ADD
                result.pose.orientation.w = 1.0
                return result

            line = marker(0, self.Marker.LINE_LIST)
            line.scale.x = .005
            line.color.r, line.color.g, line.color.b, line.color.a = 1., .78, .12, 1.
            line.points = [point((0., 0., 0.)), point(axial)]
            markers.append(line)

            residual = marker(1, self.Marker.LINE_LIST)
            residual.scale.x = .0025
            residual.color.r, residual.color.g, residual.color.b, residual.color.a = .75, .75, .75, .9
            residual.points = [point(axial), point(offset)]
            markers.append(residual)

            label = marker(2, self.Marker.TEXT_VIEW_FACING)
            label.pose.position.x = axial[0] / 2
            label.pose.position.y = axial[1] / 2
            label.pose.position.z = axial[2] / 2 + .025
            label.scale.z = .025
            label.color.r = label.color.g = label.color.b = label.color.a = 1.
            label.text = f"J{index}  {math.sqrt(sum(value * value for value in axial)) * 1000:.1f} mm"
            markers.append(label)
        return markers

    def _frame_markers(self, stamp: object) -> list[object]:
        # Markers live in TF frames so that roll and all upstream joints move them.
        frames = (
            ("link_7", "J6 roll (joint_7)", (0., 0., 0., 0., 0., 0., 1.)),
            ("tool0", "tool0", (0., 0., 0., 0., 0., 0., 1.)),
            ("link_7", "energy unit mount", self.mount_pose),
        )
        markers = []
        for index, (frame, label, pose) in enumerate(frames):
            for axis, color in enumerate(((1., 0.15, 0.15), (0.15, 1., 0.15), (0.2, 0.45, 1.))):
                marker = self.Marker()
                marker.header.frame_id = frame
                marker.header.stamp = stamp
                marker.frame_locked = True
                marker.ns = "coordinate_frames"
                marker.id = index * 4 + axis
                marker.type = self.Marker.ARROW
                marker.action = self.Marker.ADD
                marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = pose[:3]
                (marker.pose.orientation.x, marker.pose.orientation.y,
                 marker.pose.orientation.z, marker.pose.orientation.w) = pose[3:]
                end = [0., 0., 0.]
                end[axis] = 0.07
                marker.points = [self.Point(x=0., y=0., z=0.), self.Point(x=end[0], y=end[1], z=end[2])]
                marker.scale.x, marker.scale.y, marker.scale.z = 0.003, 0.008, 0.012
                marker.color.r, marker.color.g, marker.color.b = color
                marker.color.a = 1.0
                markers.append(marker)
            text = self.Marker()
            text.header.frame_id = frame
            text.header.stamp = stamp
            text.frame_locked = True
            text.ns = "coordinate_frames"
            text.id = index * 4 + 3
            text.type = self.Marker.TEXT_VIEW_FACING
            text.action = self.Marker.ADD
            text.pose.position.x, text.pose.position.y, text.pose.position.z = pose[:3]
            text.pose.position.z += 0.045
            text.pose.orientation.w = 1.0
            text.scale.z = 0.025
            text.color.r = text.color.g = text.color.b = text.color.a = 1.0
            text.text = label
            markers.append(text)
        return markers

    def _line_marker(self, marker_id: int, points: list[object], color: tuple[float, float, float]) -> object:
        marker = self.Marker()
        marker.ns = "arm_dimensions"
        marker.id = marker_id
        marker.type = self.Marker.LINE_STRIP
        marker.action = self.Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.006
        marker.color.r, marker.color.g, marker.color.b = color
        marker.color.a = 1.0
        marker.points = points
        return marker

    def _tool_marker(self, marker_id: int, point: object) -> object:
        marker = self.Marker()
        marker.ns = "arm_dimensions"
        marker.id = marker_id
        marker.type = self.Marker.SPHERE
        marker.action = self.Marker.ADD
        marker.pose.position = point
        marker.pose.orientation.w = 1.0
        marker.scale.x = marker.scale.y = marker.scale.z = 0.025
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = 1.0, 0.34, 0.1, 1.0
        return marker

    def _text_marker(
        self, marker_id: int, endpoint: tuple[float, float, float], offset: tuple[float, float, float]
    ) -> object:
        marker = self.Marker()
        marker.ns = "arm_dimensions"
        marker.id = marker_id
        marker.type = self.Marker.TEXT_VIEW_FACING
        marker.action = self.Marker.ADD
        marker.pose.position.x = offset[0] + endpoint[0]
        marker.pose.position.y = offset[1] + endpoint[1]
        marker.pose.position.z = endpoint[2] + 0.06
        marker.pose.orientation.w = 1.0
        marker.scale.z = 0.045
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = 1.0, 1.0, 1.0, 1.0
        marker.text = f"XY {math.hypot(endpoint[0], endpoint[1]) * 1000:.0f} mm   Z {endpoint[2] * 1000:.0f} mm"
        return marker

    def close(self) -> None:
        self.executor.shutdown(timeout_sec=1.0)
        self.node.destroy_node()
        if self.rclpy.ok():
            self.rclpy.shutdown()


class RVizSession:
    """Launch robot_state_publisher and an RViz2 model window."""

    def __init__(self):
        self.temp_dir = tempfile.TemporaryDirectory(prefix="rm26_arm_dimensions_")
        self.rsp: subprocess.Popen[bytes] | None = None
        self.rviz: subprocess.Popen[bytes] | None = None
        urdf = arm_urdf_for_rviz()
        self.urdf_path = Path(self.temp_dir.name) / "rm26_arm_only.urdf"
        self.urdf_path.write_text(urdf, encoding="utf-8")
        self.config_path = find_rviz_config()
        self.ros2 = shutil.which("ros2")
        self.rviz_executable = shutil.which("rviz2")
        if not self.ros2 or not self.rviz_executable:
            raise RuntimeError("找不到 ros2 或 rviz2；请先 source ROS 2 与 install-integrated/setup.bash")
        self.start()

    def start(self) -> None:
        if self.rsp is None or self.rsp.poll() is not None:
            description = self.urdf_path.read_text(encoding="utf-8")
            parameter = "robot_description:=" + json.dumps(description)
            self.rsp = subprocess.Popen(
                [self.ros2, "run", "robot_state_publisher", "robot_state_publisher", "--ros-args", "-p", parameter],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
        if self.rviz is None or self.rviz.poll() is not None:
            self.rviz = subprocess.Popen(
                [self.rviz_executable, "-d", str(self.config_path)],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )

    def status(self) -> str:
        if self.rsp is not None and self.rsp.poll() is not None:
            return "robot_state_publisher 已退出；点击“重开 RViz”恢复。"
        if self.rviz is not None and self.rviz.poll() is not None:
            return "RViz 已关闭；点击“重开 RViz”恢复。"
        return "RViz 3D 模型已启动；滑块姿态和尺寸标记实时同步。"

    def close(self) -> None:
        for process in (self.rviz, self.rsp):
            if process is not None and process.poll() is None:
                process.terminate()
                try:
                    process.wait(timeout=2.0)
                except subprocess.TimeoutExpired:
                    process.kill()
        self.temp_dir.cleanup()


class ArmDimensionsApp:
    """Interactive RM27 reach visualization and joint-limit search window."""

    def __init__(self, root: tk.Tk, fk_executable: Path):
        self.root = root
        self.fk_executable = fk_executable
        self.limits, _model_initial = read_model_limits(fk_executable)
        self.q = [min(max(angle, lower), upper) for angle, (lower, upper) in zip(HOME_JOINTS, self.limits)]
        self.fk = FKClient(fk_executable)
        self.positions, self.reference_offset = self.fk.positions(self.q)
        mount_pose = read_mount_pose(fk_executable)
        self.ros_bridge = RosVisualizationBridge(mount_pose, rod_segments(arm_urdf_for_rviz(), mount_pose))
        self.rviz = RVizSession()
        self.ros_bridge.update(self.q, self.positions[-1], self.reference_offset)
        self.search_results: dict[str, tuple[list[float], list[tuple[float, float, float]], float]] = {}
        self.message_queue: queue.Queue[tuple[str, object]] = queue.Queue()
        self.searching = False
        self.update_job: str | None = None

        self.root.title("RM27 机械臂伸展尺寸验证")
        self.root.geometry("1120x900")
        self.root.minsize(1000, 760)
        self.root.configure(bg=BG)
        self.root.protocol("WM_DELETE_WINDOW", self.close)
        self._build_ui()
        self._update_all()
        self.root.after(80, self._poll_messages)
        self.root.after(500, self._refresh_rviz_status)

    def _build_ui(self) -> None:
        header = tk.Frame(self.root, bg="#16283b", padx=24, pady=17)
        header.pack(fill="x")
        tk.Label(
            header,
            text="RM27 机械臂尺寸验证",
            bg="#16283b",
            fg="#ffffff",
            font=("TkDefaultFont", 19, "bold"),
        ).pack(anchor="w")
        tk.Label(
            header,
            text="滑块姿态实时同步到 RViz；自动搜索水平最远与竖直最高关节角",
            bg="#16283b",
            fg="#b8c8d8",
            font=("TkDefaultFont", 10),
        ).pack(anchor="w", pady=(4, 0))

        body = tk.Frame(self.root, bg=BG, padx=14, pady=14)
        body.pack(fill="both", expand=True)
        body.columnconfigure(0, weight=2, minsize=330)
        body.columnconfigure(1, weight=3, minsize=540)
        body.rowconfigure(0, weight=1)

        left = tk.Frame(body, bg=BG)
        left.grid(row=0, column=0, sticky="nsew", padx=(0, 10))
        left.columnconfigure(0, weight=1)
        self.metrics_frame = tk.Frame(left, bg=BG)
        self.metrics_frame.grid(row=1, column=0, sticky="ew", pady=(10, 0))
        self.metric_values: dict[str, tk.StringVar] = {}
        for index, (key, label) in enumerate((
            ("x", "末端 X · J1 基准"), ("y", "末端 Y · J1 基准"), ("z", "末端 Z · base 基准"), ("radial", "水平投影 XY")
        )):
            row, column = divmod(index, 2)
            self.metrics_frame.columnconfigure(column, weight=1)
            card = tk.Frame(self.metrics_frame, bg=CARD, padx=13, pady=10, highlightthickness=1, highlightbackground="#e1e7ed")
            card.grid(row=row, column=column, sticky="ew", padx=(0 if column == 0 else 5, 0), pady=(0, 5))
            tk.Label(card, text=label, bg=CARD, fg=MUTED, font=("TkDefaultFont", 9)).pack(anchor="w")
            value = tk.StringVar(value="—")
            self.metric_values[key] = value
            tk.Label(card, textvariable=value, bg=CARD, fg=INK, font=("TkDefaultFont", 15, "bold")).pack(anchor="w", pady=(3, 0))

        rviz_card = tk.Frame(left, bg=CARD, padx=15, pady=14, highlightthickness=1, highlightbackground="#e1e7ed")
        rviz_card.grid(row=0, column=0, sticky="ew")
        rviz_card.columnconfigure(0, weight=1)
        tk.Label(rviz_card, text="RViz 机械臂视图", bg=CARD, fg=INK, font=("TkDefaultFont", 12, "bold")).grid(
            row=0, column=0, sticky="w"
        )
        self.rviz_status = tk.StringVar(value=self.rviz.status())
        tk.Label(rviz_card, textvariable=self.rviz_status, bg=CARD, fg=MUTED, justify="left", wraplength=310).grid(
            row=1, column=0, sticky="ew", pady=(8, 12)
        )
        ttk.Button(rviz_card, text="重开 RViz", command=self._restart_rviz).grid(row=2, column=0, sticky="w")
        tk.Label(
            rviz_card,
            text="黄线：J0–J6 杆长投影；灰线：固定偏置。青色线：水平投影；橙色线：高度；球：tool0。坐标轴：红 X、绿 Y、蓝 Z。",
            bg=CARD,
            fg=MUTED,
            justify="left",
            wraplength=310,
        ).grid(row=3, column=0, sticky="ew", pady=(12, 0))

        right = tk.Frame(body, bg=BG)
        right.grid(row=0, column=1, sticky="nsew")
        right.columnconfigure(0, weight=1)
        right.rowconfigure(2, weight=1)

        self._build_search_card(right)
        limits_note = tk.Frame(right, bg=CARD, padx=14, pady=10, highlightthickness=1, highlightbackground="#e1e7ed")
        limits_note.grid(row=1, column=0, sticky="ew", pady=(9, 9))
        tk.Label(
            limits_note,
            text="关节控制与限位输入  ·  角度单位：°",
            bg=CARD,
            fg=INK,
            font=("TkDefaultFont", 11, "bold"),
        ).pack(anchor="w")
        tk.Label(
            limits_note,
            text="当前 MJCF 各轴范围为 ±360°；请先按 RM27 实际机械限位修改，再计算极限姿态。",
            bg=CARD,
            fg=MUTED,
            font=("TkDefaultFont", 9),
        ).pack(anchor="w", pady=(3, 0))

        controls = tk.Frame(right, bg=CARD, padx=12, pady=7, highlightthickness=1, highlightbackground="#e1e7ed")
        controls.grid(row=2, column=0, sticky="nsew")
        controls.columnconfigure(0, weight=1)
        self.control_rows: list[dict[str, object]] = []
        for index, label in enumerate(JOINT_LABELS):
            row = tk.Frame(controls, bg=CARD)
            row.pack(fill="x", pady=3)
            row.columnconfigure(1, weight=1)
            tk.Label(row, text=label, bg=CARD, fg=INK, width=10, anchor="w", font=("TkDefaultFont", 9, "bold")).grid(
                row=0, column=0, sticky="w"
            )
            lo, hi = self.limits[index]
            q_var = tk.DoubleVar(value=self.q[index] * DEG)
            scale = tk.Scale(
                row,
                variable=q_var,
                from_=lo * DEG,
                to=hi * DEG,
                orient="horizontal",
                resolution=0.1,
                showvalue=False,
                command=lambda value, joint=index: self._on_slider(joint, value),
                bg=CARD,
                activebackground="#d5eafa",
                troughcolor="#dce4eb",
                highlightthickness=0,
                bd=0,
            )
            scale.grid(row=0, column=1, sticky="ew", padx=(4, 8))
            angle_text = tk.StringVar(value=f"{self.q[index] * DEG:.1f}°")
            tk.Label(row, textvariable=angle_text, bg=CARD, fg=BLUE, width=8, anchor="e", font=("TkDefaultFont", 9, "bold")).grid(
                row=0, column=2, sticky="e"
            )

            lower_var = tk.StringVar(value=f"{lo * DEG:.1f}")
            upper_var = tk.StringVar(value=f"{hi * DEG:.1f}")
            limit_entries = []
            tk.Label(row, text="限位", bg=CARD, fg=MUTED, font=("TkDefaultFont", 8)).grid(row=1, column=0, sticky="w", pady=(0, 1))
            limits = tk.Frame(row, bg=CARD)
            limits.grid(row=1, column=1, columnspan=2, sticky="w")
            for column, (caption, variable) in enumerate((("min", lower_var), ("max", upper_var))):
                tk.Label(limits, text=caption, bg=CARD, fg=MUTED, font=("TkDefaultFont", 8)).grid(row=0, column=column * 2, padx=(0, 3))
                entry = ttk.Entry(limits, textvariable=variable, width=8, justify="right")
                entry.grid(row=0, column=column * 2 + 1, padx=(0, 9))
                entry.bind("<Return>", lambda _event, joint=index: self._commit_limits(joint))
                entry.bind("<FocusOut>", lambda _event, joint=index: self._commit_limits(joint))
                limit_entries.append(entry)
            self.control_rows.append({
                "scale": scale,
                "q_var": q_var,
                "angle": angle_text,
                "lower": lower_var,
                "upper": upper_var,
                "limit_entries": limit_entries,
            })

        bottom = tk.Frame(right, bg=BG)
        bottom.grid(row=3, column=0, sticky="ew", pady=(9, 0))
        ttk.Button(bottom, text="全零姿态", command=lambda: self._apply_joints([0.0] * 7)).pack(side="left")
        ttk.Button(bottom, text="参考姿态", command=lambda: self._apply_joints(list(HOME_JOINTS))).pack(side="left", padx=7)
        self.status_text = tk.StringVar(value="XY 基准：J1 关节轴心；Z 基准：base 高度。末端点为模型 tool0。")
        tk.Label(bottom, textvariable=self.status_text, bg=BG, fg=MUTED, anchor="w", justify="left", wraplength=430).pack(
            side="left", padx=(8, 0), fill="x", expand=True
        )

    def _build_search_card(self, parent: tk.Frame) -> None:
        card = tk.Frame(parent, bg="#e5f2f7", padx=14, pady=11, highlightthickness=1, highlightbackground="#c8e2eb")
        card.grid(row=0, column=0, sticky="ew")
        card.columnconfigure(0, weight=1)
        title = tk.Frame(card, bg="#e5f2f7")
        title.grid(row=0, column=0, sticky="ew")
        tk.Label(title, text="自动搜索最大伸展", bg="#e5f2f7", fg="#16445d", font=("TkDefaultFont", 11, "bold")).pack(side="left")
        self.search_button = ttk.Button(title, text="计算两种极限姿态", command=self._start_search)
        self.search_button.pack(side="right")
        tk.Label(
            card,
            text="按当前限位最大化水平距离（J1 XY 基准）与高度（base Z 基准）。",
            bg="#e5f2f7",
            fg="#426477",
            font=("TkDefaultFont", 9),
        ).grid(row=1, column=0, sticky="w", pady=(4, 7))
        result = tk.Frame(card, bg="#e5f2f7")
        result.grid(row=2, column=0, sticky="ew")
        result.columnconfigure(0, weight=1)
        result.columnconfigure(1, weight=1)
        self.horizontal_result = tk.StringVar(value="水平最远：尚未搜索")
        self.vertical_result = tk.StringVar(value="竖直最高：尚未搜索")
        tk.Label(result, textvariable=self.horizontal_result, bg="#e5f2f7", fg=INK, anchor="w", justify="left", font=("TkDefaultFont", 9)).grid(
            row=0, column=0, sticky="ew", padx=(0, 5)
        )
        tk.Label(result, textvariable=self.vertical_result, bg="#e5f2f7", fg=INK, anchor="w", justify="left", font=("TkDefaultFont", 9)).grid(
            row=0, column=1, sticky="ew", padx=(5, 0)
        )
        actions = tk.Frame(card, bg="#e5f2f7")
        actions.grid(row=3, column=0, sticky="ew", pady=(7, 0))
        self.apply_horizontal_button = ttk.Button(actions, text="应用水平最远姿态", command=lambda: self._apply_search("horizontal"), state="disabled")
        self.apply_horizontal_button.pack(side="left")
        self.apply_vertical_button = ttk.Button(actions, text="应用竖直最高姿态", command=lambda: self._apply_search("vertical"), state="disabled")
        self.apply_vertical_button.pack(side="left", padx=(7, 0))

    def _on_slider(self, index: int, value: str) -> None:
        angle_degrees = float(value)
        self.q[index] = angle_degrees * RAD
        self.control_rows[index]["angle"].set(f"{angle_degrees:.1f}°")
        self._schedule_update()

    def _commit_limits(self, index: int) -> None:
        row = self.control_rows[index]
        try:
            lower = float(row["lower"].get()) * RAD
            upper = float(row["upper"].get()) * RAD
            if not math.isfinite(lower) or not math.isfinite(upper) or lower >= upper:
                raise ValueError("下限必须小于上限，且两者均须为有限角度")
        except (ValueError, TypeError) as error:
            messagebox.showerror("限位输入无效", str(error), parent=self.root)
            row["lower"].set(f"{self.limits[index][0] * DEG:.1f}")
            row["upper"].set(f"{self.limits[index][1] * DEG:.1f}")
            return

        self.limits[index] = (lower, upper)
        self.search_results.clear()
        self.horizontal_result.set("水平最远：尚未搜索")
        self.vertical_result.set("竖直最高：尚未搜索")
        self.apply_horizontal_button.configure(state="disabled")
        self.apply_vertical_button.configure(state="disabled")
        row["scale"].configure(from_=lower * DEG, to=upper * DEG)
        self.q[index] = min(max(self.q[index], lower), upper)
        row["q_var"].set(self.q[index] * DEG)
        row["angle"].set(f"{self.q[index] * DEG:.1f}°")
        self.status_text.set(f"{JOINT_LABELS[index]}限位已更新；自动搜索会使用新的限位。")
        self._schedule_update()

    def _schedule_update(self) -> None:
        if self.update_job is not None:
            self.root.after_cancel(self.update_job)
        self.update_job = self.root.after(25, self._update_all)

    def _update_all(self) -> None:
        self.update_job = None
        try:
            self.positions, self.reference_offset = self.fk.positions(self.q)
            self._update_metrics()
            self.ros_bridge.update(self.q, self.positions[-1], self.reference_offset)
        except Exception as error:  # Keep the UI open so the failure is visible.
            self.status_text.set(f"正运动学错误：{error}")

    def _restart_rviz(self) -> None:
        try:
            self.rviz.start()
            self.rviz_status.set(self.rviz.status())
        except Exception as error:
            self.rviz_status.set(f"启动 RViz 失败：{error}")

    def _refresh_rviz_status(self) -> None:
        self.rviz_status.set(self.rviz.status())
        self.root.after(500, self._refresh_rviz_status)

    def _update_metrics(self) -> None:
        endpoint = self.positions[-1]
        radial = math.hypot(endpoint[0], endpoint[1])
        self.metric_values["x"].set(f"{endpoint[0] * 1000:.1f} mm")
        self.metric_values["y"].set(f"{endpoint[1] * 1000:.1f} mm")
        self.metric_values["z"].set(f"{endpoint[2] * 1000:.1f} mm")
        self.metric_values["radial"].set(f"{radial * 1000:.1f} mm")

    def _apply_joints(self, joints: list[float]) -> None:
        for index, (angle, (lower, upper)) in enumerate(zip(joints, self.limits)):
            self.q[index] = min(max(angle, lower), upper)
            row = self.control_rows[index]
            row["q_var"].set(self.q[index] * DEG)
            row["angle"].set(f"{self.q[index] * DEG:.1f}°")
        self._schedule_update()

    def _start_search(self) -> None:
        if self.searching:
            return
        bounds = list(self.limits)
        current = list(self.q)
        self.searching = True
        self.search_button.configure(state="disabled")
        self.apply_horizontal_button.configure(state="disabled")
        self.apply_vertical_button.configure(state="disabled")
        self._set_limit_entry_state("disabled")
        self.status_text.set("正在按当前限位搜索两种最大伸展姿态…")
        thread = threading.Thread(target=self._search_worker, args=(bounds, current), daemon=True)
        thread.start()

    def _search_worker(self, bounds: list[tuple[float, float]], current: list[float]) -> None:
        client: FKClient | None = None
        try:
            from scipy.optimize import differential_evolution

            client = FKClient(self.fk_executable)
            varying = [index for index, (lower, upper) in enumerate(bounds) if upper - lower > 1e-12]

            def expand(candidate: object) -> list[float]:
                joints = list(current)
                vector = list(candidate) if varying else []  # Fixed axes are omitted from the optimizer.
                for index, value in zip(varying, vector):
                    joints[index] = float(value)
                for index, (lower, upper) in enumerate(bounds):
                    joints[index] = min(max(joints[index], lower), upper)
                return joints

            def optimize(key: str, objective_index: int) -> tuple[list[float], list[tuple[float, float, float]], float]:
                self.message_queue.put(("status", f"正在搜索{'水平最远' if key == 'horizontal' else '竖直最高'}姿态…"))

                def objective(candidate: object) -> float:
                    joints = expand(candidate)
                    endpoint = client.positions(joints)[0][-1]
                    if objective_index == 0:
                        return -math.hypot(endpoint[0], endpoint[1])
                    return -endpoint[2]

                if varying:
                    result = differential_evolution(
                        objective,
                        [bounds[index] for index in varying],
                        seed=20261106,
                        maxiter=90,
                        popsize=9,
                        tol=1e-7,
                        polish=True,
                        updating="immediate",
                    )
                    best_joints = expand(result.x)
                else:
                    best_joints = list(current)
                best_positions = client.positions(best_joints)[0]
                endpoint = best_positions[-1]
                score = math.hypot(endpoint[0], endpoint[1]) if objective_index == 0 else endpoint[2]

                current_positions = client.positions(current)[0]
                current_endpoint = current_positions[-1]
                current_score = math.hypot(current_endpoint[0], current_endpoint[1]) if objective_index == 0 else current_endpoint[2]
                if current_score > score:
                    best_joints, best_positions, score = current, current_positions, current_score
                return best_joints, best_positions, score

            horizontal = optimize("horizontal", 0)
            vertical = optimize("vertical", 1)
            self.message_queue.put(("complete", {"horizontal": horizontal, "vertical": vertical}))
        except Exception as error:
            self.message_queue.put(("error", str(error)))
        finally:
            if client is not None:
                client.close()

    def _poll_messages(self) -> None:
        try:
            while True:
                kind, payload = self.message_queue.get_nowait()
                if kind == "status":
                    self.status_text.set(str(payload))
                elif kind == "complete":
                    self.search_results = payload  # type: ignore[assignment]
                    self.searching = False
                    self.search_button.configure(state="normal")
                    self.apply_horizontal_button.configure(state="normal")
                    self.apply_vertical_button.configure(state="normal")
                    self._set_limit_entry_state("normal")
                    self._format_search_results()
                    self.status_text.set("搜索完成。结果受当前限位及所用 MJCF 模型影响，可应用姿态查看关节角。")
                elif kind == "error":
                    self.searching = False
                    self.search_button.configure(state="normal")
                    self._set_limit_entry_state("normal")
                    self.status_text.set(f"搜索失败：{payload}")
                    messagebox.showerror("极限姿态搜索失败", str(payload), parent=self.root)
        except queue.Empty:
            pass
        self.root.after(80, self._poll_messages)

    def _set_limit_entry_state(self, state: str) -> None:
        for row in self.control_rows:
            for entry in row["limit_entries"]:
                entry.configure(state=state)

    def _format_search_results(self) -> None:
        for key, title in (("horizontal", "水平最远"), ("vertical", "竖直最高")):
            joints, positions, score = self.search_results[key]
            endpoint = positions[-1]
            metric = f"{score * 1000:.1f} mm"
            angles = "  ".join(f"J{index} {angle * DEG:.1f}°" for index, angle in enumerate(joints))
            angle_lines = "  ".join(angles.split("  ")[:4]) + "\n" + "  ".join(angles.split("  ")[4:])
            summary = f"{title}：{metric}\n末端 X {endpoint[0] * 1000:.1f} · Y {endpoint[1] * 1000:.1f} · Z {endpoint[2] * 1000:.1f} mm\n{angle_lines}"
            (self.horizontal_result if key == "horizontal" else self.vertical_result).set(summary)

    def _apply_search(self, key: str) -> None:
        result = self.search_results.get(key)
        if result:
            self._apply_joints(result[0])

    def close(self) -> None:
        self.fk.close()
        self.rviz.close()
        self.ros_bridge.close()
        self.root.destroy()


def main() -> int:
    executable = find_fk_executable()
    if executable is None:
        print(
            "找不到 arm_dimensions_fk。请先构建 rm26_level3_validation，再 source 对应 install/setup.bash；"
            "也可通过 RM27_ARM_DIMENSIONS_FK 指定可执行文件路径。",
            file=sys.stderr,
        )
        return 2
    try:
        root = tk.Tk()
        ArmDimensionsApp(root, executable)
        root.mainloop()
    except Exception as error:
        messagebox.showerror("RM27 尺寸验证启动失败", str(error))
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
