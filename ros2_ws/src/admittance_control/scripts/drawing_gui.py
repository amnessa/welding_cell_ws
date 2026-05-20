#!/usr/bin/env python3
"""
Sand Drawer — PyQt5 + Matplotlib 3D Trajectory Visualizer

Standalone GUI that subscribes to /visualizer/drawing_path (nav_msgs/Path)
and /visualizer/safe_probe_targets (geometry_msgs/PoseArray). Displays:
  • Robot base link origin (red dot)
  • Table rectangle from the plane JSON (translucent cyan)
  • Each incoming trajectory in magenta (updated live)
    • Safe probe targets as numbered amber markers

The ROS 2 node runs on a background QThread so the GUI stays responsive.

Usage:
  # Terminal 1 — launch the drawing pipeline
  ros2 launch sand_drawer sand_drawer.launch.py mode:=action continuous:=true

  # Terminal 2 — launch the GUI (system Python with ROS sourced)
  python3 drawing_gui.py                             # auto-finds JSON
  python3 drawing_gui.py /path/to/plane.json         # explicit path
"""

import json
import os
import sys
from pathlib import Path

import numpy as np

import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseArray
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from nav_msgs.msg import Path as NavPath

from PyQt5.QtWidgets import (QApplication, QMainWindow, QVBoxLayout,
                              QWidget, QLabel, QHBoxLayout, QPushButton)
from PyQt5.QtCore import pyqtSignal, QThread, pyqtSlot, Qt

from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
from mpl_toolkits.mplot3d.art3d import Poly3DCollection


# ─────────────────────────────────────────────────────────────────────────
# Background ROS 2 Thread
# ─────────────────────────────────────────────────────────────────────────

class ROS2ListenerThread(QThread):
    """Spins a ROS 2 node on a background thread and emits Qt signals."""

    path_received = pyqtSignal(list, list, list)   # xs, ys, zs
    probe_targets_received = pyqtSignal(object)

    def __init__(self):
        super().__init__()
        self.node = None
        self._running = True

    def run(self):
        rclpy.init()
        self.node = rclpy.create_node('drawing_gui_listener')
        _qos = QoSProfile(
            depth=10,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.node.create_subscription(
            NavPath, '/visualizer/drawing_path', self._path_cb, _qos)
        self.node.create_subscription(
            PoseArray, '/visualizer/safe_probe_targets', self._probe_targets_cb, _qos)
        self.node.get_logger().info(
            'GUI visualizer listening on /visualizer/drawing_path and /visualizer/safe_probe_targets …')

        while self._running and rclpy.ok():
            rclpy.spin_once(self.node, timeout_sec=0.05)

        self.node.destroy_node()
        rclpy.try_shutdown()

    def _path_cb(self, msg: NavPath):
        xs = [p.pose.position.x for p in msg.poses]
        ys = [p.pose.position.y for p in msg.poses]
        zs = [p.pose.position.z for p in msg.poses]
        self.path_received.emit(xs, ys, zs)

    def _probe_targets_cb(self, msg: PoseArray):
        targets = [
            {
                'index': index + 1,
                'x': pose.position.x,
                'y': pose.position.y,
                'z': pose.position.z,
            }
            for index, pose in enumerate(msg.poses)
        ]
        self.probe_targets_received.emit(targets)

    def stop(self):
        self._running = False


# ─────────────────────────────────────────────────────────────────────────
# Main PyQt Window
# ─────────────────────────────────────────────────────────────────────────

class DrawingVisualizerGUI(QMainWindow):
    """3D visualizer for sand-drawer trajectories."""

    def __init__(self, plane_json_path: str):
        super().__init__()
        self.setWindowTitle('Sand Drawer — Trajectory Visualizer')
        self.resize(900, 750)

        # ── Central layout ──
        central = QWidget()
        self.setCentralWidget(central)
        layout = QVBoxLayout(central)

        # Top bar: status + clear button
        top_bar = QHBoxLayout()
        self.status_label = QLabel('Waiting for drawing trajectory …')
        self.status_label.setStyleSheet('font-size: 13px;')
        top_bar.addWidget(self.status_label, stretch=1)

        clear_btn = QPushButton('Clear Drawings')
        clear_btn.clicked.connect(self._clear_drawings)
        top_bar.addWidget(clear_btn)

        layout.addLayout(top_bar)

        # ── 3D Matplotlib canvas ──
        self.figure = Figure(facecolor='#2b2b2b')
        self.canvas = FigureCanvas(self.figure)
        layout.addWidget(self.canvas)

        self.ax = self.figure.add_subplot(111, projection='3d',
                                          facecolor='#2b2b2b')

        # ── State ──
        self._trajectory_artists = []   # list of Line3D artists
        self._start_markers = []        # scatter artists
        self._probe_artists = []        # probe markers, guide lines, labels
        self._drawing_count = 0
        self._probe_target_count = 0
        self._plane_json_path = plane_json_path
        self._plane_normal = np.array([0.0, 0.0, 1.0], dtype=float)
        self._path_display_offset_m = 0.002
        self._probe_display_offset_m = 0.003

        # ── Draw the static environment ──
        self._draw_environment()

        # ── Start ROS listener ──
        self._ros_thread = ROS2ListenerThread()
        self._ros_thread.path_received.connect(self._on_new_path)
        self._ros_thread.probe_targets_received.connect(self._on_probe_targets)
        self._ros_thread.start()

    # ------------------------------------------------------------------
    # Static environment (robot base + table)
    # ------------------------------------------------------------------
    def _draw_environment(self):
        ax = self.ax
        ax.clear()

        # Style
        ax.set_title('Robot Workspace', color='white', fontsize=14)
        for axis_label, setter in [('X (m)', ax.set_xlabel),
                                   ('Y (m)', ax.set_ylabel),
                                   ('Z (m)', ax.set_zlabel)]:
            setter(axis_label, color='white')
        ax.tick_params(colors='grey')

        # Robot base
        ax.scatter(0, 0, 0, color='red', s=120, depthshade=False,
                   label='Base Link', zorder=10)
        ax.text(0, 0, 0.04, 'Base', color='red', fontsize=9)

        # Load + draw the table rectangle
        try:
            with open(self._plane_json_path, 'r') as f:
                data = json.load(f)

            corners = np.array(data['rectangle_corners'], dtype=float)
            # Rz(π) correction if target_frame == 'base'
            if data.get('target_frame', 'base_link') == 'base':
                corners = np.array([[-c[0], -c[1], c[2]] for c in corners])

            # Translucent table polygon
            verts = [list(zip(corners[:, 0], corners[:, 1], corners[:, 2]))]
            poly = Poly3DCollection(verts, alpha=0.25,
                                    facecolor='cyan', edgecolor='dodgerblue',
                                    linewidths=1.5)
            ax.add_collection3d(poly)

            # Corner labels
            for i, c in enumerate(corners):
                ax.text(c[0], c[1], c[2] + 0.01, f'C{i}',
                        color='dodgerblue', fontsize=8)

            # Plane normal arrow (from centre)
            centre = corners.mean(axis=0)
            normal = np.array(data['plane']['normal'], dtype=float)
            if data.get('target_frame', 'base_link') == 'base':
                normal = np.array([-normal[0], -normal[1], normal[2]])
            normal_norm = float(np.linalg.norm(normal))
            if normal_norm > 1e-9:
                self._plane_normal = normal / normal_norm
            ax.quiver(*centre, *(normal * 0.05),
                      color='lime', arrow_length_ratio=0.3, linewidth=1.5,
                      label='Normal')

            # Auto-scale from table + base
            all_pts = np.vstack([corners, [[0, 0, 0]]])
            margin = 0.15
            ax.set_xlim(all_pts[:, 0].min() - margin,
                        all_pts[:, 0].max() + margin)
            ax.set_ylim(all_pts[:, 1].min() - margin,
                        all_pts[:, 1].max() + margin)
            z_lo = min(0, all_pts[:, 2].min()) - 0.05
            z_hi = all_pts[:, 2].max() + 0.20
            ax.set_zlim(z_lo, z_hi)

        except Exception as e:
            self.status_label.setText(f'⚠ Failed to load JSON: {e}')

        ax.legend(loc='upper left', fontsize=8, framealpha=0.5)
        self.canvas.draw_idle()

    # ------------------------------------------------------------------
    # Incoming trajectory
    # ------------------------------------------------------------------
    @pyqtSlot(list, list, list)
    def _on_new_path(self, xs, ys, zs):
        self._drawing_count += 1

        xs_arr = np.asarray(xs, dtype=float)
        ys_arr = np.asarray(ys, dtype=float)
        zs_arr = np.asarray(zs, dtype=float)
        display_offset = -self._plane_normal * self._path_display_offset_m
        xs_plot = xs_arr + display_offset[0]
        ys_plot = ys_arr + display_offset[1]
        zs_plot = zs_arr + display_offset[2]

        # Draw the trajectory
        line, = self.ax.plot(xs_plot, ys_plot, zs_plot,
                             color='magenta', linewidth=2.5,
                             alpha=0.85)
        self._trajectory_artists.append(line)

        # Start-point marker
        sc = self.ax.scatter(xs_plot[0], ys_plot[0], zs_plot[0], color='lime', s=40,
                             depthshade=False)
        self._start_markers.append(sc)

        self.status_label.setText(
            f'Drawing #{self._drawing_count}: {len(xs)} waypoints')
        self.canvas.draw_idle()

    @pyqtSlot(object)
    def _on_probe_targets(self, targets):
        self._clear_probe_targets()
        if not targets:
            self.status_label.setText('No safe probe targets received.')
            self.canvas.draw_idle()
            return

        self._probe_target_count = len(targets)
        probe_offset = self._plane_normal * self._probe_display_offset_m
        xs = np.asarray([target['x'] for target in targets], dtype=float) + probe_offset[0]
        ys = np.asarray([target['y'] for target in targets], dtype=float) + probe_offset[1]
        zs = np.asarray([target['z'] for target in targets], dtype=float) + probe_offset[2]

        probe_line, = self.ax.plot(
            xs, ys, zs,
            color='orange', linewidth=1.5, alpha=0.75, linestyle='--')
        self._probe_artists.append(probe_line)

        probe_scatter = self.ax.scatter(
            xs, ys, zs,
            color='orange', edgecolors='black', s=55, depthshade=False)
        self._probe_artists.append(probe_scatter)

        for index, (x_pos, y_pos, z_pos) in enumerate(zip(xs, ys, zs), start=1):
            label = self.ax.text(
                x_pos, y_pos, z_pos + 0.005,
                f'P{index}', color='white', fontsize=8)
            self._probe_artists.append(label)

        self.status_label.setText(
            f'Safe probe targets: {self._probe_target_count} points')
        self.canvas.draw_idle()

    # ------------------------------------------------------------------
    # Clear button
    # ------------------------------------------------------------------
    def _clear_drawings(self):
        for artist in self._trajectory_artists:
            try:
                artist.remove()
            except Exception:
                pass
        for sc in self._start_markers:
            try:
                sc.remove()
            except Exception:
                pass
        self._trajectory_artists.clear()
        self._start_markers.clear()
        self._clear_probe_targets()
        self._drawing_count = 0
        self.status_label.setText('Cleared — waiting for new trajectory …')
        self.canvas.draw_idle()

    def _clear_probe_targets(self):
        for artist in self._probe_artists:
            try:
                artist.remove()
            except Exception:
                pass
        self._probe_artists.clear()
        self._probe_target_count = 0

    # ------------------------------------------------------------------
    # Shutdown
    # ------------------------------------------------------------------
    def closeEvent(self, a0):
        self._ros_thread.stop()
        self._ros_thread.wait(3000)
        super().closeEvent(a0)


# ═══════════════════════════════════════════════════════════════════════════
# Entry point
# ═══════════════════════════════════════════════════════════════════════════

def main():
    # Determine plane JSON path
    if len(sys.argv) > 1:
        json_path = sys.argv[1]
    else:
        source_package_root = Path(__file__).resolve().parents[1]
        source_plane_json = source_package_root / 'generated_planes' / 'sand_drawer_plane.json'
        if source_plane_json.exists():
            json_path = str(source_plane_json)
        else:
            json_path = os.path.join(
                get_package_share_directory('sand_drawer'),
                'generated_planes', 'sand_drawer_plane.json')

    if not os.path.exists(json_path):
        print(f'ERROR: Plane JSON not found: {json_path}', file=sys.stderr)
        print('Usage: python3 drawing_gui.py [path/to/plane.json]',
              file=sys.stderr)
        sys.exit(1)

    app = QApplication(sys.argv)

    # Dark palette (optional — matches dark IDEs)
    from PyQt5.QtGui import QPalette, QColor
    palette = QPalette()
    palette.setColor(QPalette.Window, QColor(43, 43, 43))
    palette.setColor(QPalette.WindowText, QColor(220, 220, 220))
    palette.setColor(QPalette.Base, QColor(35, 35, 35))
    palette.setColor(QPalette.Text, QColor(220, 220, 220))
    palette.setColor(QPalette.Button, QColor(53, 53, 53))
    palette.setColor(QPalette.ButtonText, QColor(220, 220, 220))
    app.setPalette(palette)

    gui = DrawingVisualizerGUI(json_path)
    gui.show()
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()
