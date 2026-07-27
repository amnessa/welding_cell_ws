#!/usr/bin/env python3
"""Gesture control for the perception pipeline.

Drives the FoundationPose bridge and the ICP refiner from hand gestures seen by
the RealSense, so the operator can run a part end to end without touching a
keyboard on either machine.

Why the RealSense and not a second webcam
-----------------------------------------
The hand and the workpiece are then in *the same image*, which removes the only
hard part of pointing. The index fingertip lands on a RealSense pixel directly:
no calibration between two cameras, no mapping, and no mirroring -- the cursor
appears where the finger physically is, over the thing being pointed at. It also
means the operator's hand is genuinely in the scene, which is the cost: see
"Occlusion" below.

Flow
----
The click and the image it refers to have to be the same frame, so segmentation
is three steps, not one:

    ILoveYou      capture: the bridge freezes an RGB-D pair and republishes the
                  colour image; that frozen image is what this window shows and
                  what the server will receive
    Pointing_Up   move the fingertip; hold it still for `dwell_sec` and the
                  pixel latches as a *pending* point (hollow yellow)
    Thumb_Down    commit the pending point as NEGATIVE (red) -- "not this one".
                  Repeat for each neighbouring part SAM2 would otherwise grab
    Thumb_Up      commit the pending point as POSITIVE (green) and segment:
                  publish the points, call the bridge trigger, and the server
                  runs SAM2 -> PPF -> FoundationPose with no operator at its end

and outside segmentation:

    Closed_Fist   run ICP           Open_Palm   stop tracking
    Victory       save object       Thumb_Down  clear the frozen frame + points
    ILoveYou      capture (again -- re-capturing replaces a bad freeze)

Thumb_Down is the one gesture that means different things in the two modes,
because a negative click is only meaningful while points are being picked. The
mode is shown in the window so it is never ambiguous.

Occlusion
---------
The gesture that triggers the capture needs the hand in view, so the hand is in
the frozen frame. Keep it to the side of the part when signing ILoveYou. The
window shows the frozen frame itself, so a hand across the workpiece is visible
immediately -- sign ILoveYou again to re-freeze.

Model
-----
MediaPipe's gesture recognizer bundle, once:

    wget -O gesture_recognizer.task https://storage.googleapis.com/mediapipe-models/\
gesture_recognizer/gesture_recognizer/float16/1/gesture_recognizer.task

Run
---
    ros2 run admittance_control gesture_control_node.py --ros-args \
        -p model_path:=/workspaces/welding_cell_ws/ros2_ws/src/admittance_control/notebooks/gesture_recognizer.task
"""

from __future__ import annotations

import json
import time
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import Image
from std_msgs.msg import String
from std_srvs.srv import Trigger

from admittance_control.sam6d_io import image_msg_to_numpy, normalize_color_image

# The 7 gestures the stock MediaPipe bundle emits, mapped to what this pipeline
# does with them. Two of these are context-dependent -- see `Gestures` below.
GESTURE_TO_ACTION = {
    'ILoveYou':    'CAPTURE',
    'Pointing_Up': 'INDICATE',
    'Thumb_Up':    'SEGMENT',
    'Thumb_Down':  'NEGATIVE_OR_CANCEL',
    'Closed_Fist': 'RUN_ICP',
    'Open_Palm':   'STOP_ICP',
    'Victory':     'SAVE_OBJECT',
}
NEUTRAL = {None, 'None'}


@dataclass
class GestureLatch:
    """Raw per-frame gestures -> clean one-shot events.

    Two problems with the recognizer's raw output: it flickers while the hand
    moves between poses, and a held gesture repeats at frame rate. So a gesture
    must hold for `stable_frames` consecutive frames before it counts, and it
    fires once on the rising edge into that stable run.

    What re-arms it matters more than it looks. Passing through neutral does, but
    neutral cannot be *required*: the working sequence here is Pointing_Up ->
    Thumb_Up, two poses one finger apart, and the recognizer frequently slides
    straight from one to the other without ever reporting None in between. A
    neutral-only latch would swallow every thumbs-up that followed a point, which
    is the one gesture the whole segmentation flow depends on. So a stable run of
    a *different* gesture re-arms it too; `stable_frames` is what keeps the
    flicker during the transition from firing, and `cooldown_sec` is what keeps a
    held pose from repeating.
    """
    stable_frames: int = 6
    cooldown_sec: float = 0.4
    _run_gesture: Optional[str] = None
    _run_count: int = 0
    _fired_run: bool = False
    _last_fired_gesture: Optional[str] = None
    _last_fire: float = 0.0

    def update(self, raw: Optional[str]) -> Optional[str]:
        if raw == self._run_gesture:
            self._run_count += 1
        else:
            self._run_gesture = raw
            self._run_count = 1
            self._fired_run = False       # a new run has not fired yet

        if raw in NEUTRAL:
            self._last_fired_gesture = None
            return None

        action = GESTURE_TO_ACTION.get(raw)
        if action is None:
            return None
        if self._fired_run or raw == self._last_fired_gesture:
            return None                   # already acted on this pose

        stable = self._run_count >= self.stable_frames
        cooled = (time.monotonic() - self._last_fire) >= self.cooldown_sec
        if stable and cooled:
            self._fired_run = True
            self._last_fired_gesture = raw
            self._last_fire = time.monotonic()
            return action
        return None

    def progress(self) -> float:
        """0..1 toward firing, for the on-screen hold indicator.

        Worth drawing: without it the wait between posing and firing reads as the
        recognizer having missed the gesture, and the operator re-poses, which
        resets the count and makes it genuinely miss.
        """
        if self._run_gesture in NEUTRAL or self._run_gesture is None:
            return 0.0
        if GESTURE_TO_ACTION.get(self._run_gesture) is None:
            return 0.0
        if self._fired_run or self._run_gesture == self._last_fired_gesture:
            return 0.0
        return min(1.0, self._run_count / max(1, self.stable_frames))


@dataclass
class DwellCursor:
    """Fingertip -> a latched pixel, by holding still.

    A gesture cannot also carry a "click now" signal, so stillness is the signal.
    Movement beyond `move_tol_px` restarts the timer; `dwell_sec` of quiet
    latches the pixel as pending.
    """
    dwell_sec: float = 0.8
    move_tol_px: int = 18
    _anchor: Optional[Tuple[int, int]] = None
    _since: float = 0.0
    pending: Optional[Tuple[int, int]] = None

    def update(self, tip: Optional[Tuple[int, int]], active: bool) -> None:
        if not active or tip is None:
            self._anchor = None
            return
        if (self._anchor is None
                or abs(tip[0] - self._anchor[0]) > self.move_tol_px
                or abs(tip[1] - self._anchor[1]) > self.move_tol_px):
            self._anchor = tip
            self._since = time.monotonic()
        elif time.monotonic() - self._since >= self.dwell_sec:
            self.pending = tip

    def progress(self, active: bool) -> float:
        if not active or self._anchor is None:
            return 0.0
        return min(1.0, (time.monotonic() - self._since) / max(1e-6, self.dwell_sec))

    def clear(self) -> None:
        self._anchor = None
        self.pending = None


@dataclass
class PickedPoints:
    """The points picked on one frozen frame, and that frame's stamp.

    The stamp travels with the points all the way to the bridge, which refuses
    any click that does not match the frame it is holding. That is what stops a
    pixel picked on one image from being segmented on another.
    """
    stamp_ns: int = 0
    points: List[List[int]] = field(default_factory=list)
    labels: List[int] = field(default_factory=list)

    def add(self, uv: Tuple[int, int], label: int) -> None:
        self.points.append([int(uv[0]), int(uv[1])])
        self.labels.append(int(label))

    def clear(self) -> None:
        self.points.clear()
        self.labels.clear()

    def to_json(self) -> str:
        return json.dumps({'stamp_ns': self.stamp_ns,
                           'points': self.points,
                           'labels': self.labels}, separators=(',', ':'))


class GestureControlNode(Node):
    def __init__(self) -> None:
        super().__init__('gesture_control')

        self.declare_parameter('model_path', 'gesture_recognizer.task')
        self.declare_parameter('rgb_topic', '/camera/color/image_raw')
        self.declare_parameter('frozen_rgb_topic', '/perception/frozen_rgb')
        self.declare_parameter('click_topic', '/perception/sam2_click')
        self.declare_parameter('bridge_ns', '/foundationpose_bridge')
        self.declare_parameter('icp_ns', '/icp_pose_refiner')
        # Higher = fewer misfires, more lag before a gesture takes.
        self.declare_parameter('stable_frames', 6)
        # Short on purpose. Repeats are already blocked by the latch tracking which
        # pose it last acted on, so this only has to absorb recognizer jitter --
        # and a long cooldown would swallow the thumbs-up that follows a dwell,
        # which arrives about a second after the point that started it.
        self.declare_parameter('cooldown_sec', 0.4)
        self.declare_parameter('dwell_sec', 0.8)
        self.declare_parameter('move_tol_px', 18)
        self.declare_parameter('min_hand_confidence', 0.5)
        # The window is only a viewport; picked coordinates are always full-res
        # camera pixels, so scaling it down cannot move a click.
        self.declare_parameter('display_scale', 1.0)
        self.declare_parameter('show_window', True)

        self._model_path = str(self.get_parameter('model_path').value)
        self._display_scale = float(self.get_parameter('display_scale').value)
        self._show_window = bool(self.get_parameter('show_window').value)
        bridge_ns = str(self.get_parameter('bridge_ns').value).rstrip('/')
        icp_ns = str(self.get_parameter('icp_ns').value).rstrip('/')

        self._latch = GestureLatch(
            stable_frames=int(self.get_parameter('stable_frames').value),
            cooldown_sec=float(self.get_parameter('cooldown_sec').value))
        self._cursor = DwellCursor(
            dwell_sec=float(self.get_parameter('dwell_sec').value),
            move_tol_px=int(self.get_parameter('move_tol_px').value))
        self._picked = PickedPoints()

        self._live_bgr: Optional[np.ndarray] = None
        self._frozen_bgr: Optional[np.ndarray] = None
        self._frozen_stamp_ns: int = 0
        self._status = 'waiting for the camera'
        self._status_until = 0.0
        self._pending_calls: Dict[str, object] = {}
        self._pending_trigger_at = 0.0

        self.create_subscription(
            Image, str(self.get_parameter('rgb_topic').value), self._on_rgb, 1)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(
            Image, str(self.get_parameter('frozen_rgb_topic').value),
            self._on_frozen, latched)
        self._click_pub = self.create_publisher(
            String, str(self.get_parameter('click_topic').value), 10)

        # NOT `self._clients`: rclpy.Node keeps its own list of service clients
        # under that exact name and its executor iterates it every spin, so
        # shadowing it takes the node down on the first spin_once.
        self._triggers = {
            'CAPTURE':     self.create_client(Trigger, f'{bridge_ns}/capture'),
            'TRIGGER':     self.create_client(Trigger, f'{bridge_ns}/trigger'),
            'CLEAR_CLICK': self.create_client(Trigger, f'{bridge_ns}/clear_click'),
            'RUN_ICP':     self.create_client(Trigger, f'{icp_ns}/run_icp'),
            'STOP_ICP':    self.create_client(Trigger, f'{icp_ns}/stop_tracking'),
            'SAVE_OBJECT': self.create_client(Trigger, f'{icp_ns}/save_object'),
        }

        self._recognizer = self._build_recognizer()
        self._t0 = time.monotonic()
        self.get_logger().info(
            f'gesture control ready (model={self._model_path}); '
            f'ILoveYou=capture  Pointing_Up=cursor  Thumb_Down=negative/cancel  '
            f'Thumb_Up=segment  Fist=run ICP  Palm=stop  Victory=save')

    # ── MediaPipe ────────────────────────────────────────────────────────
    def _build_recognizer(self):
        import mediapipe as mp
        self._mp = mp
        base = mp.tasks.BaseOptions
        options = mp.tasks.vision.GestureRecognizerOptions(
            base_options=base(model_asset_path=self._model_path),
            running_mode=mp.tasks.vision.RunningMode.VIDEO,
            num_hands=1,                       # one operator hand, no ambiguity
            min_hand_detection_confidence=float(
                self.get_parameter('min_hand_confidence').value),
            min_tracking_confidence=float(
                self.get_parameter('min_hand_confidence').value),
        )
        return mp.tasks.vision.GestureRecognizer.create_from_options(options)

    def _recognize(self, bgr: np.ndarray):
        """(top gesture name, index fingertip pixel) for one frame."""
        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        image = self._mp.Image(image_format=self._mp.ImageFormat.SRGB, data=rgb)
        result = self._recognizer.recognize_for_video(
            image, int((time.monotonic() - self._t0) * 1000))

        top = None
        score = 0.0
        if result.gestures:
            top = result.gestures[0][0].category_name
            score = result.gestures[0][0].score
        tip = None
        if result.hand_landmarks:
            height, width = bgr.shape[:2]
            landmark = result.hand_landmarks[0][8]        # index fingertip
            tip = (int(round(landmark.x * width)), int(round(landmark.y * height)))
        return top, score, tip

    # ── Subscriptions ────────────────────────────────────────────────────
    def _on_rgb(self, msg: Image) -> None:
        try:
            self._live_bgr = normalize_color_image(image_msg_to_numpy(msg), msg.encoding)
        except ValueError as exc:
            self.get_logger().warn(f'cannot read the colour image: {exc}',
                                   throttle_duration_sec=10.0)

    def _on_frozen(self, msg: Image) -> None:
        try:
            frozen = normalize_color_image(image_msg_to_numpy(msg), msg.encoding)
        except ValueError as exc:
            self.get_logger().warn(f'cannot read the frozen image: {exc}')
            return
        stamp_ns = int(msg.header.stamp.sec) * 1_000_000_000 + int(msg.header.stamp.nanosec)
        if stamp_ns == self._frozen_stamp_ns:
            return                                        # a re-latched republish
        self._frozen_bgr = frozen
        self._frozen_stamp_ns = stamp_ns
        # A new frozen frame invalidates every pixel picked on the old one.
        self._picked = PickedPoints(stamp_ns=stamp_ns)
        self._cursor.clear()
        self._say(f'frozen {frozen.shape[1]}x{frozen.shape[0]} -- point at the part')

    # ── Actions ──────────────────────────────────────────────────────────
    def _say(self, message: str, seconds: float = 4.0) -> None:
        self._status = message
        self._status_until = time.monotonic() + seconds
        self.get_logger().info(message)

    def _call(self, name: str) -> None:
        """Fire a Trigger and carry on drawing.

        Deliberately async: ~/trigger blocks for as long as FoundationPose takes,
        and a frozen UI during it would leave the operator with no idea whether
        the gesture registered.
        """
        client = self._triggers[name]
        if not client.service_is_ready():
            self._say(f'{name}: {client.srv_name} is not available')
            return
        if name in self._pending_calls and not self._pending_calls[name].done():
            self._say(f'{name}: still waiting on the previous call')
            return
        future = client.call_async(Trigger.Request())
        self._pending_calls[name] = future

        def _done(fut, label=name):
            try:
                reply = fut.result()
            except Exception as exc:  # noqa: BLE001 - a failed call is a status, not a crash
                self._say(f'{label} failed: {exc}', 6.0)
                return
            self._say(f'{label}: {reply.message}'[:160], 6.0)
            # The bridge drops its frozen frame once a pose is registered, and on
            # an explicit clear. Follow it, or this window keeps showing a spent
            # frame and Thumb_Down keeps meaning "negative point" for a
            # segmentation that is already finished.
            if label == 'CLEAR_CLICK' or (label == 'TRIGGER' and reply.success):
                self._release_frozen()

        future.add_done_callback(_done)

    def _release_frozen(self) -> None:
        self._frozen_bgr = None
        self._frozen_stamp_ns = 0
        self._picked = PickedPoints()
        self._cursor.clear()

    @property
    def _segmenting(self) -> bool:
        """True once a frame is frozen: the mode where Thumb_Down means 'not this'."""
        return self._frozen_bgr is not None

    def _dispatch(self, action: str) -> None:
        if action == 'CAPTURE':
            self._call('CAPTURE')
        elif action == 'RUN_ICP':
            self._call('RUN_ICP')
        elif action == 'STOP_ICP':
            self._call('STOP_ICP')
        elif action == 'SAVE_OBJECT':
            self._call('SAVE_OBJECT')
        elif action == 'INDICATE':
            self._say('cursor mode -- hold still to latch a point', 2.0)
        elif action == 'NEGATIVE_OR_CANCEL':
            self._on_thumb_down()
        elif action == 'SEGMENT':
            self._on_segment()

    def _on_thumb_down(self) -> None:
        """Negative point while segmenting, cancel otherwise."""
        if not self._segmenting:
            self._release_frozen()
            self._call('CLEAR_CLICK')
            return
        if self._cursor.pending is None:
            self._say('nothing to mark -- point at it and hold still first')
            return
        self._picked.add(self._cursor.pending, 0)
        self._say(f'not-this-one at {self._cursor.pending} '
                  f'({self._picked.labels.count(0)} negative)')
        self._cursor.pending = None

    def _on_segment(self) -> None:
        """Commit the pending point as positive, publish, and trigger the bridge."""
        if not self._segmenting:
            self._say('nothing frozen -- sign ILoveYou to capture a frame first')
            return
        if self._cursor.pending is None:
            self._say('point at the part and hold still before thumbs-up')
            return

        self._picked.add(self._cursor.pending, 1)
        self._cursor.pending = None
        self._click_pub.publish(String(data=self._picked.to_json()))
        n_neg = self._picked.labels.count(0)
        self._say(f'segmenting: {len(self._picked.labels) - n_neg} positive, '
                  f'{n_neg} negative', 8.0)

        # The bridge validates the click against the frame it is holding before it
        # sends anything, so publish-then-trigger is safe; it rejects rather than
        # sends a mismatched pair. Give the message a moment to land first.
        self._pending_trigger_at = time.monotonic() + 0.15

    # ── Drawing ──────────────────────────────────────────────────────────
    def _render(self, gesture: Optional[str], score: float,
                tip: Optional[Tuple[int, int]], indicating: bool) -> np.ndarray:
        # Draw on the frozen frame while segmenting: the operator has to see the
        # exact pixels the server will see, hand and all.
        base = self._frozen_bgr if self._segmenting else self._live_bgr
        vis = base.copy()
        height, width = vis.shape[:2]

        for (u, v), label in zip(self._picked.points, self._picked.labels):
            colour = (0, 200, 0) if label else (0, 0, 255)
            cv2.circle(vis, (u, v), 7, colour, -1)
            cv2.circle(vis, (u, v), 8, (255, 255, 255), 1)

        if indicating and tip is not None:
            cv2.circle(vis, tip, 12, (0, 255, 255), 2)
            held = self._cursor.progress(indicating)
            if held > 0:
                cv2.ellipse(vis, tip, (18, 18), -90, 0, int(360 * held),
                            (0, 255, 255), 3)
        if self._cursor.pending is not None:
            cv2.circle(vis, self._cursor.pending, 10, (0, 255, 255), 2)
            cv2.putText(vis, 'pending', (self._cursor.pending[0] + 14,
                                         self._cursor.pending[1] - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 2)

        mode = 'SEGMENTING (Thumb_Down = not-this)' if self._segmenting \
            else 'IDLE (ILoveYou = capture)'
        cv2.rectangle(vis, (0, 0), (width, 34), (0, 0, 0), -1)
        cv2.putText(vis, mode, (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                    (255, 255, 255), 2)
        if gesture:
            cv2.putText(vis, f'{gesture} {score:.2f}', (10, 70),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
            hold = self._latch.progress()
            if hold > 0:
                cv2.rectangle(vis, (10, 80), (10 + int(200 * hold), 92),
                              (0, 255, 0), -1)
                cv2.rectangle(vis, (10, 80), (210, 92), (255, 255, 255), 1)
        if time.monotonic() < self._status_until:
            cv2.rectangle(vis, (0, height - 34), (width, height), (0, 0, 0), -1)
            cv2.putText(vis, self._status[:110], (10, height - 11),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1)

        if self._display_scale != 1.0:
            vis = cv2.resize(vis, None, fx=self._display_scale, fy=self._display_scale)
        return vis

    # ── Main loop ────────────────────────────────────────────────────────
    def spin_forever(self) -> None:
        window = 'gesture control  (q to quit)'
        if self._show_window:
            cv2.namedWindow(window, cv2.WINDOW_NORMAL)
        try:
            while rclpy.ok():
                rclpy.spin_once(self, timeout_sec=0.005)
                if self._live_bgr is None:
                    continue

                gesture, score, tip = self._recognize(self._live_bgr)

                if self._segmenting and self._frozen_bgr.shape[:2] != self._live_bgr.shape[:2]:
                    # Fingertip pixels are only transferable between the two feeds
                    # while they are the same size. They come from one camera, so
                    # this means someone reconfigured it mid-run.
                    self.get_logger().error(
                        f'live {self._live_bgr.shape[:2]} and frozen '
                        f'{self._frozen_bgr.shape[:2]} differ in size; points would '
                        f'land on the wrong pixels. Re-capture after the stream '
                        f'settles.', throttle_duration_sec=5.0)
                    tip = None

                indicating = (gesture == 'Pointing_Up') and self._segmenting
                self._cursor.update(tip, indicating)

                action = self._latch.update(gesture)
                if action:
                    self._dispatch(action)

                # Deferred so the click message is on the wire before the trigger
                # that consumes it.
                if self._pending_trigger_at and time.monotonic() >= self._pending_trigger_at:
                    self._pending_trigger_at = 0.0
                    self._call('TRIGGER')

                if self._show_window:
                    cv2.imshow(window, self._render(gesture, score, tip, indicating))
                    if cv2.waitKey(1) & 0xFF == ord('q'):
                        break
        finally:
            if self._show_window:
                cv2.destroyAllWindows()
                cv2.waitKey(1)


def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node = GestureControlNode()
    try:
        node.spin_forever()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
