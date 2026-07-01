"""Pose integrator node for the SpaceMouse.

Subscribes to the normalized 6-DOF axes published by the ``spacenav`` driver
(``spacenav/joy``) and integrates them, at a fixed configurable rate, into two
poses:

* ``spacemouse/delta_pose`` — the incremental motion produced in one tick.
* ``spacemouse/curr_pose``  — the running accumulation of those deltas.

Both are ``geometry_msgs/msg/PoseStamped`` (translation vector + quaternion).
``spacemouse/set_pose`` (PoseStamped) explicitly resets ``curr_pose``.

This node is the core "pose output" function and runs independently of the web
dashboard.
"""
import math
import threading

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult

from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Joy

try:
    from tf2_ros import TransformBroadcaster
    from geometry_msgs.msg import TransformStamped
    _HAS_TF2 = True
except ImportError:  # tf2_ros is optional (only needed when publish_tf=True)
    _HAS_TF2 = False


# ─── Quaternion helpers, all using the (w, x, y, z) convention ─────────────
def quat_mul(a, b):
    """Hamilton product a ⊗ b, quaternions as (w, x, y, z)."""
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return (
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    )


def quat_normalize(q):
    w, x, y, z = q
    n = math.sqrt(w * w + x * x + y * y + z * z)
    if n < 1e-12:
        return (1.0, 0.0, 0.0, 0.0)
    return (w / n, x / n, y / n, z / n)


def quat_rotate_vec(q, v):
    """Rotate 3-vector v by quaternion q (w, x, y, z)."""
    w, x, y, z = q
    vx, vy, vz = v
    tx = 2.0 * (y * vz - z * vy)
    ty = 2.0 * (z * vx - x * vz)
    tz = 2.0 * (x * vy - y * vx)
    return (
        vx + w * tx + (y * tz - z * ty),
        vy + w * ty + (z * tx - x * tz),
        vz + w * tz + (x * ty - y * tx),
    )


def rotvec_to_quat(rx, ry, rz):
    """Convert a rotation vector (axis * angle, radians) to (w, x, y, z)."""
    theta = math.sqrt(rx * rx + ry * ry + rz * rz)
    if theta < 1e-12:
        return (1.0, 0.0, 0.0, 0.0)
    half = theta * 0.5
    s = math.sin(half) / theta
    return (math.cos(half), rx * s, ry * s, rz * s)


def _clamp(v, lo, hi):
    return lo if v < lo else hi if v > hi else v


class PoseNode(Node):
    def __init__(self):
        super().__init__('pose_node')

        self._lock = threading.Lock()

        # ── Parameters ──
        self._freq = float(
            self.declare_parameter('publish_frequency', 100.0).value)
        self._max_trans = float(
            self.declare_parameter('max_trans_speed', 0.1).value)
        self._max_rot = float(
            self.declare_parameter('max_rot_speed', 1.0).value)
        self._frame = str(
            self.declare_parameter('integration_frame', 'world').value)
        self._frame_id = str(
            self.declare_parameter('pose_frame_id', 'spacenav_origin').value)
        self._input_topic = str(
            self.declare_parameter('input_topic', 'spacenav/joy').value)
        self._deadzone = float(
            self.declare_parameter('deadzone', 0.0).value)
        self._input_timeout = float(
            self.declare_parameter('input_timeout', 0.5).value)
        self._publish_tf = bool(
            self.declare_parameter('publish_tf', False).value)
        self._publish_curr = bool(
            self.declare_parameter('publish_curr_pose', True).value)
        self._publish_delta = bool(
            self.declare_parameter('publish_delta_pose', True).value)

        # ── State ──
        self._p = [0.0, 0.0, 0.0]          # accumulated position
        self._q = (1.0, 0.0, 0.0, 0.0)     # accumulated orientation (w,x,y,z)
        self._axes = [0.0] * 6             # latest normalized axes
        self._last_input = self.get_clock().now()

        # ── Interfaces ──
        self._pub_curr = self.create_publisher(
            PoseStamped, 'spacemouse/curr_pose', 10)
        self._pub_delta = self.create_publisher(
            PoseStamped, 'spacemouse/delta_pose', 10)
        self.create_subscription(Joy, self._input_topic, self._on_joy, 10)
        self.create_subscription(
            PoseStamped, 'spacemouse/set_pose', self._on_set_pose, 10)

        self._tf_broadcaster = None
        if self._publish_tf:
            if _HAS_TF2:
                self._tf_broadcaster = TransformBroadcaster(self)
            else:
                self.get_logger().warn(
                    'publish_tf requested but tf2_ros is not available')

        self._timer = None
        self._make_timer()

        self.add_on_set_parameters_callback(self._on_params)

        self.get_logger().info(
            f'pose_node up: {self._freq:.1f} Hz, '
            f'max_trans={self._max_trans} m/s, max_rot={self._max_rot} rad/s, '
            f"frame='{self._frame}', input='{self._input_topic}'")

    # ── Timer management ──
    def _make_timer(self):
        if self._timer is not None:
            self.destroy_timer(self._timer)
        period = 1.0 / max(self._freq, 1e-3)
        self._timer = self.create_timer(period, self._on_tick)

    # ── Subscriptions ──
    def _on_joy(self, msg: Joy):
        if len(msg.axes) < 6:
            return
        with self._lock:
            self._axes = [float(a) for a in msg.axes[:6]]
            self._last_input = self.get_clock().now()

    def _on_set_pose(self, msg: PoseStamped):
        q = (msg.pose.orientation.w, msg.pose.orientation.x,
             msg.pose.orientation.y, msg.pose.orientation.z)
        with self._lock:
            self._p = [msg.pose.position.x,
                       msg.pose.position.y,
                       msg.pose.position.z]
            self._q = quat_normalize(q)
        self.get_logger().info(
            f'curr_pose set to p=({self._p[0]:.3f}, {self._p[1]:.3f}, '
            f'{self._p[2]:.3f})')

    # ── Parameter updates (runtime adjustable) ──
    def _on_params(self, params):
        new_freq = None
        for p in params:
            try:
                if p.name == 'publish_frequency':
                    f = float(p.value)
                    if f <= 0.0:
                        return SetParametersResult(
                            successful=False,
                            reason='publish_frequency must be > 0')
                    new_freq = f
                elif p.name == 'max_trans_speed':
                    v = float(p.value)
                    if v < 0.0:
                        return SetParametersResult(
                            successful=False,
                            reason='max_trans_speed must be >= 0')
                    with self._lock:
                        self._max_trans = v
                elif p.name == 'max_rot_speed':
                    v = float(p.value)
                    if v < 0.0:
                        return SetParametersResult(
                            successful=False,
                            reason='max_rot_speed must be >= 0')
                    with self._lock:
                        self._max_rot = v
                elif p.name == 'integration_frame':
                    s = str(p.value)
                    if s not in ('body', 'world'):
                        return SetParametersResult(
                            successful=False,
                            reason="integration_frame must be 'body' or 'world'")
                    with self._lock:
                        self._frame = s
                elif p.name == 'pose_frame_id':
                    with self._lock:
                        self._frame_id = str(p.value)
                elif p.name == 'deadzone':
                    with self._lock:
                        self._deadzone = float(p.value)
                elif p.name == 'input_timeout':
                    with self._lock:
                        self._input_timeout = float(p.value)
                elif p.name == 'publish_tf':
                    on = bool(p.value)
                    with self._lock:
                        self._publish_tf = on
                    if on and self._tf_broadcaster is None and _HAS_TF2:
                        self._tf_broadcaster = TransformBroadcaster(self)
                elif p.name == 'publish_curr_pose':
                    with self._lock:
                        self._publish_curr = bool(p.value)
                elif p.name == 'publish_delta_pose':
                    with self._lock:
                        self._publish_delta = bool(p.value)
            except (ValueError, TypeError) as exc:
                return SetParametersResult(successful=False, reason=str(exc))

        if new_freq is not None:
            with self._lock:
                self._freq = new_freq
            self._make_timer()
        return SetParametersResult(successful=True)

    # ── Integration tick ──
    def _on_tick(self):
        now = self.get_clock().now()
        with self._lock:
            freq = self._freq
            max_trans = self._max_trans
            max_rot = self._max_rot
            frame = self._frame
            frame_id = self._frame_id
            deadzone = self._deadzone
            timeout = self._input_timeout
            axes = list(self._axes)
            last_input = self._last_input
            p = list(self._p)
            q = self._q
            pub_curr = self._publish_curr
            pub_delta = self._publish_delta

        dt = 1.0 / max(freq, 1e-3)

        # Safety: drop stale input so a lost stream can't keep integrating.
        if timeout > 0.0 and \
                (now - last_input).nanoseconds * 1e-9 > timeout:
            axes = [0.0] * 6

        def dz(val):
            return 0.0 if abs(val) < deadzone else val

        v = [_clamp(dz(axes[i]), -1.0, 1.0) for i in range(3)]
        w = [_clamp(dz(axes[i]), -1.0, 1.0) for i in range(3, 6)]

        # Per-tick delta translation (meters) and rotation (quaternion).
        d_trans = [v[i] * max_trans * dt for i in range(3)]
        rotvec = [w[i] * max_rot * dt for i in range(3)]
        q_delta = rotvec_to_quat(rotvec[0], rotvec[1], rotvec[2])

        # Accumulate into curr_pose ONLY while publishing is enabled; when
        # curr_pose is disabled the underlying pose FREEZES (no drift) and only
        # resumes on re-enable or an explicit set_pose.
        if pub_curr:
            if frame == 'body':
                q_new = quat_normalize(quat_mul(q, q_delta))
                dtw = quat_rotate_vec(q, d_trans)
                p_new = [p[i] + dtw[i] for i in range(3)]
            else:  # 'world'
                q_new = quat_normalize(quat_mul(q_delta, q))
                p_new = [p[i] + d_trans[i] for i in range(3)]
            with self._lock:
                self._p = p_new
                self._q = q_new
        else:
            p_new, q_new = p, q

        stamp = now.to_msg()
        if pub_delta:
            self._pub_delta.publish(
                self._make_pose(stamp, frame_id, d_trans, q_delta))
        if pub_curr:
            self._pub_curr.publish(
                self._make_pose(stamp, frame_id, p_new, q_new))

        if self._publish_tf and self._tf_broadcaster is not None:
            self._broadcast_tf(stamp, frame_id, p_new, q_new)

    @staticmethod
    def _make_pose(stamp, frame_id, p, q):
        msg = PoseStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = frame_id
        msg.pose.position.x = p[0]
        msg.pose.position.y = p[1]
        msg.pose.position.z = p[2]
        msg.pose.orientation.w = q[0]
        msg.pose.orientation.x = q[1]
        msg.pose.orientation.y = q[2]
        msg.pose.orientation.z = q[3]
        return msg

    def _broadcast_tf(self, stamp, frame_id, p, q):
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = frame_id
        t.child_frame_id = 'spacenav_curr_pose'
        t.transform.translation.x = p[0]
        t.transform.translation.y = p[1]
        t.transform.translation.z = p[2]
        t.transform.rotation.w = q[0]
        t.transform.rotation.x = q[1]
        t.transform.rotation.y = q[2]
        t.transform.rotation.z = q[3]
        self._tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = PoseNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
