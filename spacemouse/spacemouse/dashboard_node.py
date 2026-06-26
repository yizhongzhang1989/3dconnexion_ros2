import json
import math
import os
import time
import threading
from http.server import ThreadingHTTPServer, BaseHTTPRequestHandler

import rclpy
from rclpy.node import Node
from rcl_interfaces.srv import SetParameters, GetParameters
from rcl_interfaces.msg import Parameter as ParameterMsg
from rcl_interfaces.msg import ParameterType
from geometry_msgs.msg import Twist, Vector3, PoseStamped
from sensor_msgs.msg import Joy
from ament_index_python.packages import get_package_share_directory

_IDENTITY_POSE = {'px': 0.0, 'py': 0.0, 'pz': 0.0,
                  'qx': 0.0, 'qy': 0.0, 'qz': 0.0, 'qw': 1.0}


class _HzTracker:
    """Tracks message frequency and freshness for a topic.

    ``hz`` reports 0 once messages stop (older trackers froze at the last
    computed value), and ``age`` exposes the seconds since the last message so
    callers can decide whether a stream is alive.
    """

    # A stream is considered stale (hz -> 0) if nothing arrives for this long.
    _STALE_AFTER = 1.5

    def __init__(self):
        self._count = 0
        self._hz = 0.0
        self._last_reset = time.monotonic()
        self._last_tick = None
        self._lock = threading.Lock()

    def tick(self):
        with self._lock:
            now = time.monotonic()
            self._count += 1
            self._last_tick = now
            elapsed = now - self._last_reset
            if elapsed >= 1.0:
                self._hz = self._count / elapsed
                self._count = 0
                self._last_reset = now

    @property
    def hz(self):
        with self._lock:
            if (self._last_tick is None or
                    time.monotonic() - self._last_tick > self._STALE_AFTER):
                self._hz = 0.0
            return round(self._hz, 1)

    @property
    def age(self):
        """Seconds since the last message, or ``None`` if none seen yet."""
        with self._lock:
            if self._last_tick is None:
                return None
            return round(time.monotonic() - self._last_tick, 2)


class DashboardNode(Node):
    # Topic -> (publishing node, bool parameter that gates it).
    _PUBLISH_TOPICS = {
        'spacenav/twist': ('spacenav', 'publish_twist'),
        'spacenav/offset': ('spacenav', 'publish_offset'),
        'spacenav/rot_offset': ('spacenav', 'publish_rot_offset'),
        'spacenav/joy': ('spacenav', 'publish_joy'),
        'spacenav/curr_pose': ('pose', 'publish_curr_pose'),
        'spacenav/delta_pose': ('pose', 'publish_delta_pose'),
    }

    def __init__(self):
        super().__init__('dashboard_node')

        self.declare_parameter('http_port', 8080)
        self._http_port = int(self.get_parameter('http_port').value)

        self.declare_parameter('pose_node_name', 'pose_node')
        self._pose_node_name = str(
            self.get_parameter('pose_node_name').value)

        self.declare_parameter('spacenav_node_name', 'spacenav_node')
        self._spacenav_node_name = str(
            self.get_parameter('spacenav_node_name').value)

        self._web_dir = os.path.join(
            get_package_share_directory('spacemouse'), 'web'
        )

        # Shared data (read by HTTP handler, written by ROS callbacks)
        self.data = {
            'twist': {'lx': 0.0, 'ly': 0.0, 'lz': 0.0,
                      'ax': 0.0, 'ay': 0.0, 'az': 0.0},
            'offset': {'x': 0.0, 'y': 0.0, 'z': 0.0},
            'rot_offset': {'x': 0.0, 'y': 0.0, 'z': 0.0},
            'joy': {'axes': [], 'buttons': []},
            'curr_pose': dict(_IDENTITY_POSE),
            'delta_pose': dict(_IDENTITY_POSE),
        }
        self.data_lock = threading.Lock()
        # Monotonic time of the last REAL input (non-zero motion or a button
        # press). Used to tell a live SpaceMouse apart from the all-zero stream
        # the driver emits even with no device attached.
        self._last_activity = None

        # Cached pose_node parameter values (used to seed the dashboard sliders)
        self._pose_params = {
            'max_trans_speed': None,
            'max_rot_speed': None,
            'publish_frequency': None,
        }
        # Per-topic publish on/off state (mirrors the nodes' publish_* params).
        self._publish_state = {t: True for t in self._PUBLISH_TOPICS}

        # Pending UI commands; flushed to ROS by a timer on the executor thread
        # so the HTTP handler threads never touch ROS directly.
        self._cmd_lock = threading.Lock()
        self._pending_set_pose = None
        self._pending_params = None
        self._pending_publish = {}

        # Frequency trackers
        self._hz = {
            'spacenav/twist': _HzTracker(),
            'spacenav/offset': _HzTracker(),
            'spacenav/rot_offset': _HzTracker(),
            'spacenav/joy': _HzTracker(),
            'spacenav/curr_pose': _HzTracker(),
            'spacenav/delta_pose': _HzTracker(),
        }

        # Subscribers
        self.create_subscription(Twist, 'spacenav/twist', self._twist_cb, 10)
        self.create_subscription(Vector3, 'spacenav/offset', self._offset_cb, 10)
        self.create_subscription(Vector3, 'spacenav/rot_offset', self._rot_offset_cb, 10)
        self.create_subscription(Joy, 'spacenav/joy', self._joy_cb, 10)
        self.create_subscription(
            PoseStamped, 'spacenav/curr_pose', self._curr_pose_cb, 10)
        self.create_subscription(
            PoseStamped, 'spacenav/delta_pose', self._delta_pose_cb, 10)

        # Publisher + parameter service clients to drive pose_node (dashboard -> ROS)
        self._set_pose_pub = self.create_publisher(
            PoseStamped, 'spacenav/set_pose', 10)
        self._set_param_cli = self.create_client(
            SetParameters, f'{self._pose_node_name}/set_parameters')
        self._get_param_cli = self.create_client(
            GetParameters, f'{self._pose_node_name}/get_parameters')
        self._set_param_cli_sn = self.create_client(
            SetParameters, f'{self._spacenav_node_name}/set_parameters')
        self._get_param_cli_sn = self.create_client(
            GetParameters, f'{self._spacenav_node_name}/get_parameters')

        # Command bridge + periodic parameter refresh.
        self.create_timer(0.05, self._flush_commands)
        self.create_timer(1.0, self._refresh_pose_params)
        self.create_timer(1.0, self._refresh_publish_state)

        # Start HTTP server
        self._start_http_server()

        self.get_logger().info(
            f'Dashboard running at http://localhost:{self._http_port}'
        )

    def _twist_cb(self, msg: Twist):
        self._hz['spacenav/twist'].tick()
        moving = max(abs(msg.linear.x), abs(msg.linear.y), abs(msg.linear.z),
                     abs(msg.angular.x), abs(msg.angular.y),
                     abs(msg.angular.z)) > self._ACTIVITY_EPS
        with self.data_lock:
            self.data['twist'] = {
                'lx': msg.linear.x, 'ly': msg.linear.y, 'lz': msg.linear.z,
                'ax': msg.angular.x, 'ay': msg.angular.y, 'az': msg.angular.z,
            }
            if moving:
                self._last_activity = time.monotonic()

    def _offset_cb(self, msg: Vector3):
        self._hz['spacenav/offset'].tick()
        with self.data_lock:
            self.data['offset'] = {'x': msg.x, 'y': msg.y, 'z': msg.z}

    def _rot_offset_cb(self, msg: Vector3):
        self._hz['spacenav/rot_offset'].tick()
        with self.data_lock:
            self.data['rot_offset'] = {'x': msg.x, 'y': msg.y, 'z': msg.z}

    def _joy_cb(self, msg: Joy):
        self._hz['spacenav/joy'].tick()
        active = (any(msg.buttons) or
                  any(abs(a) > self._ACTIVITY_EPS for a in msg.axes))
        with self.data_lock:
            self.data['joy'] = {
                'axes': list(msg.axes),
                'buttons': list(msg.buttons),
            }
            if active:
                self._last_activity = time.monotonic()

    def _curr_pose_cb(self, msg: PoseStamped):
        self._hz['spacenav/curr_pose'].tick()
        with self.data_lock:
            self.data['curr_pose'] = self._pose_to_dict(msg.pose)

    def _delta_pose_cb(self, msg: PoseStamped):
        self._hz['spacenav/delta_pose'].tick()
        with self.data_lock:
            self.data['delta_pose'] = self._pose_to_dict(msg.pose)

    @staticmethod
    def _pose_to_dict(pose):
        return {
            'px': pose.position.x, 'py': pose.position.y, 'pz': pose.position.z,
            'qx': pose.orientation.x, 'qy': pose.orientation.y,
            'qz': pose.orientation.z, 'qw': pose.orientation.w,
        }

    # ── Device connectivity ──────────────────────────────────────────────
    # IMPORTANT: the spacenav driver bursts ZERO-valued twist/joy at ~33 Hz
    # whenever it can reach the spacenavd daemon -- even with NO device plugged
    # in (verified: a device absent from lsusb still yields zero twists). So a
    # fresh stream only proves the driver<->spacenavd link, not a live
    # SpaceMouse. We report three states instead:
    #   offline : no twist/joy at all (driver down or spacenavd unreachable).
    #   idle    : frames flowing but all zero -- a STILL or an UNPLUGGED device
    #             look identical (no movement is being delivered either way).
    #   active  : real non-zero motion or a button seen recently -> the
    #             SpaceMouse is definitely present and delivering input.
    _ACTIVITY_EPS = 1e-4
    _DEVICE_FRESH_AFTER = 1.5
    _DEVICE_ACTIVE_AFTER = 2.5

    def device_status(self):
        twist = self._hz['spacenav/twist']
        joy = self._hz['spacenav/joy']

        def _fresh(age):
            return age is not None and age < self._DEVICE_FRESH_AFTER

        streaming = _fresh(twist.age) or _fresh(joy.age)
        with self.data_lock:
            last_act = self._last_activity
        active = (last_act is not None and
                  time.monotonic() - last_act < self._DEVICE_ACTIVE_AFTER)
        if not streaming:
            state = 'offline'
        elif active:
            state = 'active'
        else:
            state = 'idle'
        return {
            'state': state,
            'connected': active,    # honest: only True while delivering input
            'streaming': streaming,
            'twist_hz': twist.hz,
            'joy_hz': joy.hz,
            'twist_age': twist.age,
            'joy_age': joy.age,
        }

    # ── UI command bridge: HTTP threads store intents, a ROS timer applies them
    def request_set_pose(self, payload):
        kind = payload.get('type', 'pose')
        if kind in ('identity', 'offset'):
            req = (kind, None)
        elif kind == 'pose':
            pos = payload.get('position', [0.0, 0.0, 0.0])
            ori = payload.get('orientation', [0.0, 0.0, 0.0, 1.0])
            if len(pos) != 3 or len(ori) != 4:
                return False, 'position must be [x,y,z], orientation [x,y,z,w]'
            req = ('pose', (pos, ori))
        else:
            return False, f'unknown set_pose type: {kind}'
        with self._cmd_lock:
            self._pending_set_pose = req
        return True, None

    def request_params(self, payload):
        updates = {}
        for key in ('max_trans_speed', 'max_rot_speed', 'publish_frequency'):
            if key in payload:
                try:
                    val = float(payload[key])
                except (ValueError, TypeError):
                    return False, f'{key} must be a number'
                if val < 0.0:
                    return False, f'{key} must be >= 0'
                updates[key] = val
        if not updates:
            return False, 'no recognized parameters'
        with self._cmd_lock:
            self._pending_params = updates
        return True, None

    def request_publish(self, payload):
        topic = payload.get('topic')
        if topic not in self._PUBLISH_TOPICS:
            return False, f'unknown topic: {topic}'
        if 'enabled' not in payload:
            return False, 'missing "enabled"'
        enabled = bool(payload['enabled'])
        with self._cmd_lock:
            self._pending_publish[topic] = enabled
        with self.data_lock:
            self._publish_state[topic] = enabled
        return True, None

    def _flush_commands(self):
        with self._cmd_lock:
            set_pose = self._pending_set_pose
            self._pending_set_pose = None
            params = self._pending_params
            self._pending_params = None
            publish = self._pending_publish
            self._pending_publish = {}
        if set_pose is not None:
            self._publish_set_pose(set_pose)
        if params is not None:
            self._apply_params(params)
        if publish:
            self._apply_publish(publish)

    def _publish_set_pose(self, req):
        kind, data = req
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'spacenav_origin'
        if kind == 'identity':
            msg.pose.orientation.w = 1.0
        elif kind == 'offset':
            msg.pose.position.x = 0.5
            msg.pose.position.y = 0.0
            msg.pose.position.z = 0.2
            half = math.radians(45.0) / 2.0
            msg.pose.orientation.z = math.sin(half)
            msg.pose.orientation.w = math.cos(half)
        else:  # explicit pose
            pos, ori = data
            msg.pose.position.x = float(pos[0])
            msg.pose.position.y = float(pos[1])
            msg.pose.position.z = float(pos[2])
            msg.pose.orientation.x = float(ori[0])
            msg.pose.orientation.y = float(ori[1])
            msg.pose.orientation.z = float(ori[2])
            msg.pose.orientation.w = float(ori[3])
        self._set_pose_pub.publish(msg)

    def _apply_params(self, updates):
        if not self._set_param_cli.service_is_ready():
            self.get_logger().warn(
                'pose_node set_parameters service unavailable; cannot set params')
            return
        req = SetParameters.Request()
        for k, v in updates.items():
            pmsg = ParameterMsg()
            pmsg.name = k
            pmsg.value.type = ParameterType.PARAMETER_DOUBLE
            pmsg.value.double_value = float(v)
            req.parameters.append(pmsg)
        future = self._set_param_cli.call_async(req)
        with self.data_lock:
            for k, v in updates.items():
                self._pose_params[k] = v

        def _done(fut):
            try:
                fut.result()
            except Exception as exc:  # noqa: BLE001
                self.get_logger().warn(f'set_parameters failed: {exc}')
        future.add_done_callback(_done)

    def _refresh_pose_params(self):
        if not self._get_param_cli.service_is_ready():
            return
        names = ['max_trans_speed', 'max_rot_speed', 'publish_frequency']
        req = GetParameters.Request()
        req.names = names
        future = self._get_param_cli.call_async(req)

        def _done(fut):
            try:
                resp = fut.result()
            except Exception:  # noqa: BLE001
                return
            values = getattr(resp, 'values', None)
            if not values:
                return
            with self.data_lock:
                for name, pv in zip(names, values):
                    if pv.type == ParameterType.PARAMETER_DOUBLE:
                        self._pose_params[name] = pv.double_value
                    elif pv.type == ParameterType.PARAMETER_INTEGER:
                        self._pose_params[name] = float(pv.integer_value)
        future.add_done_callback(_done)

    def _apply_publish(self, updates):
        clients = {'pose': self._set_param_cli,
                   'spacenav': self._set_param_cli_sn}
        by_node = {'pose': [], 'spacenav': []}
        for topic, enabled in updates.items():
            node_kind, param = self._PUBLISH_TOPICS[topic]
            by_node[node_kind].append((param, bool(enabled)))
        for node_kind, items in by_node.items():
            if not items:
                continue
            cli = clients[node_kind]
            if not cli.service_is_ready():
                self.get_logger().warn(
                    f'{node_kind} set_parameters unavailable; '
                    'cannot toggle publishing')
                continue
            req = SetParameters.Request()
            for param, enabled in items:
                pmsg = ParameterMsg()
                pmsg.name = param
                pmsg.value.type = ParameterType.PARAMETER_BOOL
                pmsg.value.bool_value = enabled
                req.parameters.append(pmsg)

            def _done(fut):
                try:
                    fut.result()
                except Exception as exc:  # noqa: BLE001
                    self.get_logger().warn(f'publish toggle failed: {exc}')
            cli.call_async(req).add_done_callback(_done)

    def _refresh_publish_state(self):
        groups = {
            'pose': (self._get_param_cli,
                     ['publish_curr_pose', 'publish_delta_pose']),
            'spacenav': (self._get_param_cli_sn,
                         ['publish_twist', 'publish_offset',
                          'publish_rot_offset', 'publish_joy']),
        }
        for node_kind, (cli, names) in groups.items():
            if not cli.service_is_ready():
                continue
            rev = {p: t for t, (nk, p) in self._PUBLISH_TOPICS.items()
                   if nk == node_kind}
            req = GetParameters.Request()
            req.names = names

            def _done(fut, names=names, rev=rev):
                try:
                    resp = fut.result()
                except Exception:  # noqa: BLE001
                    return
                values = getattr(resp, 'values', None)
                if not values:
                    return
                with self.data_lock:
                    for name, pv in zip(names, values):
                        if (pv.type == ParameterType.PARAMETER_BOOL
                                and name in rev):
                            self._publish_state[rev[name]] = pv.bool_value
            cli.call_async(req).add_done_callback(_done)

    def _start_http_server(self):
        node_ref = self
        web_dir = self._web_dir

        class Handler(BaseHTTPRequestHandler):
            # Drop idle/half-open connections (e.g. browser pre-connect sockets
            # that never send a request) so they can't pin a worker thread.
            timeout = 10

            def do_GET(self):
                # JSON data endpoint
                if self.path == '/data':
                    with node_ref.data_lock:
                        resp = dict(node_ref.data)
                        resp['pose_params'] = dict(node_ref._pose_params)
                        resp['publish_state'] = dict(node_ref._publish_state)
                    resp['hz'] = {t: tr.hz for t, tr in node_ref._hz.items()}
                    resp['age'] = {t: tr.age for t, tr in node_ref._hz.items()}
                    resp['device'] = node_ref.device_status()
                    payload = json.dumps(resp).encode()
                    self.send_response(200)
                    self.send_header('Content-Type', 'application/json')
                    self.send_header('Content-Length', str(len(payload)))
                    self.send_header('Access-Control-Allow-Origin', '*')
                    self.send_header('Cache-Control', 'no-cache')
                    self.end_headers()
                    self.wfile.write(payload)
                    return

                # Static file serving
                path = self.path.split('?')[0].split('#')[0]
                if path == '/':
                    path = '/index.html'
                path = os.path.normpath(path).lstrip('/')

                # Block directory traversal
                if '..' in path.split(os.sep):
                    self.send_error(403)
                    return

                file_path = os.path.join(web_dir, path)
                if not os.path.isfile(file_path):
                    # Resolve symlink and retry (for symlink-install)
                    resolved = os.path.realpath(file_path)
                    if os.path.isfile(resolved):
                        file_path = resolved
                    else:
                        self.send_error(404)
                        return

                with open(file_path, 'rb') as f:
                    content = f.read()

                ext = os.path.splitext(file_path)[1]
                content_types = {
                    '.html': 'text/html; charset=utf-8',
                    '.js': 'application/javascript',
                    '.css': 'text/css',
                    '.png': 'image/png',
                    '.ico': 'image/x-icon',
                    '.svg': 'image/svg+xml',
                    '.json': 'application/json',
                }
                ct = content_types.get(ext, 'application/octet-stream')

                self.send_response(200)
                self.send_header('Content-Type', ct)
                self.send_header('Content-Length', str(len(content)))
                self.send_header('Cache-Control', 'no-cache, no-store, must-revalidate')
                self.end_headers()
                self.wfile.write(content)

            def do_POST(self):
                length = int(self.headers.get('Content-Length', 0) or 0)
                body = self.rfile.read(length) if length > 0 else b''
                try:
                    payload = json.loads(body or b'{}')
                except (json.JSONDecodeError, ValueError):
                    self.send_error(400, 'invalid JSON')
                    return

                if self.path == '/set_pose':
                    ok, err = node_ref.request_set_pose(payload)
                elif self.path == '/params':
                    ok, err = node_ref.request_params(payload)
                elif self.path == '/publish':
                    ok, err = node_ref.request_publish(payload)
                else:
                    self.send_error(404)
                    return

                if not ok:
                    self.send_error(400, err or 'bad request')
                    return
                out = json.dumps({'ok': True}).encode()
                self.send_response(200)
                self.send_header('Content-Type', 'application/json')
                self.send_header('Content-Length', str(len(out)))
                self.send_header('Access-Control-Allow-Origin', '*')
                self.end_headers()
                self.wfile.write(out)

            def handle_one_request(self):
                # A browser that disconnects mid-response (e.g. refresh while a
                # /data poll is in flight) raises BrokenPipeError/ConnectionReset;
                # swallow it quietly instead of dumping a traceback.
                try:
                    super().handle_one_request()
                except (BrokenPipeError, ConnectionResetError):
                    self.close_connection = True

            def log_message(self, format, *args):
                # Suppress per-request logs to keep terminal clean
                pass

        # Threading server: each connection is handled in its own thread, so a
        # single slow/idle client cannot block the accept loop (which previously
        # froze the whole dashboard for both local and remote browsers).
        server = ThreadingHTTPServer(('0.0.0.0', self._http_port), Handler)
        server.daemon_threads = True
        t = threading.Thread(target=server.serve_forever, daemon=True)
        t.start()


def main(args=None):
    rclpy.init(args=args)
    node = DashboardNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
