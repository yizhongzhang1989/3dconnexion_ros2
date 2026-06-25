import json
import os
import time
import threading
from http.server import ThreadingHTTPServer, BaseHTTPRequestHandler

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import Joy
from ament_index_python.packages import get_package_share_directory


class _HzTracker:
    """Tracks message frequency for a topic."""

    def __init__(self):
        self._count = 0
        self._hz = 0.0
        self._last_reset = time.monotonic()
        self._lock = threading.Lock()

    def tick(self):
        with self._lock:
            self._count += 1
            now = time.monotonic()
            elapsed = now - self._last_reset
            if elapsed >= 1.0:
                self._hz = self._count / elapsed
                self._count = 0
                self._last_reset = now

    @property
    def hz(self):
        with self._lock:
            return round(self._hz, 1)


class DashboardNode(Node):
    def __init__(self):
        super().__init__('dashboard_node')

        self.declare_parameter('http_port', 8080)
        self._http_port = int(self.get_parameter('http_port').value)

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
        }
        self.data_lock = threading.Lock()

        # Frequency trackers
        self._hz = {
            'spacenav/twist': _HzTracker(),
            'spacenav/offset': _HzTracker(),
            'spacenav/rot_offset': _HzTracker(),
            'spacenav/joy': _HzTracker(),
        }

        # Subscribers
        self.create_subscription(Twist, 'spacenav/twist', self._twist_cb, 10)
        self.create_subscription(Vector3, 'spacenav/offset', self._offset_cb, 10)
        self.create_subscription(Vector3, 'spacenav/rot_offset', self._rot_offset_cb, 10)
        self.create_subscription(Joy, 'spacenav/joy', self._joy_cb, 10)

        # Start HTTP server
        self._start_http_server()

        self.get_logger().info(
            f'Dashboard running at http://localhost:{self._http_port}'
        )

    def _twist_cb(self, msg: Twist):
        self._hz['spacenav/twist'].tick()
        with self.data_lock:
            self.data['twist'] = {
                'lx': msg.linear.x, 'ly': msg.linear.y, 'lz': msg.linear.z,
                'ax': msg.angular.x, 'ay': msg.angular.y, 'az': msg.angular.z,
            }

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
        with self.data_lock:
            self.data['joy'] = {
                'axes': list(msg.axes),
                'buttons': list(msg.buttons),
            }

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
                    resp['hz'] = {t: tr.hz for t, tr in node_ref._hz.items()}
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
