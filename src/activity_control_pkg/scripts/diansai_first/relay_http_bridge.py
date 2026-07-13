#!/usr/bin/env python3
"""HTTP-to-ROS bridge for the diansai_first relay delivery task."""

import json
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Any, Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty
from std_msgs.msg import String


# =============================================================================
# 无人机2接力地点映射
# 网页端只发送语义地点；香橙派端在这里映射为 ROS 飞行坐标。
# pose = (x_cm, y_cm, z_cm, yaw_deg)，单位分别为厘米和角度。
# =============================================================================

DESTINATION_POINTS = {
    'A楼': (145.0, 10.0, 80.0, 0.0),
    'B楼': (145.0, 65.0, 80.0, 0.0),
    'C楼': (145.0, 120.0, 80.0, 0.0),
}

DESTINATION_ALIASES = {
    '01': 'A楼',
    '1': 'A楼',
    'a': 'A楼',
    'A': 'A楼',
    'A楼': 'A楼',
    '02': 'B楼',
    '2': 'B楼',
    'b': 'B楼',
    'B': 'B楼',
    'B楼': 'B楼',
    '03': 'C楼',
    '3': 'C楼',
    'c': 'C楼',
    'C': 'C楼',
    'C楼': 'C楼',
}

BUSY_STATES = {
    'command_published',
    'accepted',
    'running',
}

ERROR_STATES = {
    'rejected',
    'error',
    'failed',
    'stopped',
}


def state_ok(state: str) -> bool:
    return state not in ERROR_STATES


class RelayHttpBridge(Node):
    def __init__(self) -> None:
        super().__init__('diansai_first_relay_http_bridge')
        self.declare_parameter('host', '0.0.0.0')
        self.declare_parameter('port', 8080)
        self.declare_parameter('relay_delivery_topic', '/web/relay_delivery')
        self.declare_parameter('relay_status_topic', '/web/relay_status')
        self.declare_parameter('mission_complete_topic', '/mission_complete')

        self.host = str(self.get_parameter('host').value)
        self.port = int(self.get_parameter('port').value)
        self.relay_delivery_topic = str(self.get_parameter('relay_delivery_topic').value)
        self.relay_status_topic = str(self.get_parameter('relay_status_topic').value)
        self.mission_complete_topic = str(self.get_parameter('mission_complete_topic').value)

        self.delivery_pub = self.create_publisher(String, self.relay_delivery_topic, 10)
        self.create_subscription(String, self.relay_status_topic, self.status_callback, 10)
        self.create_subscription(Empty, self.mission_complete_topic, self.mission_complete_callback, 10)

        self._lock = threading.Lock()
        self._latest_command: Optional[dict[str, Any]] = None
        self._latest_status: dict[str, Any] = self._make_status(
            'bridge_started',
            'HTTP bridge is running, waiting for ROS task status.',
        )
        self._server: Optional[ThreadingHTTPServer] = None
        self._server_thread: Optional[threading.Thread] = None

    def start_server(self) -> None:
        handler = self._make_handler()
        self._server = ThreadingHTTPServer((self.host, self.port), handler)
        self._server_thread = threading.Thread(target=self._server.serve_forever, daemon=True)
        self._server_thread.start()
        self.get_logger().info(
            '无人机2 HTTP Bridge 已启动：http://%s:%d  POST /api/relay_delivery  GET /api/status'
            % (self.host, self.port)
        )

    def stop_server(self) -> None:
        if self._server is not None:
            self._server.shutdown()
            self._server.server_close()
            self._server = None
        if self._server_thread is not None:
            self._server_thread.join(timeout=1.0)
            self._server_thread = None

    def status_callback(self, msg: String) -> None:
        try:
            status = json.loads(msg.data)
        except json.JSONDecodeError:
            status = self._make_status('raw_status', msg.data)
        if not isinstance(status, dict):
            status = self._make_status('raw_status', str(status))
        state = str(status.get('state', 'raw_status'))
        message = str(status.get('message', ''))
        with self._lock:
            merged = self._make_status(state, message)
            merged.update(status)
            merged['ok'] = state_ok(state)
            if self._latest_command is not None:
                merged.setdefault('mission_id', str(self._latest_command.get('mission_id', '')))
                merged.setdefault('destination', str(self._latest_command.get('destination', '')))
            self._latest_status = merged

    def mission_complete_callback(self, _msg: Empty) -> None:
        with self._lock:
            self._latest_status = self._make_status('done', 'Relay delivery mission completed.')

    def latest_status(self) -> dict[str, Any]:
        with self._lock:
            response = dict(self._latest_status)
            if self._latest_command is not None:
                response['latest_command'] = dict(self._latest_command)
            return response

    def set_manual_status(self, state: str, message: str) -> None:
        with self._lock:
            if state == 'bridge_started':
                self._latest_command = None
            self._latest_status = self._make_status(state, message)

    def _make_status(self, state: str, message: str) -> dict[str, Any]:
        mission_id = ''
        destination = ''
        if self._latest_command is not None:
            mission_id = str(self._latest_command.get('mission_id', ''))
            destination = str(self._latest_command.get('destination', ''))
        return {
            'ok': state_ok(state),
            'state': state,
            'message': message,
            'mission_id': mission_id,
            'destination': destination,
            'updated_unix_ms': int(time.time() * 1000),
        }

    def publish_delivery(self, payload: dict[str, Any]) -> dict[str, Any]:
        command = self._normalize_payload(payload)
        with self._lock:
            state = str(self._latest_status.get('state', ''))
            if state in BUSY_STATES:
                raise RuntimeError('Mission is already running.')
        message = String()
        message.data = json.dumps(command, ensure_ascii=False, separators=(',', ':'))
        self.delivery_pub.publish(message)
        with self._lock:
            self._latest_command = command
            self._latest_status = self._make_status(
                'command_published',
                'Relay delivery command was published to ROS.',
            )
        self.get_logger().info('已发布接力任务命令：%s' % message.data)
        return command

    def _normalize_payload(self, payload: dict[str, Any]) -> dict[str, Any]:
        if not isinstance(payload, dict):
            raise ValueError('JSON body must be an object.')

        raw_destination = (
            payload.get('destination')
            or payload.get('target_location')
            or payload.get('location')
            or payload.get('target')
            or payload.get('destination_id')
            or payload.get('location_id')
        )
        if raw_destination is None:
            raise ValueError('Missing destination/location.')

        destination_key = DESTINATION_ALIASES.get(str(raw_destination).strip())
        if destination_key is None:
            allowed = ', '.join(DESTINATION_POINTS.keys())
            raise ValueError('Unknown destination %r. Allowed: %s.' % (raw_destination, allowed))

        x_cm, y_cm, z_cm, yaw_deg = DESTINATION_POINTS[destination_key]
        command = {
            'schema': 'relay_delivery.v1',
            'mission_type': str(payload.get('mission_type') or 'relay_delivery'),
            'item_id': payload.get('item_id', ''),
            'item_name': str(payload.get('item_name') or payload.get('material') or payload.get('supply') or ''),
            'mission_id': str(payload.get('mission_id') or payload.get('task_id') or int(time.time() * 1000)),
            'material': str(payload.get('material') or payload.get('supply') or ''),
            'destination': destination_key,
            'endpoint': {
                'x_cm': x_cm,
                'y_cm': y_cm,
                'z_cm': z_cm,
                'yaw_deg': yaw_deg,
            },
            'source': 'relay_http_bridge',
            'received_unix_ms': int(time.time() * 1000),
        }
        return command

    def _make_handler(self):
        bridge = self

        class Handler(BaseHTTPRequestHandler):
            server_version = 'RelayHttpBridge/1.0'

            def do_OPTIONS(self) -> None:
                self._send_json(200, {'ok': True})

            def do_GET(self) -> None:
                path = self.path.split('?', 1)[0]
                if path not in ('/api/status', '/health'):
                    self._send_json(404, {'ok': False, 'error': 'not found'})
                    return
                self._send_json(200, bridge.latest_status())

            def do_POST(self) -> None:
                path = self.path.split('?', 1)[0]
                if path in ('/api/stop', '/stop'):
                    bridge.set_manual_status('stopped', 'Relay bridge was stopped manually. ROS flight must be handled by operator if already airborne.')
                    self._send_json(200, {'ok': True, 'status': bridge.latest_status()})
                    return
                if path in ('/api/reset', '/reset'):
                    bridge.set_manual_status('bridge_started', 'HTTP bridge was reset manually, waiting for ROS task status.')
                    self._send_json(200, {'ok': True, 'status': bridge.latest_status()})
                    return
                if path not in ('/api/relay_delivery', '/relay_delivery'):
                    self._send_json(404, {'ok': False, 'error': 'not found'})
                    return
                try:
                    length = int(self.headers.get('Content-Length', '0'))
                    if length <= 0:
                        raise ValueError('Empty request body.')
                    if length > 4096:
                        raise ValueError('Request body is too large.')
                    raw = self.rfile.read(length).decode('utf-8')
                    payload = json.loads(raw)
                    command = bridge.publish_delivery(payload)
                    self._send_json(200, {'ok': True, 'command': command, 'status': bridge.latest_status()})
                except RuntimeError as exc:
                    self._send_json(409, {'ok': False, 'error': str(exc), 'status': bridge.latest_status()})
                except (UnicodeDecodeError, json.JSONDecodeError, ValueError) as exc:
                    self._send_json(400, {'ok': False, 'error': str(exc)})
                except Exception as exc:  # noqa: BLE001 - keep HTTP server alive on unexpected errors.
                    bridge.get_logger().error('处理 HTTP 接力任务失败：%s' % exc)
                    self._send_json(500, {'ok': False, 'error': str(exc)})

            def log_message(self, fmt: str, *args) -> None:
                bridge.get_logger().info('HTTP %s - %s' % (self.address_string(), fmt % args))

            def _send_json(self, code: int, payload: dict[str, Any]) -> None:
                body = json.dumps(payload, ensure_ascii=False, separators=(',', ':')).encode('utf-8')
                self.send_response(code)
                self.send_header('Content-Type', 'application/json; charset=utf-8')
                self.send_header('Content-Length', str(len(body)))
                self.send_header('Access-Control-Allow-Origin', '*')
                self.send_header('Access-Control-Allow-Methods', 'GET, POST, OPTIONS')
                self.send_header('Access-Control-Allow-Headers', 'Content-Type')
                self.end_headers()
                self.wfile.write(body)

        return Handler


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RelayHttpBridge()
    node.start_server()
    try:
        rclpy.spin(node)
    finally:
        node.stop_server()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
