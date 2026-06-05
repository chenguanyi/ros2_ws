#!/usr/bin/env python3
import argparse
import json
import socket
import sys
import time

from PyQt5.QtCore import QObject, QPointF, QThread, QTimer, Qt, pyqtSignal
from PyQt5.QtGui import QColor, QPainter, QPen
from PyQt5.QtWidgets import (
    QApplication,
    QFormLayout,
    QHBoxLayout,
    QHeaderView,
    QLabel,
    QLineEdit,
    QMainWindow,
    QPushButton,
    QTableWidget,
    QTableWidgetItem,
    QTabWidget,
    QVBoxLayout,
    QWidget,
)


DEFAULT_HOST = "192.168.4.1"
DEFAULT_PORT = 8888
DEFAULT_LED_PIN = 11
DEFAULT_SOURCE = "ros2"
LED_FLASH_MS = 1000

ROS2_ROUTE_TOPIC = "/warehouse_inventory/route"
ROS2_SCAN_RESULT_TOPIC = "/warehouse_inventory/scan_result"
ROS2_ALL_RESULTS_TOPIC = "/warehouse_inventory/all_results"
ROS2_TARGET_ID_TOPIC = "/warehouse_inventory/target_id"
ROS2_QUERY_RESULT_TOPIC = "/warehouse_inventory/query_result"
ROS2_MISSION_COMPLETE_TOPIC = "/mission_complete"

WAREHOUSE_WIDTH_CM = 400
WAREHOUSE_HEIGHT_CM = 500
INVENTORY_TIME_LIMIT_S = 270
TARGET_TIME_LIMIT_S = 180

ITEM_COUNT = 24
VALID_ITEM_IDS = {str(index) for index in range(1, ITEM_COUNT + 1)}
VALID_POSITIONS = {f"{face}{slot}" for face in "ABCD" for slot in range(1, 7)}

PHASE_NAMES = {
    "takeoff": "起飞",
    "inventory": "遍历盘点",
    "target": "定点盘点",
    "landing": "降落",
    "done": "完成",
}


class Led:
    def __init__(self, pin=11):
        self.pin = pin
        self.gpio = None
        self.backend = "software"
        for module_name in ("Hobot.GPIO", "RPi.GPIO", "OPi.GPIO"):
            try:
                module = __import__(module_name, fromlist=["GPIO"])
                self.gpio = module
                self.backend = module_name
                break
            except Exception:
                continue
        if self.gpio:
            try:
                self.gpio.setmode(self.gpio.BOARD)
                self.gpio.setup(self.pin, self.gpio.OUT)
                self.off()
            except Exception:
                self.gpio = None
                self.backend = "software"

    def on(self):
        if self.gpio:
            self.gpio.output(self.pin, self.gpio.HIGH)

    def off(self):
        if self.gpio:
            self.gpio.output(self.pin, self.gpio.LOW)

    def cleanup(self):
        if self.gpio:
            self.off()
            self.gpio.cleanup()


class Receiver(QObject):
    message = pyqtSignal(dict)
    state = pyqtSignal(str)

    def __init__(self, host, port):
        super().__init__()
        self.host = host
        self.port = port
        self.running = True

    def stop(self):
        self.running = False

    def run(self):
        while self.running:
            try:
                self.state.emit(f"连接 {self.host}:{self.port} ...")
                with socket.create_connection((self.host, self.port), timeout=5) as sock:
                    self.state.emit("已连接")
                    file_obj = sock.makefile("r", encoding="utf-8")
                    while self.running:
                        line = file_obj.readline()
                        if not line:
                            break
                        try:
                            self.message.emit(json.loads(line))
                        except json.JSONDecodeError:
                            self.state.emit("收到无法解析的 JSON")
            except OSError as exc:
                self.state.emit(f"连接失败: {exc}")
                time.sleep(1)


class Ros2Receiver(QObject):
    message = pyqtSignal(dict)
    state = pyqtSignal(str)

    def __init__(self):
        super().__init__()
        self.running = True
        self.node = None

    def stop(self):
        self.running = False

    def run(self):
        try:
            import rclpy
            from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
            from std_msgs.msg import Empty, Int32, String
        except Exception as exc:
            self.state.emit(f"ROS2 导入失败: {exc}")
            return

        try:
            if not rclpy.ok():
                rclpy.init(args=None)

            durable_qos = QoSProfile(depth=10)
            durable_qos.reliability = ReliabilityPolicy.RELIABLE
            durable_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

            normal_qos = QoSProfile(depth=10)
            self.node = rclpy.create_node("ground_station_ros2_receiver")
            self.node.create_subscription(
                String, ROS2_SCAN_RESULT_TOPIC, self.on_scan_result, durable_qos
            )
            self.node.create_subscription(
                String, ROS2_ALL_RESULTS_TOPIC, self.on_all_results, durable_qos
            )
            self.node.create_subscription(String, ROS2_ROUTE_TOPIC, self.on_route, durable_qos)
            self.node.create_subscription(Int32, ROS2_TARGET_ID_TOPIC, self.on_target_id, durable_qos)
            self.node.create_subscription(
                String, ROS2_QUERY_RESULT_TOPIC, self.on_query_result, durable_qos
            )
            self.node.create_subscription(
                Empty, ROS2_MISSION_COMPLETE_TOPIC, self.on_mission_complete, normal_qos
            )

            self.state.emit("ROS2 话题接收已启动")
            while self.running and rclpy.ok():
                rclpy.spin_once(self.node, timeout_sec=0.1)
        except Exception as exc:
            self.state.emit(f"ROS2 接收失败: {exc}")
        finally:
            if self.node is not None:
                self.node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()

    def parse_json(self, text, source):
        try:
            return json.loads(text)
        except json.JSONDecodeError as exc:
            self.state.emit(f"{source} JSON 解析失败: {exc}")
            return None

    def on_scan_result(self, msg):
        data = self.parse_json(msg.data, ROS2_SCAN_RESULT_TOPIC)
        if not data:
            return
        item_id = str(data.get("id", "")).strip()
        coord = data.get("coord", "")
        self.message.emit({"type": "scan", "id": item_id, "pos": coord})

    def on_all_results(self, msg):
        data = self.parse_json(msg.data, ROS2_ALL_RESULTS_TOPIC)
        if not data:
            return
        inventory = data.get("inventory", {})
        all_items = []
        for item_id, record in inventory.items():
            if isinstance(record, dict):
                all_items.append({"id": str(item_id), "pos": record.get("coord", "")})
        self.message.emit({"type": "scan", "id": "", "all": all_items})

    def on_route(self, msg):
        data = self.parse_json(msg.data, ROS2_ROUTE_TOPIC)
        if not data:
            return
        steps = data.get("steps", [])
        waypoints = []
        target_coord = ""
        for step in steps:
            if not isinstance(step, dict):
                continue
            if "x_cm" in step and "y_cm" in step:
                waypoints.append([step["x_cm"], step["y_cm"]])
            if step.get("scan") and step.get("coord"):
                target_coord = step["coord"]
        self.message.emit({"type": "route", "target": target_coord, "waypoints": waypoints})

    def on_target_id(self, msg):
        self.message.emit({"type": "recognized", "id": str(msg.data), "pos": ""})

    def on_query_result(self, msg):
        data = self.parse_json(msg.data, ROS2_QUERY_RESULT_TOPIC)
        if not data:
            return
        self.message.emit(
            {
                "type": "query_result",
                "id": str(data.get("id", "")),
                "found": bool(data.get("found", False)),
                "pos": data.get("coord", ""),
            }
        )

    def on_mission_complete(self, msg):
        self.message.emit({"type": "status", "mode": 1, "phase": "done"})


class LedDot(QLabel):
    def __init__(self):
        super().__init__()
        self.setFixedSize(22, 22)
        self.set_on(False)

    def set_on(self, value):
        color = "#22c55e" if value else "#334155"
        self.setStyleSheet(
            f"border-radius: 11px; background: {color}; border: 2px solid #e2e8f0;"
        )


class RouteCanvas(QWidget):
    def __init__(self):
        super().__init__()
        self.target = ""
        self.waypoints = []
        self.setMinimumHeight(380)

    def set_route(self, target, waypoints):
        self.target = str(target or "")
        self.waypoints = waypoints or []
        self.update()

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        painter.fillRect(self.rect(), QColor("#f8fafc"))

        margin = 36
        area = self.rect().adjusted(margin, margin, -margin, -margin)
        painter.setPen(QPen(QColor("#cbd5e1"), 2))
        painter.drawRect(area)
        painter.setPen(QColor("#475569"))
        painter.drawText(area.adjusted(8, 8, -8, -8), Qt.AlignTop | Qt.AlignLeft, "仓库 400cm x 500cm")

        painter.setPen(Qt.NoPen)
        painter.setBrush(QColor("#111827"))
        painter.drawRect(area.left() + 8, area.top() + 8, 22, 22)
        painter.drawEllipse(area.right() - 32, area.bottom() - 32, 24, 24)
        painter.setPen(QColor("#111827"))
        painter.drawText(area.left() + 36, area.top() + 26, "起飞点")
        painter.drawText(area.right() - 86, area.bottom() - 18, "降落点")

        if not self.waypoints:
            painter.setPen(QColor("#64748b"))
            painter.drawText(self.rect(), Qt.AlignCenter, "等待航线数据")
            return

        def map_point(point):
            x, y = float(point[0]), float(point[1])
            px = area.left() + x / WAREHOUSE_WIDTH_CM * area.width()
            py = area.top() + y / WAREHOUSE_HEIGHT_CM * area.height()
            return QPointF(px, py)

        points = [map_point(p) for p in self.waypoints]
        painter.setPen(QPen(QColor("#2563eb"), 4))
        for left, right in zip(points, points[1:]):
            painter.drawLine(left, right)

        for index, point in enumerate(points):
            painter.setBrush(QColor("#ef4444" if index == len(points) - 1 else "#0f172a"))
            painter.setPen(Qt.NoPen)
            painter.drawEllipse(point, 7, 7)
            painter.setPen(QColor("#0f172a"))
            painter.drawText(point + QPointF(10, -8), str(index + 1))


class GroundStation(QMainWindow):
    def __init__(self, host, port, led_pin, source):
        super().__init__()
        self.setWindowTitle("D题立体货架盘点无人机系统地面站")
        self.resize(1060, 720)

        self.items = {}
        self.start_time = time.monotonic()
        self.current_mode = 1
        self.time_limit_s = INVENTORY_TIME_LIMIT_S
        self.led = Led(pin=led_pin)
        self.source = source

        self.led_dot = LedDot()
        self.status_label = QLabel("未连接" if source == "tcp" else "ROS2 未启动")
        self.phase_label = QLabel("阶段: -")
        self.timer_label = QLabel("00:00")
        self.limit_label = QLabel(f"限时: {INVENTORY_TIME_LIMIT_S}s")
        self.count_label = QLabel(f"盘点: 0/{ITEM_COUNT}")
        self.recognized_label = QLabel("识别目标: -")
        self.query_result = QLabel("坐标: -")

        self.live_table = self.make_table(("序号", "货物编号", "坐标"))
        self.all_table = self.make_table(("货物编号", "坐标"))
        self.route_canvas = RouteCanvas()
        self.query_input = QLineEdit()
        self.query_input.setPlaceholderText("输入 1~24 的货物编号")

        central = QWidget()
        root = QVBoxLayout(central)
        root.addLayout(self.make_top_bar())

        tabs = QTabWidget()
        tabs.addTab(self.make_live_tab(), "实时盘点")
        tabs.addTab(self.make_all_tab(), "全部结果")
        tabs.addTab(self.make_route_tab(), "航线图")
        root.addWidget(tabs)
        self.setCentralWidget(central)

        self.setStyleSheet(
            """
            QWidget { font-family: "WenQuanYi Zen Hei", "Noto Sans CJK SC", Arial; font-size: 20px; }
            QMainWindow { background: #eef2f7; }
            QLabel { color: #0f172a; }
            QTableWidget { background: white; gridline-color: #cbd5e1; }
            QHeaderView::section { background: #e2e8f0; padding: 8px; font-weight: 700; }
            QPushButton { background: #2563eb; color: white; border: 0; padding: 10px 18px; border-radius: 6px; }
            QLineEdit { background: white; border: 1px solid #94a3b8; padding: 9px; border-radius: 4px; }
            QTabWidget::pane { border: 1px solid #cbd5e1; background: white; }
            QTabBar::tab { padding: 12px 22px; }
            QTabBar::tab:selected { background: white; font-weight: 700; }
            """
        )

        self.clock = QTimer(self)
        self.clock.timeout.connect(self.update_clock)
        self.clock.start(500)

        self.led_timer = QTimer(self)
        self.led_timer.setSingleShot(True)
        self.led_timer.timeout.connect(self.turn_led_off)

        self.thread = QThread(self)
        self.receiver = Ros2Receiver() if source == "ros2" else Receiver(host, port)
        self.receiver.moveToThread(self.thread)
        self.thread.started.connect(self.receiver.run)
        self.receiver.message.connect(self.handle_message)
        self.receiver.state.connect(self.status_label.setText)
        self.thread.start()

    def make_top_bar(self):
        layout = QHBoxLayout()
        title = QLabel("D题立体货架盘点地面站")
        title.setStyleSheet("font-weight: 700;")
        layout.addWidget(title)
        layout.addStretch()
        layout.addWidget(self.count_label)
        layout.addWidget(self.phase_label)
        layout.addWidget(self.limit_label)
        layout.addWidget(self.timer_label)
        layout.addWidget(self.status_label)
        layout.addWidget(QLabel("LED"))
        layout.addWidget(self.led_dot)
        return layout

    def make_live_tab(self):
        widget = QWidget()
        layout = QVBoxLayout(widget)
        layout.addWidget(self.live_table)
        return widget

    def make_all_tab(self):
        widget = QWidget()
        layout = QVBoxLayout(widget)
        form = QFormLayout()
        button = QPushButton("查询")
        button.clicked.connect(self.query_item)
        self.query_input.returnPressed.connect(self.query_item)
        row = QHBoxLayout()
        row.addWidget(self.query_input)
        row.addWidget(button)
        form.addRow("编号", row)
        form.addRow("", self.query_result)
        layout.addLayout(form)
        layout.addWidget(self.all_table)
        return widget

    def make_route_tab(self):
        widget = QWidget()
        layout = QVBoxLayout(widget)
        layout.addWidget(self.recognized_label)
        layout.addWidget(self.route_canvas)
        return widget

    def make_table(self, headers):
        table = QTableWidget(0, len(headers))
        table.setHorizontalHeaderLabels(headers)
        table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        table.verticalHeader().setVisible(False)
        table.setEditTriggers(QTableWidget.NoEditTriggers)
        table.setSelectionBehavior(QTableWidget.SelectRows)
        return table

    def update_clock(self):
        elapsed = int(time.monotonic() - self.start_time)
        self.timer_label.setText(f"{elapsed // 60:02d}:{elapsed % 60:02d}")
        if elapsed > self.time_limit_s:
            self.timer_label.setStyleSheet("color: #dc2626; font-weight: 700;")
        else:
            self.timer_label.setStyleSheet("")

    def handle_message(self, data):
        msg_type = data.get("type")
        if msg_type == "reset":
            self.reset()
        elif msg_type == "status":
            self.handle_status(data)
        elif msg_type == "scan":
            self.handle_scan(data)
        elif msg_type == "recognized":
            item_id = str(data.get("id", ""))
            pos = self.format_pos(data.get("pos", ""))
            self.recognized_label.setText(f"识别目标: {item_id}  坐标: {pos}")
        elif msg_type == "route":
            self.route_canvas.set_route(data.get("target"), data.get("waypoints", []))
        elif msg_type == "query_result":
            self.handle_query_result(data)

    def handle_status(self, data):
        self.current_mode = int(data.get("mode", self.current_mode) or 1)
        self.time_limit_s = (
            TARGET_TIME_LIMIT_S if self.current_mode == 2 else INVENTORY_TIME_LIMIT_S
        )
        phase = data.get("phase", "-")
        phase_text = PHASE_NAMES.get(phase, phase)
        self.phase_label.setText(f"要求{self.current_mode} / {phase_text}")
        self.limit_label.setText(f"限时: {self.time_limit_s}s")

    def handle_scan(self, data):
        before = set(self.items)
        for row in data.get("all") or []:
            self.upsert_item(row.get("id"), row.get("pos", ""))
        item_id = str(data.get("id", ""))
        if item_id:
            self.upsert_item(item_id, data.get("pos", ""))
        if set(self.items) - before:
            self.flash_led()
        self.refresh_tables(item_id)

    def upsert_item(self, item_id, pos):
        item_id = str(item_id).strip()
        if not item_id:
            return
        pos_text = self.format_pos(pos)
        if item_id not in VALID_ITEM_IDS:
            self.status_label.setText(f"编号超出题目范围: {item_id}")
        elif pos_text and pos_text not in VALID_POSITIONS:
            self.status_label.setText(f"坐标超出题目范围: {pos_text}")
        self.items[item_id] = pos_text

    def refresh_tables(self, latest_id=""):
        self.count_label.setText(f"盘点: {len(self.items)}/{ITEM_COUNT}")
        if latest_id:
            pos = self.items.get(latest_id, "")
            row = self.live_table.rowCount()
            self.live_table.insertRow(row)
            values = [str(row + 1), latest_id, pos]
            for col, value in enumerate(values):
                self.live_table.setItem(row, col, QTableWidgetItem(value))
            self.live_table.scrollToBottom()

        self.all_table.setRowCount(0)
        for item_id in sorted(self.items, key=self.item_sort_key):
            row = self.all_table.rowCount()
            self.all_table.insertRow(row)
            values = [item_id, self.items[item_id]]
            for col, value in enumerate(values):
                self.all_table.setItem(row, col, QTableWidgetItem(value))

    def format_pos(self, pos):
        if isinstance(pos, str):
            return pos.strip().upper()
        if isinstance(pos, (list, tuple)) and pos:
            if len(pos) == 1:
                return str(pos[0]).strip().upper()
            return ",".join(str(value) for value in pos)
        return ""

    def item_sort_key(self, item_id):
        try:
            return int(item_id)
        except ValueError:
            return ITEM_COUNT + 1

    def query_item(self):
        item_id = self.query_input.text().strip()
        pos = self.items.get(item_id)
        self.query_result.setText(f"坐标: {pos}" if pos else "坐标: 未找到")

    def handle_query_result(self, data):
        item_id = data.get("id", "")
        if data.get("found"):
            self.query_result.setText(f"编号 {item_id} 坐标: {self.format_pos(data.get('pos', ''))}")
        else:
            self.query_result.setText(f"编号 {item_id} 坐标: 未找到")

    def flash_led(self):
        self.led.on()
        self.led_dot.set_on(True)
        self.led_timer.start(LED_FLASH_MS)

    def turn_led_off(self):
        self.led.off()
        self.led_dot.set_on(False)

    def reset(self):
        self.items.clear()
        self.live_table.setRowCount(0)
        self.all_table.setRowCount(0)
        self.count_label.setText(f"盘点: 0/{ITEM_COUNT}")
        self.query_result.setText("坐标: -")
        self.route_canvas.set_route("", [])
        self.start_time = time.monotonic()
        self.timer_label.setStyleSheet("")

    def closeEvent(self, event):
        self.receiver.stop()
        self.thread.quit()
        self.thread.wait(1000)
        self.led.cleanup()
        super().closeEvent(event)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--source", choices=("ros2", "tcp"), default=DEFAULT_SOURCE)
    parser.add_argument("--host", default=DEFAULT_HOST)
    parser.add_argument("--port", type=int, default=DEFAULT_PORT)
    parser.add_argument("--led-pin", type=int, default=DEFAULT_LED_PIN)
    parser.add_argument("--windowed", action="store_true")
    args = parser.parse_args()

    app = QApplication(sys.argv)
    window = GroundStation(args.host, args.port, args.led_pin, args.source)
    if args.windowed:
        window.show()
    else:
        window.showFullScreen()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
