#!/usr/bin/env python3
"""On-screen ROS 2 rover joystick. Run inside the sourced ROS container.

    source ./setup_terminal.sh
    python3 joystick.py

Options: --topic /diff_drive_controller/cmd_vel --max-linear 0.5
         --max-angular 1.0 --real-time (use wall time for physical hardware)

Drag and hold to drive. Release, Space, Escape, focus loss, or closing stops.
No installation or changes to launch files are required.
"""

import argparse
import math
import signal
import sys
import time

try:
    import rclpy
    from geometry_msgs.msg import TwistStamped
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.signals import SignalHandlerOptions
    from PyQt5.QtCore import QEvent, QPointF, Qt, QTimer, pyqtSignal
    from PyQt5.QtGui import QColor, QFont, QPainter, QPen
    from PyQt5.QtWidgets import (
        QApplication, QDoubleSpinBox, QHBoxLayout, QLabel, QPushButton,
        QVBoxLayout, QWidget,
    )
except ImportError as error:
    raise SystemExit(
        f'{error}\nRun inside the ROS container after: source ./setup_terminal.sh'
    ) from error


# Rover control desk: graphite panels, amber motion, green link, red stop.
# One large spring-centered pad replaces directional buttons; a two-axis
# readout replaces gauges. Quiet borders and an 8 px spacing scale keep the
# small window legible beside Gazebo. Numeric values use a fixed-width font.
BG = '#171c21'
PANEL = '#20272e'
BORDER = '#36414a'
TEXT = '#edf1f3'
MUTED = '#9baab5'
ACCENT = '#edb65a'
GREEN = '#82c5a3'
RED = '#ef8a84'


def drive_axes(x, y, deadzone=0.06):
    """Convert screen coordinates to normalized forward and left-turn demand."""
    magnitude = math.hypot(x, y)
    if magnitude <= deadzone:
        return 0.0, 0.0
    amount = (min(magnitude, 1.0) - deadzone) / (1.0 - deadzone)
    return -y / magnitude * amount, -x / magnitude * amount


class JoystickPad(QWidget):
    """A circular, mouse-held joystick which always springs back on release."""

    moved = pyqtSignal(float, float)

    def __init__(self):
        super().__init__()
        self.setMinimumSize(320, 320)
        self.setFocusPolicy(Qt.StrongFocus)
        self.setCursor(Qt.OpenHandCursor)
        self.active = False
        self.position = QPointF()
        self.setAccessibleName('Rover joystick')
        self.setToolTip('Hold and drag to drive. Release to stop.')

    def geometry_values(self):
        return QPointF(self.width() / 2, self.height() / 2), min(
            self.width(), self.height()) / 2 - 48

    def reset(self):
        self.active = False
        self.position = QPointF()
        self.setCursor(Qt.OpenHandCursor)
        self.moved.emit(0.0, 0.0)
        self.update()

    def move_to(self, point):
        center, radius = self.geometry_values()
        dx, dy = (point.x() - center.x()) / radius, (point.y() - center.y()) / radius
        distance = max(1.0, math.hypot(dx, dy))
        self.position = QPointF(dx / distance, dy / distance)
        self.moved.emit(*drive_axes(self.position.x(), self.position.y()))
        self.update()

    def mousePressEvent(self, event):
        if event.button() == Qt.LeftButton:
            self.setFocus()
            self.active = True
            self.setCursor(Qt.ClosedHandCursor)
            self.move_to(event.localPos())
            event.accept()

    def mouseMoveEvent(self, event):
        if self.active and event.buttons() & Qt.LeftButton:
            self.move_to(event.localPos())

    def mouseReleaseEvent(self, event):
        if event.button() == Qt.LeftButton:
            self.reset()
            event.accept()

    def event(self, event):
        if event.type() == QEvent.UngrabMouse and self.active:
            self.reset()
        return super().event(event)

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        center, radius = self.geometry_values()
        painter.setBrush(QColor(PANEL))
        painter.setPen(QPen(QColor(BORDER), 1))
        painter.drawEllipse(center, radius, radius)
        painter.setBrush(Qt.NoBrush)
        painter.drawEllipse(center, radius * 0.55, radius * 0.55)
        painter.setPen(QPen(QColor(BORDER), 1, Qt.DashLine))
        painter.drawLine(QPointF(center.x() - radius, center.y()),
                         QPointF(center.x() + radius, center.y()))
        painter.drawLine(QPointF(center.x(), center.y() - radius),
                         QPointF(center.x(), center.y() + radius))
        painter.setPen(QColor(MUTED))
        painter.setFont(QFont('DejaVu Sans', 9))
        for label, x, y in [('FORWARD', 0, -radius - 24),
                            ('REVERSE', 0, radius + 24),
                            ('L', -radius - 24, 0), ('R', radius + 24, 0)]:
            width = painter.fontMetrics().horizontalAdvance(label)
            painter.drawText(QPointF(center.x() + x - width / 2,
                                     center.y() + y + 4), label)
        stick = center + self.position * radius
        color = QColor(ACCENT if self.active else MUTED)
        painter.setPen(QPen(color, 3))
        painter.drawLine(center, stick)
        painter.setBrush(QColor(PANEL))
        painter.setPen(QPen(color, 2))
        painter.drawEllipse(stick, 23, 23)
        painter.setBrush(color)
        painter.setPen(Qt.NoPen)
        painter.drawEllipse(stick, 5, 5)


class RoverJoystick(QWidget):
    """Keep Qt and ROS callbacks on one non-blocking event loop."""

    def __init__(self, node, topic, max_linear, max_angular, simulation=True):
        super().__init__()
        self.node = node
        self.publisher = node.create_publisher(TwistStamped, topic, 10)
        self.simulation = simulation
        self.forward = self.turn = 0.0
        self.last_clock = None
        self.clock_changed = time.monotonic()
        self.setWindowTitle('URC · Rover joystick')
        self.resize(400, 600)
        self.setMinimumWidth(360)
        self.setStyleSheet(f'''
            QWidget {{ background: {BG}; color: {TEXT}; font: 11pt 'DejaVu Sans'; }}
            QLabel#muted {{ color: {MUTED}; font-size: 9pt; }}
            QDoubleSpinBox {{ background: {PANEL}; border: 1px solid {BORDER};
                             border-radius: 4px; padding: 6px; }}
            QPushButton {{ background: #382628; color: {RED}; border: 1px solid #694044;
                           border-radius: 5px; padding: 12px; font-weight: bold; }}
            QPushButton:hover {{ background: #4a2e30; }}
            QPushButton:pressed {{ background: #633638; }}
            QPushButton:focus {{ border: 2px solid {RED}; }}
        ''')
        layout = QVBoxLayout(self)
        layout.setContentsMargins(24, 24, 24, 24)
        layout.setSpacing(16)
        title = QLabel('ROVER / MANUAL DRIVE')
        title.setStyleSheet('font-size: 12pt; font-weight: bold;')
        layout.addWidget(title)
        self.status = QLabel('Connecting…')
        self.status.setStyleSheet(f'color: {MUTED}; font-size: 9pt;')
        layout.addWidget(self.status)
        self.pad = JoystickPad()
        self.pad.moved.connect(self.set_demand)
        layout.addWidget(self.pad, 1)
        self.readout = QLabel()
        self.readout.setAlignment(Qt.AlignCenter)
        self.readout.setStyleSheet('font: 14pt "DejaVu Sans Mono";')
        layout.addWidget(self.readout)
        controls = QHBoxLayout()
        self.linear_limit = self.limit_control(controls, 'Speed limit', max_linear, 2.0, ' m/s')
        self.angular_limit = self.limit_control(controls, 'Turn limit', max_angular, 3.0, ' rad/s')
        layout.addLayout(controls)
        stop = QPushButton('STOP  /  SPACE')
        stop.clicked.connect(self.halt)
        layout.addWidget(stop)
        hint = QLabel('Hold to drive · Release to stop\nUp/down: drive    Left/right: turn')
        hint.setObjectName('muted')
        hint.setAlignment(Qt.AlignCenter)
        layout.addWidget(hint)
        self.update_readout()
        # Wall-clock Qt timer also keeps stop/status responsive when Gazebo pauses.
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.tick)
        self.timer.start(50)
        QApplication.instance().installEventFilter(self)

    def limit_control(self, row, label, value, maximum, suffix):
        column = QVBoxLayout()
        column.setSpacing(8)
        text = QLabel(label)
        text.setObjectName('muted')
        column.addWidget(text)
        control = QDoubleSpinBox()
        control.setRange(0.05, maximum)
        control.setDecimals(2)
        control.setSingleStep(0.05)
        control.setValue(value)
        control.setSuffix(suffix)
        control.valueChanged.connect(self.update_readout)
        column.addWidget(control)
        row.addLayout(column)
        return control

    def set_demand(self, forward, turn):
        self.forward, self.turn = forward, turn
        self.update_readout()

    def velocities(self):
        return self.forward * self.linear_limit.value(), self.turn * self.angular_limit.value()

    def update_readout(self, *args):
        if not hasattr(self, 'angular_limit'):
            return
        linear, angular = self.velocities()
        self.readout.setText(f'{linear:+.2f} m/s   {angular:+.2f} rad/s')

    def publish(self, linear=0.0, angular=0.0):
        if not rclpy.ok():
            return
        msg = TwistStamped()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.twist.linear.x = float(linear)
        msg.twist.angular.z = float(angular)
        self.publisher.publish(msg)

    def halt(self):
        self.pad.reset()
        self.publish()

    def tick(self):
        if not rclpy.ok():
            self.pad.reset()
            QApplication.instance().quit()
            return
        rclpy.spin_once(self.node, timeout_sec=0)
        now = self.node.get_clock().now().nanoseconds
        if now != self.last_clock:
            self.last_clock = now
            self.clock_changed = time.monotonic()
        clock_ok = not self.simulation or (now > 0 and time.monotonic() - self.clock_changed < 1)
        connected = self.publisher.get_subscription_count() > 0
        ready = connected and clock_ok
        if not ready:
            self.pad.reset()
        if not connected:
            text, color = '●  Waiting for drive connection', MUTED
        elif not clock_ok:
            text, color = '●  Simulation clock paused or unavailable', ACCENT
        elif self.pad.active:
            text, color = '●  Driving — release to stop', ACCENT
        else:
            text, color = '●  Connected · stick centered', GREEN
        self.status.setText(text)
        self.status.setStyleSheet(f'color: {color}; font-size: 9pt;')
        self.publish(*self.velocities())

    def eventFilter(self, watched, event):
        if event.type() == QEvent.ApplicationDeactivate:
            self.halt()
        elif (event.type() == QEvent.WindowDeactivate and watched is self):
            self.halt()
        elif event.type() == QEvent.KeyPress and event.key() in (Qt.Key_Space, Qt.Key_Escape):
            self.halt()
            return True
        return super().eventFilter(watched, event)

    def closeEvent(self, event):
        self.timer.stop()
        self.halt()
        event.accept()


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--topic', default='/diff_drive_controller/cmd_vel')
    parser.add_argument('--max-linear', type=float, default=0.5, help='initial speed limit, m/s (0.05–2)')
    parser.add_argument('--max-angular', type=float, default=1.0, help='initial turn limit, rad/s (0.05–3)')
    parser.add_argument('--real-time', action='store_true', help='use wall time instead of the Gazebo clock')
    options, ros_args = parser.parse_known_args()
    if not (0.05 <= options.max_linear <= 2 and 0.05 <= options.max_angular <= 3):
        parser.error('speed limits must be within their documented ranges')
    app = QApplication(sys.argv[:1])
    app.setApplicationName('URC Rover Joystick')
    rclpy.init(args=ros_args, signal_handler_options=SignalHandlerOptions.NO)
    node = Node('onscreen_joystick', parameter_overrides=[
        Parameter('use_sim_time', value=not options.real_time),
    ])
    window = RoverJoystick(node, options.topic, options.max_linear, options.max_angular,
                           simulation=not options.real_time)

    def quit_app(signum, frame):
        window.halt()
        app.quit()

    signal.signal(signal.SIGINT, quit_app)
    signal.signal(signal.SIGTERM, quit_app)
    window.show()
    try:
        return app.exec_()
    finally:
        window.timer.stop()
        for _ in range(3):
            window.publish()
            if rclpy.ok():
                rclpy.spin_once(node, timeout_sec=0)
            time.sleep(0.02)
        app.removeEventFilter(window)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
