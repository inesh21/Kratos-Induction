"""PyQt5 main window: waypoint entry, dispatch button, e-stop, telemetry, log."""

from PyQt5.QtWidgets import (
    QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QLabel,
    QLineEdit, QPushButton, QTextEdit, QMessageBox, QGroupBox
)

from .validators import validate_waypoint


class MainWindow(QMainWindow):
    def __init__(self, dispatch_callback, estop_callback):
        super().__init__()
        self.dispatch_callback = dispatch_callback
        self.estop_callback = estop_callback
        self.waypoint_inputs = []

        self.setWindowTitle("Kratos Mission Control")
        self.resize(600, 500)
        self.build_ui()

    def build_ui(self):
        central = QWidget()
        layout = QVBoxLayout()

        waypoint_box = QGroupBox("Waypoints")
        waypoint_layout = QVBoxLayout()
        for i in range(3):
            row = QHBoxLayout()
            row.addWidget(QLabel(f"Waypoint {i + 1}:  X ="))
            x_edit = QLineEdit()
            row.addWidget(x_edit)
            row.addWidget(QLabel("Y ="))
            y_edit = QLineEdit()
            row.addWidget(y_edit)
            waypoint_layout.addLayout(row)
            self.waypoint_inputs.append((x_edit, y_edit))
        waypoint_box.setLayout(waypoint_layout)
        layout.addWidget(waypoint_box)

        self.dispatch_button = QPushButton("Dispatch Mission")
        self.dispatch_button.clicked.connect(self.on_dispatch_clicked)
        layout.addWidget(self.dispatch_button)

        self.estop_button = QPushButton("EMERGENCY STOP")
        self.estop_button.setStyleSheet("background-color: red; color: white; font-weight: bold;")
        self.estop_button.clicked.connect(self.estop_callback)
        layout.addWidget(self.estop_button)

        telemetry_box = QGroupBox("Telemetry")
        telemetry_layout = QVBoxLayout()
        self.position_label = QLabel("Position: x=--, y=--")
        self.velocity_label = QLabel("Velocity: linear=--, angular=--")
        self.lidar_label = QLabel("Closest obstacle: -- m")
        telemetry_layout.addWidget(self.position_label)
        telemetry_layout.addWidget(self.velocity_label)
        telemetry_layout.addWidget(self.lidar_label)
        telemetry_box.setLayout(telemetry_layout)
        layout.addWidget(telemetry_box)

        layout.addWidget(QLabel("Mission Log:"))
        self.log_panel = QTextEdit()
        self.log_panel.setReadOnly(True)
        layout.addWidget(self.log_panel)

        central.setLayout(layout)
        self.setCentralWidget(central)

    def on_dispatch_clicked(self):
        waypoints = []
        for i, (x_edit, y_edit) in enumerate(self.waypoint_inputs, start=1):
            ok, x, y = validate_waypoint(x_edit.text(), y_edit.text())
            if not ok:
                QMessageBox.warning(self, "Invalid Input", f"Waypoint {i} has an empty or non-numeric value.")
                return
            waypoints.append((x, y))

        self.dispatch_button.setEnabled(False)
        self.log("Starting mission dispatch...")
        self.dispatch_callback(waypoints)

    def log(self, message):
        self.log_panel.append(message)

    def mission_finished(self, success_count, total):
        self.dispatch_button.setEnabled(True)
        self.log(f"Mission complete: {success_count}/{total} waypoints reached")

    def update_position(self, x, y):
        self.position_label.setText(f"Position: x={x:.2f}, y={y:.2f}")

    def update_velocity(self, linear, angular):
        self.velocity_label.setText(f"Velocity: linear={linear:.2f} m/s, angular={angular:.2f} rad/s")

    def update_lidar(self, min_distance):
        self.lidar_label.setText(f"Closest obstacle: {min_distance:.2f} m")