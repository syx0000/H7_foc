"""Main window for FOC Tuner application.

Assembles all panels and manages data flow between serial worker and GUI.
"""

from PySide6.QtWidgets import (
    QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QSplitter, QTabWidget
)
from PySide6.QtCore import Qt, Slot

from core.serial_worker import SerialWorker
from core.data_model import DataModel
from core.parser import parse_line, get_channel_names, set_active_logid
from gui.serial_panel import SerialPanel
from gui.console_widget import ConsoleWidget
from gui.waveform_widget import WaveformWidget
from gui.motor_control_panel import MotorControlPanel
from gui.pid_panel import PIDPanel
from gui.bandwidth_test_panel import BandwidthTestPanel
from gui.flash_panel import FlashPanel
from gui.fault_panel import FaultPanel
from gui.maintenance_panel import MaintenancePanel


class MainWindow(QMainWindow):
    """Main application window."""

    def __init__(self):
        super().__init__()
        self.setWindowTitle("FOC Motor Tuner")
        self.resize(1400, 900)

        # Core components
        self._serial = SerialWorker()
        self._can = None  # 懒加载, 避免无创芯 DLL 时启动报错
        self._active_worker = self._serial  # 默认串口
        self._data_model = DataModel(buffer_size=10000)

        # Build UI
        central = QWidget()
        self.setCentralWidget(central)
        main_layout = QVBoxLayout(central)

        # Top: Serial connection panel
        self._serial_panel = SerialPanel()
        main_layout.addWidget(self._serial_panel)

        # Main horizontal split: left side (tabs over waveform) | console
        splitter = QSplitter(Qt.Horizontal)

        # Left side: vertical split with tabs on top, waveform below
        left_split = QSplitter(Qt.Vertical)

        # Top: Tabbed control panels
        left_tabs = QTabWidget()
        self._control_panel = MotorControlPanel()
        left_tabs.addTab(self._control_panel, "Motor Control")

        self._pid_panel = PIDPanel()
        left_tabs.addTab(self._pid_panel, "PID Tuning")

        self._bw_panel = BandwidthTestPanel()
        left_tabs.addTab(self._bw_panel, "BW Test")

        self._flash_panel = FlashPanel()
        left_tabs.addTab(self._flash_panel, "Flash")

        self._fault_panel = FaultPanel()
        left_tabs.addTab(self._fault_panel, "Faults")

        self._maint_panel = MaintenancePanel(self._serial)
        left_tabs.addTab(self._maint_panel, "Maintenance")

        left_split.addWidget(left_tabs)

        # Bottom: Waveform display
        self._waveform = WaveformWidget(self._data_model)
        left_split.addWidget(self._waveform)

        # Tabs keep their natural height; waveform absorbs vertical resize
        left_split.setStretchFactor(0, 0)
        left_split.setStretchFactor(1, 1)
        left_split.setSizes([180, 800])

        # Only Motor Control needs the compact height (it's wide-but-short by design).
        # Other tabs keep their natural full size — switching to them expands the tab area.
        self._left_tabs = left_tabs
        self._left_split = left_split
        left_tabs.currentChanged.connect(self._on_tab_changed)
        # Apply initial height for the default tab (index 0 = Motor Control)
        self._on_tab_changed(0)

        splitter.addWidget(left_split)

        # Right: Console (unchanged)
        self._console = ConsoleWidget()
        splitter.addWidget(self._console)

        # Horizontal sizes: left side wide, console fixed
        splitter.setSizes([980, 420])
        splitter.setStretchFactor(0, 1)
        splitter.setStretchFactor(1, 0)

        main_layout.addWidget(splitter)

        # Connect signals
        self._serial_panel.sig_connect_request.connect(self._on_connect_request)
        self._serial_panel.sig_disconnect_request.connect(self._on_disconnect_request)
        self._serial_panel.sig_reset_request.connect(self._on_reset_request)
        self._serial.sig_line_received.connect(self._on_line_received)
        self._serial.sig_error.connect(self._on_error)
        self._serial.sig_connected.connect(self._on_connected)
        self._control_panel.sig_command.connect(self._send)
        self._control_panel.sig_wly_speed.connect(self._send_wly_speed)
        self._control_panel.sig_wly_position.connect(self._send_wly_position)
        self._control_panel.sig_wly_torque.connect(self._send_wly_torque)
        self._control_panel.sig_wly_enable.connect(self._send_wly_enable)
        self._control_panel._logid_combo.currentIndexChanged.connect(self._on_logid_changed)
        self._pid_panel.sig_command.connect(self._send)
        self._bw_panel.sig_command.connect(self._send)
        self._flash_panel.sig_command.connect(self._send)
        self._fault_panel.sig_command.connect(self._send)
        self._maint_panel.sig_command.connect(self._send)
        self._console.sig_send_command.connect(self._send)

        # Disabled until serial connects
        self._control_panel.setEnabled(False)
        self._pid_panel.setEnabled(False)
        self._bw_panel.setEnabled(False)
        self._flash_panel.setEnabled(False)
        self._fault_panel.setEnabled(False)
        self._maint_panel.setEnabled(False)

        # Initialize waveform channels for default logid (50 - speed)
        self._update_waveform_channels(50)

    @Slot(str, dict)
    def _on_connect_request(self, backend: str, params: dict):
        """Handle connection request from serial panel.

        backend == 'serial': params = {port, baud}
        backend == 'can':    params = {channel, abit, dbit}
        """
        # 通知 Motor Control 面板当前后端类型
        self._control_panel.set_backend(backend)

        if backend == "serial":
            self._active_worker = self._serial
            self._serial.connect_port(params["port"], params["baud"])
        else:
            # 懒加载 CanWorker (避免无 DLL 时启动报错)
            if not hasattr(self, '_can') or self._can is None:
                from core.can_worker import CanWorker
                self._can = CanWorker()
                self._can.sig_line_received.connect(self._on_line_received)
                self._can.sig_error.connect(self._on_error)
                self._can.sig_connected.connect(self._on_connected)
            self._active_worker = self._can
            self._can.connect_port(
                channel=params["channel"], abit=params["abit"], dbit=params["dbit"]
            )

    @Slot()
    def _on_disconnect_request(self):
        """Handle disconnection request."""
        if self._active_worker:
            self._active_worker.disconnect_port()

    @Slot(str)
    def _send(self, cmd: str):
        """Forward command to active worker (Serial or CAN)."""
        if self._active_worker:
            self._active_worker.send(cmd)

    @Slot(float)
    def _send_wly_speed(self, speed_rpm: float):
        """发送万里扬速度指令 (仅 CAN 模式)."""
        if hasattr(self._active_worker, 'send_wly_speed'):
            self._active_worker.send_wly_speed(speed_rpm)

    @Slot(float, float)
    def _send_wly_position(self, pos_deg: float, speed_rpm: float):
        """发送万里扬位置指令 (仅 CAN 模式)."""
        if hasattr(self._active_worker, 'send_wly_position'):
            self._active_worker.send_wly_position(pos_deg, speed_rpm)

    @Slot(float)
    def _send_wly_torque(self, torque_nm: float):
        """发送万里扬转矩指令 (仅 CAN 模式)."""
        if hasattr(self._active_worker, 'send_wly_torque'):
            self._active_worker.send_wly_torque(torque_nm)

    @Slot(bool)
    def _send_wly_enable(self, enable: bool):
        """发送万里扬使能控制 (仅 CAN 模式)."""
        if hasattr(self._active_worker, 'send_wly_enable'):
            self._active_worker.send_wly_enable(enable)

    @Slot()
    def _on_reset_request(self):
        """Send MCU reset command and sync UI state."""
        from core.protocol import build_reset
        self._send(build_reset())
        # MCU will reboot, PWM state goes away — uncheck the Enable button
        # silently (no further command needed; MCU is mid-reset).
        if self._control_panel._enable_btn.isChecked():
            self._control_panel._enable_btn.blockSignals(True)
            self._control_panel._enable_btn.setChecked(False)
            self._control_panel._enable_btn.blockSignals(False)
        self._console.append_line(">>> Reset sent, MCU rebooting...")

    @Slot(bool)
    def _on_connected(self, connected: bool):
        """Update UI when connection state changes."""
        self._serial_panel.set_connected(connected)

        # Enable functional panels only when connected
        self._control_panel.setEnabled(connected)
        self._pid_panel.setEnabled(connected)
        self._bw_panel.setEnabled(connected)
        self._flash_panel.setEnabled(connected)
        self._fault_panel.setEnabled(connected)
        self._maint_panel.setEnabled(connected)
        self._console.set_send_enabled(connected)

        if connected:
            self._console.append_line("=== Connected ===")
            # Send logfreq only; logid stays 0 until user enables logging
            from core.protocol import build_logfreq, build_logid
            self._send(build_logid(0))
            self._send(build_logfreq(10))
            # If user is already on the PID tab, auto-query params now
            if self._left_tabs.currentIndex() == 1:
                self._pid_panel.query_params()
        else:
            self._console.append_line("=== Disconnected ===")

    @Slot(str)
    def _on_line_received(self, line: str):
        """Process received line from serial port."""
        # Always show in console
        self._console.append_line(line)

        # Pass to bandwidth test panel for result parsing
        self._bw_panel.process_line(line)

        # Pass to flash panel for Flash-related output
        self._flash_panel.process_line(line)

        # Pass to fault panel for fault detection
        self._fault_panel.process_line(line)

        # Pass to maintenance panel for Cali / version output
        self._maint_panel.process_line(line)

        # Pass to PID panel for getparams response
        self._pid_panel.process_line(line)

        # Try to parse
        frame = parse_line(line)
        if frame:
            self._data_model.append(frame.logid, frame.timestamp, frame.fields)

            # Update waveform channels if logid changed
            current_logid = self._control_panel.get_current_logid()
            if frame.logid == current_logid:
                # Check if we need to update channel list
                expected_channels = get_channel_names(frame.logid)
                if expected_channels and not self._waveform._curves:
                    self._update_waveform_channels(frame.logid)

    @Slot(str)
    def _on_error(self, error_msg: str):
        """Display error message in console."""
        self._console.append_line(f"ERROR: {error_msg}")

    def _update_waveform_channels(self, logid: int):
        """Update waveform display for new logid.

        Args:
            logid: Log ID to display
        """
        set_active_logid(logid)
        channels = get_channel_names(logid)
        if channels:
            self._waveform.set_channels(channels)

    @Slot()
    def _on_logid_changed(self):
        """Handle logid change from control panel."""
        logid = self._control_panel.get_current_logid()
        self._update_waveform_channels(logid)

    @Slot(int)
    def _on_tab_changed(self, index: int):
        """Resize the tab area based on which tab is active.

        - Motor Control (0): compact 180px, waveform takes the rest.
        - PID Tuning (1): natural ~400px, waveform takes the rest. Auto-query
          current PID + phase comp values from MCU.
        - BW Test / Flash / Faults / Maintenance (2/3/4/5): hide waveform — these
          pages have their own result views and don't need live plots.
        """
        # Tabs that don't need the waveform pane visible
        WAVEFORM_HIDDEN_TABS = (2, 3, 4, 5)

        # Auto-query on PID tab activation (only if connected)
        if index == 1 and self._serial_panel._connected:
            self._pid_panel.query_params()

        if index in WAVEFORM_HIDDEN_TABS:
            self._waveform.hide()
            self._left_tabs.setMaximumHeight(16777215)
            return

        # Waveform-bearing tabs
        self._waveform.show()
        cur = self._left_split.sizes()
        total = sum(cur) if sum(cur) > 0 else 980

        if index == 0:
            # Motor Control: compact horizontal layout
            self._left_tabs.setMaximumHeight(220)
            self._left_split.setSizes([180, max(total - 180, 200)])
        else:
            # PID Tuning: full size
            self._left_tabs.setMaximumHeight(16777215)
            self._left_split.setSizes([400, max(total - 400, 200)])

    def closeEvent(self, event):
        """Clean up on window close."""
        self._serial.disconnect_port()
        if self._can:
            self._can.disconnect_port()
        event.accept()
