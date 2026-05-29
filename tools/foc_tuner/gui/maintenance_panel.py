"""Maintenance panel: electrical angle calibration + OTA firmware upgrade.

- Cali: triggers ElecAngleEstimate + Flash write on the MCU via the "Cali" command.
- OTA: lets the user pick a .bin file, computes size/CRC32 locally, and runs
  the upload via OTAUploader (background QThread).
"""

import os
import zlib

from PySide6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QLabel, QPushButton,
    QGroupBox, QCheckBox, QLineEdit, QFileDialog, QProgressBar,
    QMessageBox, QTextEdit, QSpinBox
)
from PySide6.QtCore import Signal, Qt, Slot, QSettings

from core.protocol import build_cali, build_version, build_ota_swap
from core.ota_worker import OTAUploader


class MaintenancePanel(QWidget):
    """Calibration + OTA firmware upgrade interface."""

    sig_command = Signal(str)  # Command to send to MCU (text path)
    sig_busy = Signal(bool)    # True = OTA in progress, lock other panels

    def __init__(self, serial_worker, parent=None):
        super().__init__(parent)

        self._serial = serial_worker
        self._uploader: OTAUploader | None = None

        self._bin_path: str | None = None
        self._bin_size: int = 0
        self._bin_crc32: int = 0

        layout = QVBoxLayout(self)

        layout.addWidget(self._build_cali_group())
        layout.addWidget(self._build_ota_group())

        # Shared status / log
        log_group = QGroupBox("Log")
        log_layout = QVBoxLayout(log_group)
        self._log_text = QTextEdit()
        self._log_text.setReadOnly(True)
        self._log_text.setStyleSheet(
            "QTextEdit { font-family: Consolas, 'Courier New', monospace; }"
        )
        self._log_text.setMaximumHeight(140)
        log_layout.addWidget(self._log_text)
        layout.addWidget(log_group)

        layout.addStretch()

    # ------------------------------------------------------------------ Cali

    def _build_cali_group(self) -> QGroupBox:
        group = QGroupBox("Electrical Angle Calibration (Cali)")
        v = QVBoxLayout(group)

        info = QLabel(
            "Runs ElecAngleEstimate, identifies the electrical angle offset, "
            "and persists it to Flash. Motor will rotate slightly during the "
            "process — keep the shaft free of load. Stops FOC and clears "
            "fault flags before running."
        )
        info.setWordWrap(True)
        v.addWidget(info)

        ack_row = QHBoxLayout()
        self._cali_ack_cb = QCheckBox(
            "I confirm the motor shaft is free and safe to rotate"
        )
        self._cali_ack_cb.toggled.connect(self._on_cali_ack_toggled)
        ack_row.addWidget(self._cali_ack_cb)
        ack_row.addStretch()
        v.addLayout(ack_row)

        btn_row = QHBoxLayout()
        self._cali_btn = QPushButton("Run Calibration")
        self._cali_btn.setEnabled(False)
        self._cali_btn.clicked.connect(self._on_cali_clicked)
        btn_row.addWidget(self._cali_btn)

        self._cali_status_label = QLabel("Idle")
        self._cali_status_label.setStyleSheet("QLabel { color: gray; }")
        btn_row.addWidget(self._cali_status_label)
        btn_row.addStretch()
        v.addLayout(btn_row)

        return group

    def _on_cali_ack_toggled(self, checked: bool):
        self._cali_btn.setEnabled(checked)

    def _on_cali_clicked(self):
        self.sig_command.emit(build_cali())
        self._cali_status_label.setText("Calibrating...")
        self._cali_status_label.setStyleSheet(
            "QLabel { color: orange; font-weight: bold; }"
        )
        self._append_log(">>> Cali sent — waiting for MCU...")

    # ------------------------------------------------------------------- OTA

    def _build_ota_group(self) -> QGroupBox:
        group = QGroupBox("OTA Firmware Upgrade (reserved)")
        v = QVBoxLayout(group)

        # Version query
        ver_row = QHBoxLayout()
        self._version_btn = QPushButton("Query Firmware Version")
        self._version_btn.clicked.connect(self._on_version_clicked)
        ver_row.addWidget(self._version_btn)

        self._version_label = QLabel("version: ?")
        ver_row.addWidget(self._version_label)
        ver_row.addStretch()
        v.addLayout(ver_row)

        # File picker row
        file_row = QHBoxLayout()
        file_row.addWidget(QLabel(".bin file:"))
        self._bin_path_edit = QLineEdit()
        self._bin_path_edit.setReadOnly(True)
        self._bin_path_edit.setPlaceholderText("No file selected")
        file_row.addWidget(self._bin_path_edit, 1)

        self._browse_btn = QPushButton("Browse...")
        self._browse_btn.clicked.connect(self._on_browse_clicked)
        file_row.addWidget(self._browse_btn)
        v.addLayout(file_row)

        # File info
        self._bin_info_label = QLabel("size: -    crc32: -")
        self._bin_info_label.setStyleSheet("QLabel { color: gray; }")
        v.addWidget(self._bin_info_label)

        # Version (free-form integer, stored in App-B header)
        ver_in = QHBoxLayout()
        ver_in.addWidget(QLabel("Version:"))
        self._version_spin = QSpinBox()
        self._version_spin.setRange(0, 0xFFFF)
        self._version_spin.setValue(1)
        ver_in.addWidget(self._version_spin)
        ver_in.addStretch()
        v.addLayout(ver_in)

        # Upgrade button + progress
        upg_row = QHBoxLayout()
        self._upload_btn = QPushButton("Upload && Flash")
        self._upload_btn.setEnabled(False)
        self._upload_btn.clicked.connect(self._on_upload_clicked)
        upg_row.addWidget(self._upload_btn)

        self._cancel_btn = QPushButton("Cancel")
        self._cancel_btn.setEnabled(False)
        self._cancel_btn.clicked.connect(self._on_cancel_clicked)
        upg_row.addWidget(self._cancel_btn)

        self._progress = QProgressBar()
        self._progress.setRange(0, 100)
        self._progress.setValue(0)
        self._progress.setTextVisible(True)
        upg_row.addWidget(self._progress, 1)
        v.addLayout(upg_row)

        note = QLabel(
            "Stage 1: writes firmware into App-B slot (Bank2 Sector 0~5) "
            "and verifies CRC32. Stage 2 bootloader (slot swapping) is "
            "not yet implemented — App-A keeps running after upload."
        )
        note.setWordWrap(True)
        note.setStyleSheet("QLabel { color: #aa6600; font-style: italic; }")
        v.addWidget(note)

        return group

    def _on_version_clicked(self):
        self.sig_command.emit(build_version())
        self._append_log(">>> version sent")

    def _on_browse_clicked(self):
        settings = QSettings()
        last_dir = settings.value("ota/last_bin_dir", "", type=str)
        path, _ = QFileDialog.getOpenFileName(
            self,
            "Select firmware .bin file",
            last_dir,
            "Firmware Binary (*.bin);;All Files (*.*)",
        )
        if not path:
            return
        try:
            with open(path, "rb") as f:
                data = f.read()
        except OSError as e:
            QMessageBox.warning(self, "Read failed", f"Could not read file:\n{e}")
            return

        # Remember the directory for next time
        settings.setValue("ota/last_bin_dir", os.path.dirname(path))

        self._bin_path = path
        self._bin_size = len(data)
        self._bin_crc32 = zlib.crc32(data) & 0xFFFFFFFF

        self._bin_path_edit.setText(path)
        self._bin_info_label.setText(
            f"size: {self._bin_size} bytes ({self._bin_size / 1024:.1f} KB)"
            f"    crc32: 0x{self._bin_crc32:08X}"
        )
        self._bin_info_label.setStyleSheet("QLabel { color: black; }")
        self._upload_btn.setEnabled(True)
        self._progress.setValue(0)
        self._append_log(
            f">>> selected {os.path.basename(path)}  "
            f"size={self._bin_size}  crc32=0x{self._bin_crc32:08X}"
        )

    def _on_upload_clicked(self):
        if not self._bin_path:
            return
        if self._uploader is not None and self._uploader.isRunning():
            return  # already running
        try:
            with open(self._bin_path, "rb") as f:
                bin_data = f.read()
        except OSError as e:
            QMessageBox.warning(self, "Read failed", f"Could not read file:\n{e}")
            return

        # Sanity-check size against App-B slot capacity (768KB)
        if len(bin_data) > 768 * 1024:
            QMessageBox.warning(
                self, "File too large",
                f"Firmware is {len(bin_data)} bytes, but App-B slot is only 768 KB."
            )
            return

        self._progress.setValue(0)
        self._upload_btn.setEnabled(False)
        self._cancel_btn.setEnabled(True)
        self.sig_busy.emit(True)
        self._append_log(
            f">>> upload start: {os.path.basename(self._bin_path)} "
            f"({len(bin_data)} B, crc=0x{self._bin_crc32:08X})"
        )

        self._uploader = OTAUploader(
            serial_worker=self._serial,
            bin_data=bin_data,
            version=self._version_spin.value(),
        )
        # Route serial line events into the uploader's ACK queue
        self._serial.sig_line_received.connect(self._uploader.on_line)
        self._uploader.sig_progress.connect(self._on_upload_progress)
        self._uploader.sig_status.connect(self._append_log)
        self._uploader.sig_done.connect(self._on_upload_done)
        self._uploader.start()

    def _on_cancel_clicked(self):
        if self._uploader is not None and self._uploader.isRunning():
            self._uploader.cancel()
            self._cancel_btn.setEnabled(False)
            self._append_log(">>> cancellation requested")

    @Slot(int, int)
    def _on_upload_progress(self, sent: int, total: int):
        if total > 0:
            self._progress.setValue(int(sent * 100 / total))

    @Slot(bool, str)
    def _on_upload_done(self, ok: bool, msg: str):
        # Disconnect line routing so a future upload doesn't double-receive
        if self._uploader is not None:
            try:
                self._serial.sig_line_received.disconnect(self._uploader.on_line)
            except (RuntimeError, TypeError):
                pass

        self._upload_btn.setEnabled(True)
        self._cancel_btn.setEnabled(False)
        self.sig_busy.emit(False)

        if ok:
            self._append_log(f">>> OTA SUCCESS: {msg}")
            reply = QMessageBox.question(
                self, "Upload complete",
                "Firmware written to App-B and CRC verified.\n\n"
                "Reboot now to let the bootloader swap to the new slot?\n"
                "(Stage 2 not yet implemented — reboot will resume App-A)",
                QMessageBox.Yes | QMessageBox.No, QMessageBox.No,
            )
            if reply == QMessageBox.Yes:
                self.sig_command.emit(build_ota_swap())
        else:
            self._append_log(f">>> OTA FAILED: {msg}")
            QMessageBox.warning(self, "Upload failed", msg)

    # ----------------------------------------------------------- Misc / log

    def _append_log(self, text: str):
        self._log_text.append(text)

    def process_line(self, line: str):
        """Forward serial output relevant to this panel."""
        # Cali results
        if "Cali done" in line:
            self._cali_status_label.setText("Done")
            self._cali_status_label.setStyleSheet(
                "QLabel { color: green; font-weight: bold; }"
            )
            self._append_log(line)
            return

        if "Cali: Flash erase FAIL" in line:
            self._cali_status_label.setText("Failed")
            self._cali_status_label.setStyleSheet(
                "QLabel { color: red; font-weight: bold; }"
            )
            self._append_log(line)
            return

        # Version banner — firmware prints something like "version: vX.Y.Z"
        # or includes a "version" token in the boot banner. Capture loosely.
        lower = line.lower()
        if lower.startswith("version") or "firmware version" in lower:
            self._version_label.setText(line.strip())
            self._append_log(line)
