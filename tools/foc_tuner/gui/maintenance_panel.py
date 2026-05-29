"""Maintenance panel: electrical angle calibration + OTA firmware upgrade.

- Cali: triggers ElecAngleEstimate + Flash write on the MCU via the "Cali" command.
- OTA: lets the user pick a .bin file, computes size/CRC32 locally, and exposes
  an "Upload & Flash" entry point. The transfer protocol itself is stubbed
  pending firmware bootloader support — the UI is wired so adding the protocol
  later only touches core/protocol.py and a small upload worker.
"""

import os
import zlib

from PySide6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QLabel, QPushButton,
    QGroupBox, QCheckBox, QLineEdit, QFileDialog, QProgressBar,
    QMessageBox, QTextEdit
)
from PySide6.QtCore import Signal, Qt

from core.protocol import build_cali, build_version


class MaintenancePanel(QWidget):
    """Calibration + OTA firmware upgrade interface."""

    sig_command = Signal(str)  # Command to send to MCU

    def __init__(self, parent=None):
        super().__init__(parent)

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

        # Upgrade button + progress
        upg_row = QHBoxLayout()
        self._upload_btn = QPushButton("Upload && Flash")
        self._upload_btn.setEnabled(False)
        self._upload_btn.clicked.connect(self._on_upload_clicked)
        upg_row.addWidget(self._upload_btn)

        self._progress = QProgressBar()
        self._progress.setRange(0, 100)
        self._progress.setValue(0)
        self._progress.setTextVisible(True)
        upg_row.addWidget(self._progress, 1)
        v.addLayout(upg_row)

        note = QLabel(
            "Note: bootloader-side OTA receiver is not yet implemented in firmware. "
            "File selection, size, and CRC32 are computed locally; the upload "
            "transfer is a stub for now."
        )
        note.setWordWrap(True)
        note.setStyleSheet("QLabel { color: #aa6600; font-style: italic; }")
        v.addWidget(note)

        return group

    def _on_version_clicked(self):
        self.sig_command.emit(build_version())
        self._append_log(">>> version sent")

    def _on_browse_clicked(self):
        path, _ = QFileDialog.getOpenFileName(
            self,
            "Select firmware .bin file",
            "",
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
        # Stub: bootloader OTA protocol not yet defined on firmware side.
        # When implemented, build_ota_begin(size, crc) + chunked send + wait
        # for ACK should be wired through a background QThread to avoid
        # blocking the GUI.
        QMessageBox.information(
            self,
            "OTA not yet available",
            "Firmware bootloader OTA receiver is not implemented yet.\n\n"
            "This UI is reserved for the upcoming protocol — file path, size, "
            "and CRC32 are ready to send once the MCU side lands.",
        )
        self._append_log(
            f">>> upload requested: {os.path.basename(self._bin_path)} "
            f"({self._bin_size} B, crc=0x{self._bin_crc32:08X}) — stub, no transfer"
        )

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
