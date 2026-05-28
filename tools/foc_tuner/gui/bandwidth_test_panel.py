"""Bandwidth test panel for running identification and autoTune sequences.

Provides buttons for bwtest1-9 and displays results in Bode plots.
"""

from PySide6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QLabel, QPushButton,
    QGroupBox, QTextEdit, QSplitter
)
from PySide6.QtCore import Signal, Qt
import numpy as np
from gui.bode_widget import BodePlotWidget


class BandwidthTestPanel(QWidget):
    """Bandwidth test control and result display."""

    sig_command = Signal(str)  # Command to send

    def __init__(self, parent=None):
        super().__init__(parent)

        layout = QVBoxLayout(self)

        # Top: Test buttons
        button_group = QGroupBox("Bandwidth Tests")
        button_layout = QVBoxLayout(button_group)

        # Row 1: BW tests
        bw_layout = QHBoxLayout()
        bw_layout.addWidget(QLabel("Bandwidth:"))

        self._bw1_btn = QPushButton("Current Loop (bwtest1)")
        self._bw1_btn.clicked.connect(lambda: self._send_bwtest(1))
        bw_layout.addWidget(self._bw1_btn)

        self._bw2_btn = QPushButton("Speed Loop (bwtest2)")
        self._bw2_btn.clicked.connect(lambda: self._send_bwtest(2))
        bw_layout.addWidget(self._bw2_btn)

        self._bw9_btn = QPushButton("Position Loop (bwtest9)")
        self._bw9_btn.clicked.connect(lambda: self._send_bwtest(9))
        bw_layout.addWidget(self._bw9_btn)

        bw_layout.addStretch()
        button_layout.addLayout(bw_layout)

        # Row 2: Identification
        ident_layout = QHBoxLayout()
        ident_layout.addWidget(QLabel("Identification:"))

        self._bw3_btn = QPushButton("Rs/Ld/Lq (bwtest3)")
        self._bw3_btn.clicked.connect(lambda: self._send_bwtest(3))
        ident_layout.addWidget(self._bw3_btn)

        self._bw4_btn = QPushButton("Flux ψ_f (bwtest4)")
        self._bw4_btn.clicked.connect(lambda: self._send_bwtest(4))
        ident_layout.addWidget(self._bw4_btn)

        self._bw5_btn = QPushButton("Inertia J (bwtest5)")
        self._bw5_btn.clicked.connect(lambda: self._send_bwtest(5))
        ident_layout.addWidget(self._bw5_btn)

        ident_layout.addStretch()
        button_layout.addLayout(ident_layout)

        # Row 3: AutoTune
        tune_layout = QHBoxLayout()
        tune_layout.addWidget(QLabel("AutoTune:"))

        self._bw6_btn = QPushButton("Current PI (bwtest6)")
        self._bw6_btn.clicked.connect(lambda: self._send_bwtest(6))
        tune_layout.addWidget(self._bw6_btn)

        self._bw7_btn = QPushButton("Speed PI (bwtest7)")
        self._bw7_btn.clicked.connect(lambda: self._send_bwtest(7))
        tune_layout.addWidget(self._bw7_btn)

        self._bw8_btn = QPushButton("Position PI (bwtest8)")
        self._bw8_btn.clicked.connect(lambda: self._send_bwtest(8))
        tune_layout.addWidget(self._bw8_btn)

        tune_layout.addStretch()
        button_layout.addLayout(tune_layout)

        # Row 4: One-click sequence
        seq_layout = QHBoxLayout()
        self._seq_btn = QPushButton("Run Full Sequence (3→4→5→6→7→8)")
        self._seq_btn.clicked.connect(self._run_sequence)
        seq_layout.addWidget(self._seq_btn)
        seq_layout.addStretch()
        button_layout.addLayout(seq_layout)

        layout.addWidget(button_group)

        # Bottom: Splitter (Bode plot | Text results)
        splitter = QSplitter(Qt.Horizontal)

        # Left: Bode plot
        self._bode_widget = BodePlotWidget()
        splitter.addWidget(self._bode_widget)

        # Right: Text results
        self._result_text = QTextEdit()
        self._result_text.setReadOnly(True)
        self._result_text.setMaximumWidth(400)
        splitter.addWidget(self._result_text)

        splitter.setSizes([600, 400])
        layout.addWidget(splitter)

        # State for parsing multi-line results
        self._collecting = False
        self._current_test = None
        self._freq_list = []
        self._gain_list = []
        self._phase_list = []

    def _send_bwtest(self, test_id: int):
        """Send bwtest command."""
        from core.protocol import build_bwtest
        cmd = build_bwtest(test_id)
        self.sig_command.emit(cmd)
        self._result_text.append(f"\n>>> Sent: {cmd}")

        # Reset collection state
        self._collecting = False
        self._current_test = test_id
        self._freq_list.clear()
        self._gain_list.clear()
        self._phase_list.clear()

    def _run_sequence(self):
        """Run full identification and autoTune sequence."""
        self._result_text.append("\n>>> Starting full sequence: bwtest3→4→5→6→7→8")
        self._result_text.append(">>> Please wait, this will take several minutes...")
        # Note: This is a simplified version. Full implementation would need
        # to wait for each test to complete before starting the next.
        # For now, just send them in sequence (firmware will queue).
        for test_id in [3, 4, 5, 6, 7, 8]:
            self._send_bwtest(test_id)

    def process_line(self, line: str):
        """Process a line from serial output to extract bwtest results.

        Args:
            line: Text line from serial port
        """
        # Detect start of frequency sweep data
        if "Freq(Hz)" in line and "Gain(dB)" in line and "Phase(deg)" in line:
            self._collecting = True
            self._freq_list.clear()
            self._gain_list.clear()
            self._phase_list.clear()
            self._result_text.append(line)
            return

        # Collect data lines
        if self._collecting:
            # Try to parse: "freq\t\tgain\t\tphase"
            parts = line.split()
            if len(parts) >= 3:
                try:
                    freq = float(parts[0])
                    gain = float(parts[1])
                    phase = float(parts[2])
                    self._freq_list.append(freq)
                    self._gain_list.append(gain)
                    self._phase_list.append(phase)
                    self._result_text.append(line)
                    return
                except ValueError:
                    pass

            # End of data (summary line or empty)
            if len(self._freq_list) > 0:
                self._collecting = False
                self._update_bode_plot()

        # Show all lines in text widget
        if "bwtest" in line.lower() or "bandwidth" in line.lower() or \
           "dB" in line or "Hz" in line or "autotune" in line.lower():
            self._result_text.append(line)

    def _update_bode_plot(self):
        """Update Bode plot with collected data."""
        if len(self._freq_list) < 2:
            return

        freq = np.array(self._freq_list)
        gain = np.array(self._gain_list)
        phase = np.array(self._phase_list)

        # Set title based on current test
        titles = {
            1: "Current Loop Bandwidth Test",
            2: "Speed Loop Bandwidth Test",
            9: "Position Loop Bandwidth Test",
        }
        title = titles.get(self._current_test, "Bandwidth Test")
        self._bode_widget.set_title(title)

        self._bode_widget.set_data(freq, gain, phase)
