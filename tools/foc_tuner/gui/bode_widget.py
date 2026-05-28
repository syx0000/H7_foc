"""Bode plot widget for displaying frequency response data.

Displays magnitude and phase plots from bandwidth test results.
"""

import numpy as np
import pyqtgraph as pg
from PySide6.QtWidgets import QWidget, QVBoxLayout, QHBoxLayout, QLabel
from PySide6.QtCore import Qt


class BodePlotWidget(QWidget):
    """Bode plot with magnitude and phase subplots."""

    def __init__(self, parent=None):
        super().__init__(parent)

        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)

        # Title
        self._title_label = QLabel("Bode Plot")
        self._title_label.setAlignment(Qt.AlignCenter)
        layout.addWidget(self._title_label)

        # Magnitude plot
        self._mag_plot = pg.PlotWidget()
        self._mag_plot.setBackground('w')
        self._mag_plot.showGrid(x=True, y=True, alpha=0.3)
        self._mag_plot.setLabel('left', 'Magnitude', units='dB')
        self._mag_plot.setLabel('bottom', 'Frequency', units='Hz')
        self._mag_plot.setLogMode(x=True, y=False)
        self._mag_curve = self._mag_plot.plot([], [], pen=pg.mkPen('b', width=2), symbol='o', symbolSize=5)

        # Add -3dB reference line
        self._mag_ref_line = pg.InfiniteLine(pos=-3, angle=0, pen=pg.mkPen('r', style=Qt.DashLine))
        self._mag_plot.addItem(self._mag_ref_line)

        layout.addWidget(self._mag_plot)

        # Phase plot
        self._phase_plot = pg.PlotWidget()
        self._phase_plot.setBackground('w')
        self._phase_plot.showGrid(x=True, y=True, alpha=0.3)
        self._phase_plot.setLabel('left', 'Phase', units='deg')
        self._phase_plot.setLabel('bottom', 'Frequency', units='Hz')
        self._phase_plot.setLogMode(x=True, y=False)
        self._phase_curve = self._phase_plot.plot([], [], pen=pg.mkPen('g', width=2), symbol='o', symbolSize=5)

        # Add -180deg reference line
        self._phase_ref_line = pg.InfiniteLine(pos=-180, angle=0, pen=pg.mkPen('r', style=Qt.DashLine))
        self._phase_plot.addItem(self._phase_ref_line)

        layout.addWidget(self._phase_plot)

        # Info label
        self._info_label = QLabel("")
        self._info_label.setAlignment(Qt.AlignLeft)
        layout.addWidget(self._info_label)

    def set_title(self, title: str):
        """Set plot title."""
        self._title_label.setText(title)

    def set_data(self, freq: np.ndarray, gain_db: np.ndarray, phase_deg: np.ndarray):
        """Update Bode plot with new data.

        Args:
            freq: Frequency array (Hz)
            gain_db: Magnitude array (dB)
            phase_deg: Phase array (degrees)
        """
        if len(freq) == 0:
            return

        self._mag_curve.setData(freq, gain_db)
        self._phase_curve.setData(freq, phase_deg)

        # Auto-range
        self._mag_plot.enableAutoRange()
        self._phase_plot.enableAutoRange()

        # Calculate metrics
        self._update_info(freq, gain_db, phase_deg)

    def _update_info(self, freq: np.ndarray, gain_db: np.ndarray, phase_deg: np.ndarray):
        """Calculate and display bandwidth metrics."""
        if len(freq) < 2:
            return

        # Find peak gain
        peak_idx = np.argmax(gain_db)
        peak_gain = gain_db[peak_idx]
        peak_freq = freq[peak_idx]

        # Find -3dB bandwidth (from peak or DC)
        ref_gain = peak_gain if peak_gain > 0 else gain_db[0]
        target_gain = ref_gain - 3.0

        bw_freq = None
        for i in range(len(gain_db)):
            if gain_db[i] < target_gain:
                if i > 0:
                    # Linear interpolation
                    f1, g1 = freq[i-1], gain_db[i-1]
                    f2, g2 = freq[i], gain_db[i]
                    bw_freq = f1 + (target_gain - g1) * (f2 - f1) / (g2 - g1)
                else:
                    bw_freq = freq[i]
                break

        # Find 0dB crossover
        crossover_freq = None
        for i in range(len(gain_db) - 1):
            if gain_db[i] >= 0 and gain_db[i+1] < 0:
                f1, g1 = freq[i], gain_db[i]
                f2, g2 = freq[i+1], gain_db[i+1]
                crossover_freq = f1 + (0 - g1) * (f2 - f1) / (g2 - g1)
                break

        # Phase margin at crossover
        pm = None
        if crossover_freq:
            # Find phase at crossover frequency
            idx = np.searchsorted(freq, crossover_freq)
            if idx < len(phase_deg):
                phase_at_cross = phase_deg[idx]
                pm = 180 + phase_at_cross  # PM = 180 + phase (when phase is negative)

        # Build info string
        info_parts = []
        if peak_gain > 0.5:
            info_parts.append(f"Peak: {peak_gain:.2f} dB @ {peak_freq:.1f} Hz")
        if bw_freq:
            info_parts.append(f"-3dB BW: {bw_freq:.1f} Hz")
        if crossover_freq:
            info_parts.append(f"0dB crossover: {crossover_freq:.1f} Hz")
        if pm is not None:
            info_parts.append(f"PM: {pm:.0f}°")

        self._info_label.setText("  |  ".join(info_parts))

    def clear(self):
        """Clear plot data."""
        self._mag_curve.setData([], [])
        self._phase_curve.setData([], [])
        self._info_label.setText("")
