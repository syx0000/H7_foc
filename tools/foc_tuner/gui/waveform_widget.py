"""Waveform widget for real-time scrolling plots.

Uses pyqtgraph for high-performance time-series display.
Supports dual-cursor measurement (click to place A/B, shows values + Δt).
"""

import numpy as np
import pyqtgraph as pg
from PySide6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QPushButton, QCheckBox,
    QLabel, QDoubleSpinBox, QFrame
)
from PySide6.QtCore import QTimer, Qt
from PySide6.QtGui import QFont


class WaveformWidget(QWidget):
    """Real-time scrolling waveform display with multiple channels."""

    def __init__(self, data_model, parent=None):
        super().__init__(parent)
        self._data_model = data_model
        self._paused = False
        self._window_sec = 10.0  # Visible time window when follow X is on
        self._curves: dict[str, pg.PlotDataItem] = {}
        self._channel_checkboxes: dict[str, QCheckBox] = {}
        self._channel_colors = [
            (255, 0, 0),    # Red
            (0, 180, 0),    # Green
            (0, 0, 255),    # Blue
            (200, 150, 0),  # Orange
            (180, 0, 180),  # Magenta
            (0, 180, 180),  # Cyan
            (120, 60, 0),   # Brown
            (100, 100, 100),# Gray
        ]

        layout = QVBoxLayout(self)

        # Top control bar
        control_layout = QHBoxLayout()

        self._pause_btn = QPushButton("Pause")
        self._pause_btn.setCheckable(True)
        self._pause_btn.toggled.connect(self._on_pause_toggled)
        control_layout.addWidget(self._pause_btn)

        self._clear_btn = QPushButton("Clear")
        self._clear_btn.clicked.connect(self._on_clear_clicked)
        control_layout.addWidget(self._clear_btn)

        self._auto_y_cb = QCheckBox("Auto Y")
        self._auto_y_cb.setChecked(True)
        control_layout.addWidget(self._auto_y_cb)

        self._follow_x_cb = QCheckBox("Follow X")
        self._follow_x_cb.setChecked(True)
        self._follow_x_cb.toggled.connect(self._on_follow_x_toggled)
        control_layout.addWidget(self._follow_x_cb)

        control_layout.addWidget(QLabel("Window (s):"))
        self._window_spin = QDoubleSpinBox()
        self._window_spin.setRange(0.1, 600.0)
        self._window_spin.setValue(self._window_sec)
        self._window_spin.setDecimals(1)
        self._window_spin.setSingleStep(1.0)
        self._window_spin.valueChanged.connect(self._on_window_changed)
        control_layout.addWidget(self._window_spin)

        self._reset_view_btn = QPushButton("Reset View")
        self._reset_view_btn.clicked.connect(self._on_reset_view_clicked)
        control_layout.addWidget(self._reset_view_btn)

        control_layout.addStretch()
        layout.addLayout(control_layout)

        # Channel selection bar (dynamically populated by set_channels)
        self._channel_bar = QFrame()
        self._channel_bar_layout = QHBoxLayout(self._channel_bar)
        self._channel_bar_layout.setContentsMargins(2, 2, 2, 2)
        self._channel_bar_layout.addWidget(QLabel("Channels:"))
        self._channel_bar_layout.addStretch()
        layout.addWidget(self._channel_bar)

        # Plot widget
        self._plot_widget = pg.PlotWidget()
        self._plot_widget.setBackground('w')
        self._plot_widget.showGrid(x=True, y=True, alpha=0.3)
        self._plot_widget.setLabel('bottom', 'Time', units='s')
        self._plot_widget.addLegend()
        self._plot_widget.setMouseEnabled(x=True, y=True)
        layout.addWidget(self._plot_widget)

        # Cursor measurement (A/B vertical lines)
        self._cursor_state = 0  # 0=next click places A, 1=places B, 2=clears
        self._cursor_a = pg.InfiniteLine(
            pos=0, angle=90, movable=True,
            pen=pg.mkPen(color=(200, 180, 0), width=1.5, style=Qt.DashLine))
        self._cursor_b = pg.InfiniteLine(
            pos=0, angle=90, movable=True,
            pen=pg.mkPen(color=(0, 180, 200), width=1.5, style=Qt.DashLine))
        self._cursor_a.setVisible(False)
        self._cursor_b.setVisible(False)
        self._plot_widget.addItem(self._cursor_a)
        self._plot_widget.addItem(self._cursor_b)
        self._cursor_a.sigPositionChanged.connect(self._update_cursor_label)
        self._cursor_b.sigPositionChanged.connect(self._update_cursor_label)
        self._plot_widget.scene().sigMouseClicked.connect(self._on_plot_clicked)

        # Cursor info label
        self._cursor_label = QLabel("")
        self._cursor_label.setFont(QFont("Consolas", 9))
        self._cursor_label.setStyleSheet("QLabel { color: #333; }")
        layout.addWidget(self._cursor_label)

        # Refresh timer (30 Hz)
        self._timer = QTimer()
        self._timer.timeout.connect(self._refresh)
        self._timer.start(33)

    def set_channels(self, channel_names: list[str]):
        """Configure which channels to display.

        Creates one curve and one checkbox per channel. User can toggle
        individual channels on/off via the channel bar.

        Args:
            channel_names: List of channel names from parser
        """
        # Clear existing curves
        self._plot_widget.clear()
        self._curves.clear()

        # Clear existing checkboxes (keep "Channels:" label and trailing stretch)
        for cb in self._channel_checkboxes.values():
            self._channel_bar_layout.removeWidget(cb)
            cb.deleteLater()
        self._channel_checkboxes.clear()

        # Create new curves + checkboxes
        for i, name in enumerate(channel_names):
            color = self._channel_colors[i % len(self._channel_colors)]
            pen = pg.mkPen(color=color, width=2)
            curve = self._plot_widget.plot([], [], pen=pen, name=name)
            self._curves[name] = curve

            cb = QCheckBox(name)
            cb.setChecked(True)
            # Color the checkbox label to match curve
            cb.setStyleSheet(
                f"QCheckBox {{ color: rgb({color[0]},{color[1]},{color[2]}); "
                f"font-weight: bold; }}"
            )
            cb.toggled.connect(self._make_channel_toggle_handler(name))
            # Insert before the trailing stretch (which is at the end)
            self._channel_bar_layout.insertWidget(
                self._channel_bar_layout.count() - 1, cb
            )
            self._channel_checkboxes[name] = cb

    def _make_channel_toggle_handler(self, name: str):
        """Create a closure that shows/hides a specific channel curve."""
        def handler(checked: bool):
            curve = self._curves.get(name)
            if curve is not None:
                curve.setVisible(checked)
        return handler

    def _refresh(self):
        """Update plot with latest data (called by timer)."""
        if self._paused or not self._curves:
            return

        # Get current time reference (most recent data point)
        t_now = None
        for name in self._curves.keys():
            t, y = self._data_model.get_channel(name)
            if len(t) > 0:
                t_now = t[-1]
                break

        if t_now is None:
            return

        # Update visible curves only (skip hidden ones for performance)
        for name, curve in self._curves.items():
            if not curve.isVisible():
                continue
            t, y = self._data_model.get_channel(name)
            if len(t) > 0:
                t_rel = t - t_now
                curve.setData(t_rel, y)

        # X axis: follow latest data, or let user pan/zoom freely
        if self._follow_x_cb.isChecked():
            self._plot_widget.setXRange(-self._window_sec, 0, padding=0)

        # Y axis: auto-scale or user controlled
        if self._auto_y_cb.isChecked():
            self._plot_widget.enableAutoRange(axis='y')

    def _on_pause_toggled(self, checked: bool):
        """Handle pause button toggle."""
        self._paused = checked
        self._pause_btn.setText("Resume" if checked else "Pause")

    def _on_clear_clicked(self):
        """Clear all data."""
        self._data_model.clear_all()
        for curve in self._curves.values():
            curve.setData([], [])

    def _on_follow_x_toggled(self, checked: bool):
        """When follow is re-enabled, snap back to latest window."""
        if checked:
            self._plot_widget.setXRange(-self._window_sec, 0, padding=0)

    def _on_window_changed(self, value: float):
        """Update visible time window."""
        self._window_sec = value
        if self._follow_x_cb.isChecked():
            self._plot_widget.setXRange(-self._window_sec, 0, padding=0)

    def _on_reset_view_clicked(self):
        """Reset view to default: follow X with current window, auto Y."""
        self._follow_x_cb.setChecked(True)
        self._auto_y_cb.setChecked(True)
        self._plot_widget.setXRange(-self._window_sec, 0, padding=0)
        self._plot_widget.enableAutoRange(axis='y')

    # ─── Cursor measurement ───────────────────────────────────────────

    def _on_plot_clicked(self, event):
        """Handle mouse click on plot: place cursor A, B, or clear."""
        if event.button() == Qt.RightButton:
            self._clear_cursors()
            return
        if event.button() != Qt.LeftButton:
            return
        vb = self._plot_widget.plotItem.vb
        pos = vb.mapSceneToView(event.scenePos())
        t_click = pos.x()

        if self._cursor_state == 0:
            self._cursor_a.setValue(t_click)
            self._cursor_a.setVisible(True)
            self._cursor_b.setVisible(False)
            self._cursor_state = 1
        elif self._cursor_state == 1:
            self._cursor_b.setValue(t_click)
            self._cursor_b.setVisible(True)
            self._cursor_state = 2
        else:
            self._clear_cursors()
            self._cursor_a.setValue(t_click)
            self._cursor_a.setVisible(True)
            self._cursor_state = 1

        self._update_cursor_label()

    def _clear_cursors(self):
        """Hide both cursors and clear label."""
        self._cursor_a.setVisible(False)
        self._cursor_b.setVisible(False)
        self._cursor_state = 0
        self._cursor_label.setText("")

    def _update_cursor_label(self):
        """Update the cursor info label with values at cursor positions."""
        if not self._cursor_a.isVisible():
            self._cursor_label.setText("")
            return

        t_a = self._cursor_a.value()
        vals_a = self._get_values_at_time(t_a)
        parts = [f"A: t={t_a:.3f}s {self._fmt_vals(vals_a)}"]

        if self._cursor_b.isVisible():
            t_b = self._cursor_b.value()
            vals_b = self._get_values_at_time(t_b)
            dt = abs(t_b - t_a)
            freq = 1.0 / dt if dt > 1e-6 else 0
            parts.append(f"B: t={t_b:.3f}s {self._fmt_vals(vals_b)}")
            parts.append(f"Δt={dt:.4f}s ({freq:.1f}Hz)")

        self._cursor_label.setText("  |  ".join(parts))

    def _get_values_at_time(self, t_rel: float) -> dict:
        """Look up nearest sample value for each visible channel at t_rel."""
        result = {}
        t_now = None
        for name in self._curves.keys():
            t, y = self._data_model.get_channel(name)
            if len(t) > 0:
                t_now = t[-1]
                break
        if t_now is None:
            return result

        for name, curve in self._curves.items():
            if not curve.isVisible():
                continue
            t, y = self._data_model.get_channel(name)
            if len(t) == 0:
                continue
            t_rel_arr = t - t_now
            idx = np.searchsorted(t_rel_arr, t_rel, side='left')
            idx = np.clip(idx, 0, len(y) - 1)
            result[name] = y[idx]
        return result

    @staticmethod
    def _fmt_vals(vals: dict) -> str:
        """Format channel values dict into compact string."""
        if not vals:
            return "[]"
        items = [f"{k}={v:.1f}" for k, v in vals.items()]
        return "[" + ", ".join(items) + "]"
