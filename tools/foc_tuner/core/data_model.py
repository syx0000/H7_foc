"""Ring-buffer data model for real-time waveform display.

Stores time-stamped data in numpy ring buffers, one per channel.
Emits Qt signals when new data arrives for GUI update.
"""

import numpy as np
from PySide6.QtCore import QObject, Signal


class ChannelBuffer:
    """Fixed-size ring buffer for one signal channel."""

    def __init__(self, max_len: int = 10000):
        self._max_len = max_len
        self._t = np.zeros(max_len, dtype=np.float64)
        self._y = np.zeros(max_len, dtype=np.float64)
        self._head = 0
        self._count = 0

    def append(self, t: float, y: float):
        """Append a single data point.

        Args:
            t: Timestamp (perf_counter seconds)
            y: Value in SI units
        """
        self._t[self._head] = t
        self._y[self._head] = y
        self._head = (self._head + 1) % self._max_len
        if self._count < self._max_len:
            self._count += 1

    @property
    def count(self) -> int:
        return self._count

    def get_data(self) -> tuple[np.ndarray, np.ndarray]:
        """Get time and value arrays in chronological order.

        Returns:
            Tuple of (time_array, value_array), both length == count
        """
        if self._count < self._max_len:
            return self._t[:self._count].copy(), self._y[:self._count].copy()
        # Ring buffer is full, need to unwrap
        idx = np.arange(self._head, self._head + self._max_len) % self._max_len
        return self._t[idx].copy(), self._y[idx].copy()

    def clear(self):
        """Clear all data."""
        self._head = 0
        self._count = 0


class DataModel(QObject):
    """Central data store for all waveform channels.

    Manages ring buffers per channel and emits signals on new data.
    """

    sig_new_data = Signal(int)  # logid that just received new data

    def __init__(self, buffer_size: int = 10000, parent=None):
        super().__init__(parent)
        self._buffer_size = buffer_size
        self._channels: dict[str, ChannelBuffer] = {}

    def append(self, logid: int, timestamp: float, fields: dict[str, float]):
        """Append a parsed frame's data to the appropriate channel buffers.

        Args:
            logid: Log type identifier
            timestamp: Time of receipt (perf_counter)
            fields: Dict of channel_name -> value
        """
        for name, value in fields.items():
            if name not in self._channels:
                self._channels[name] = ChannelBuffer(self._buffer_size)
            self._channels[name].append(timestamp, value)
        self.sig_new_data.emit(logid)

    def get_channel(self, name: str) -> tuple[np.ndarray, np.ndarray]:
        """Get data for a specific channel.

        Args:
            name: Channel name (e.g., "I_q", "V_d")

        Returns:
            Tuple of (time_array, value_array). Empty arrays if channel doesn't exist.
        """
        if name in self._channels:
            return self._channels[name].get_data()
        return np.array([]), np.array([])

    def get_channel_names(self) -> list[str]:
        """Get list of all channels that have data.

        Returns:
            List of channel names
        """
        return [name for name, buf in self._channels.items() if buf.count > 0]

    def clear_all(self):
        """Clear all channel data."""
        for buf in self._channels.values():
            buf.clear()

    def clear_channel(self, name: str):
        """Clear data for a specific channel.

        Args:
            name: Channel name
        """
        if name in self._channels:
            self._channels[name].clear()
