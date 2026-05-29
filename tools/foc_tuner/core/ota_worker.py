"""OTA firmware uploader running in a background QThread.

Drives the begin → data → end sequence over the serial worker.
ACK lines from the MCU come in via on_line() (called from the main thread's
serial line handler) and land in a thread-safe queue that the worker drains.

Usage:
    uploader = OTAUploader(serial_worker, bin_data, version=1)
    serial_worker.sig_line_received.connect(uploader.on_line)
    uploader.sig_progress.connect(progress_bar.setValue)  # 0..100
    uploader.sig_status.connect(log.append)
    uploader.sig_done.connect(handle_done)  # bool ok, str msg
    uploader.start()
"""

import queue
import time
import zlib

from PySide6.QtCore import QThread, Signal, Slot

from core.protocol import (
    build_ota_abort,
    build_ota_begin,
    build_ota_data_frame,
    build_ota_end,
    get_ota_chunk_size,
)


class OTAUploader(QThread):
    """Background uploader. One instance per OTA session — discard after done."""

    # bytes_sent, total_bytes — emit at most once per chunk
    sig_progress = Signal(int, int)
    # human-readable status line (goes to log widget)
    sig_status = Signal(str)
    # success: bool, message: str
    sig_done = Signal(bool, str)

    # Per-chunk timing budget. The MCU writes Flash inside the data path
    # (~250us per 32B page on H7), so a 256B chunk takes ~2ms of Flash work
    # plus serial latency. 1 second is a generous ceiling that still keeps
    # a stuck session from hanging the user.
    _ACK_TIMEOUT_S = 1.0
    _MAX_RETRIES = 3
    _BEGIN_TIMEOUT_S = 10.0  # MCU erases 7 sectors (~1.75s); generous margin
    _END_TIMEOUT_S = 10.0    # MCU CRCs the whole image (~5MB/s memcpy)

    def __init__(self, serial_worker, bin_data: bytes, version: int = 1, parent=None):
        super().__init__(parent)
        self._sw = serial_worker
        self._bin = bin_data
        self._version = version
        self._crc32 = zlib.crc32(bin_data) & 0xFFFFFFFF
        self._ack_q: "queue.Queue[str]" = queue.Queue()
        self._cancel = False

    # Called from the main thread (serial worker's line signal).
    # Filters and forwards relevant lines into the worker queue.
    @Slot(str)
    def on_line(self, line: str):
        for prefix in ("OTA_READY", "OTA_ACK", "OTA_NAK",
                       "OTA_DONE", "OTA_FAIL", "OTA_ERR"):
            if line.startswith(prefix):
                self._ack_q.put(line)
                return

    def cancel(self):
        """Request cancellation; the worker checks this between chunks."""
        self._cancel = True

    def _wait_line(self, prefix: str, timeout_s: float) -> str | None:
        """Block until a line starting with `prefix` arrives, or timeout."""
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            remaining = deadline - time.monotonic()
            try:
                line = self._ack_q.get(timeout=min(remaining, 0.1))
            except queue.Empty:
                if self._cancel:
                    return None
                continue
            if line.startswith(prefix):
                return line
            # Other ACK-class line (e.g., NAK while waiting for ACK) —
            # let the caller see it via the queue on the next call. But
            # since we only call _wait_line for one prefix at a time, we
            # drop unrelated lines silently.
        return None

    def _wait_ack_or_nak(self, expected_seq: int, timeout_s: float) -> tuple[str, str]:
        """Wait for OTA_ACK <seq> or OTA_NAK <seq> ... line.

        Returns:
            ('ack', '') on success, ('nak', reason) on NAK,
            ('timeout', '') if no response in time.
        """
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            remaining = deadline - time.monotonic()
            try:
                line = self._ack_q.get(timeout=min(remaining, 0.1))
            except queue.Empty:
                if self._cancel:
                    return ('cancel', '')
                continue
            parts = line.split()
            if len(parts) >= 2 and parts[0] == "OTA_ACK":
                try:
                    if int(parts[1]) == expected_seq:
                        return ('ack', '')
                except ValueError:
                    pass
            elif len(parts) >= 2 and parts[0] == "OTA_NAK":
                reason = " ".join(parts[2:]) if len(parts) > 2 else "unknown"
                return ('nak', reason)
            elif line.startswith(("OTA_FAIL", "OTA_ERR")):
                return ('nak', line)
        return ('timeout', '')

    def run(self):
        total = len(self._bin)
        chunk = get_ota_chunk_size()

        # ---- 1. Begin ----
        self.sig_status.emit(
            f"OTA: begin size={total} crc32=0x{self._crc32:08X} ver={self._version}"
        )
        self._sw.send(build_ota_begin(total, self._crc32, str(self._version)))

        ready = self._wait_line("OTA_READY", self._BEGIN_TIMEOUT_S)
        if ready is None:
            self.sig_done.emit(False, "Timeout waiting for OTA_READY")
            return
        self.sig_status.emit(f"<- {ready}")

        # ---- 2. Send chunks ----
        sent = 0
        seq = 0
        while sent < total:
            if self._cancel:
                self.sig_status.emit("OTA cancelled by user")
                self._sw.send(build_ota_abort())
                self.sig_done.emit(False, "Cancelled")
                return

            payload = self._bin[sent:sent + chunk]
            frame = build_ota_data_frame(seq, payload)

            for attempt in range(self._MAX_RETRIES):
                self._sw.send_bytes(frame)
                kind, reason = self._wait_ack_or_nak(seq, self._ACK_TIMEOUT_S)
                if kind == 'ack':
                    break
                if kind == 'cancel':
                    self.sig_status.emit("OTA cancelled by user")
                    self._sw.send(build_ota_abort())
                    self.sig_done.emit(False, "Cancelled")
                    return
                self.sig_status.emit(
                    f"seq={seq} {kind}: {reason} (retry {attempt + 1}/{self._MAX_RETRIES})"
                )
            else:
                self._sw.send(build_ota_abort())
                self.sig_done.emit(False, f"Chunk {seq} failed after retries")
                return

            sent += len(payload)
            seq = (seq + 1) & 0xFFFF
            self.sig_progress.emit(sent, total)

        # ---- 3. End ----
        self.sig_status.emit(f"OTA: all chunks sent, finalizing...")
        self._sw.send(build_ota_end())

        deadline = time.monotonic() + self._END_TIMEOUT_S
        while time.monotonic() < deadline:
            remaining = deadline - time.monotonic()
            try:
                line = self._ack_q.get(timeout=min(remaining, 0.1))
            except queue.Empty:
                continue
            if line.startswith("OTA_DONE"):
                self.sig_status.emit(f"<- {line}")
                self.sig_done.emit(True, line)
                return
            if line.startswith(("OTA_FAIL", "OTA_ERR")):
                self.sig_status.emit(f"<- {line}")
                self.sig_done.emit(False, line)
                return

        self.sig_done.emit(False, "Timeout waiting for OTA_DONE")
