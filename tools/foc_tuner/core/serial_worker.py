"""Serial port worker thread for FOC motor controller.

Handles serial I/O in a separate QThread to avoid blocking the GUI.
Accumulates bytes and emits complete lines via signal.
"""

import serial
import serial.tools.list_ports
from PySide6.QtCore import QThread, Signal, QMutex


class SerialWorker(QThread):
    """QThread for serial port communication.

    Reads bytes in a loop, accumulates partial lines, and emits complete lines.
    Thread-safe send() method for transmitting commands.
    """

    sig_line_received = Signal(str)   # Complete line (stripped \\r\\n)
    sig_error = Signal(str)           # Error message
    sig_connected = Signal(bool)      # Connection state changed

    def __init__(self, parent=None):
        super().__init__(parent)
        self._port: serial.Serial | None = None
        self._running = False
        self._buf = bytearray()
        self._mutex = QMutex()

    def connect_port(self, port_name: str, baud: int = 921600):
        """Open serial port.

        Args:
            port_name: COM port name (e.g., "COM4")
            baud: Baud rate (default 921600)
        """
        try:
            self._port = serial.Serial(
                port=port_name,
                baudrate=baud,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0.001,  # Non-blocking read
            )
            self._buf.clear()
            self._running = True
            self.start()  # Start thread
            self.sig_connected.emit(True)
        except Exception as e:
            self.sig_error.emit(f"Failed to open {port_name}: {e}")
            self.sig_connected.emit(False)

    def disconnect_port(self):
        """Close serial port and stop thread."""
        self._running = False
        self.wait(1000)  # Wait up to 1s for thread to finish
        if self._port and self._port.is_open:
            self._port.close()
        self._port = None
        self.sig_connected.emit(False)

    def send(self, cmd: str):
        """Send command to serial port (thread-safe).

        Args:
            cmd: Command string (will append \\r\\n terminator)
        """
        self._mutex.lock()
        try:
            if self._port and self._port.is_open:
                # Firmware DMA IDLE detection requires line terminator
                if not cmd.endswith('\r\n'):
                    cmd = cmd + '\r\n'
                self._port.write(cmd.encode('ascii'))
                self._port.flush()
        except Exception as e:
            self.sig_error.emit(f"Send error: {e}")
        finally:
            self._mutex.unlock()

    def send_bytes(self, data: bytes):
        """Send raw bytes (thread-safe), no \\r\\n appended.

        Used for OTA binary data frames. Bypasses the ASCII conversion in
        send() so 0x00 / 0x0A / 0x0D pass through untouched.
        """
        self._mutex.lock()
        try:
            if self._port and self._port.is_open:
                self._port.write(data)
                self._port.flush()
        except Exception as e:
            self.sig_error.emit(f"Send error: {e}")
        finally:
            self._mutex.unlock()

    def run(self):
        """Main thread loop: read bytes, split on \\n, emit lines."""
        while self._running:
            try:
                if self._port and self._port.is_open and self._port.in_waiting:
                    chunk = self._port.read(self._port.in_waiting)
                    self._buf.extend(chunk)

                    # Buffer overflow protection
                    if len(self._buf) > 4096:
                        self.sig_error.emit("Buffer overflow, discarding old data")
                        self._buf = self._buf[-2048:]

                    # Split on \\n
                    while b'\n' in self._buf:
                        line_bytes, self._buf = self._buf.split(b'\n', 1)
                        text = line_bytes.rstrip(b'\r').decode('ascii', errors='replace')
                        if text:
                            self.sig_line_received.emit(text)
                else:
                    self.msleep(1)  # Yield when no data
            except Exception as e:
                self.sig_error.emit(f"Read error: {e}")
                self.msleep(10)

    @staticmethod
    def list_ports() -> list[str]:
        """Get list of available COM ports.

        Returns:
            List of port names (e.g., ["COM3", "COM4"])
        """
        ports = serial.tools.list_ports.comports()
        return [p.device for p in ports]
