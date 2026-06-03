"""Quick test for CAN-FD connection in GUI context."""
import sys
from PySide6.QtWidgets import QApplication
from PySide6.QtCore import QTimer
from core.can_worker import CanWorker


def test_can():
    app = QApplication.instance() or QApplication(sys.argv)

    worker = CanWorker()

    def on_connected(ok):
        print(f"Connected: {ok}")
        if ok:
            print("Connection successful!")
            QTimer.singleShot(1000, app.quit)
        else:
            print("Connection failed")
            app.quit()

    def on_error(msg):
        print(f"Error: {msg}")

    def on_line(line):
        print(f"RX: {line}")

    worker.sig_connected.connect(on_connected)
    worker.sig_error.connect(on_error)
    worker.sig_line_received.connect(on_line)

    print("Attempting to connect to CAN ch=0...")
    worker.connect_port(channel=0, abit=1_000_000, dbit=5_000_000)

    app.exec()

    worker.disconnect_port()
    print("Disconnected")


if __name__ == "__main__":
    test_can()
