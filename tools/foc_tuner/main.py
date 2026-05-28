"""FOC Motor Tuner - Main entry point.

Desktop application for tuning and debugging FOC motor controllers.
"""

import sys
from PySide6.QtWidgets import QApplication
from gui.main_window import MainWindow


def main():
    """Application entry point."""
    app = QApplication(sys.argv)
    app.setApplicationName("FOC Motor Tuner")
    app.setOrganizationName("FOC")

    window = MainWindow()
    window.show()

    sys.exit(app.exec())


if __name__ == "__main__":
    main()
