#!/usr/bin/env python
"""
TurtleRabbit SSL Command Center — launch the full UI.

Usage:
    python ui_main.py
"""

import sys
import multiprocessing

from PySide6.QtWidgets import QApplication
from PySide6.QtCore import Qt
from PySide6.QtGui import QFont

from research_sdk.ui.main_window import MainWindow


def main():
    multiprocessing.freeze_support()

    app = QApplication(sys.argv)
    app.setApplicationName("TurtleRabbit")
    app.setOrganizationName("WSU")

    app.setFont(QFont("Segoe UI", 11))

    window = MainWindow()
    window.show()

    sys.exit(app.exec())


if __name__ == "__main__":
    main()
