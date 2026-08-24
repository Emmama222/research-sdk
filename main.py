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


from research_sdk.ui.app import ResearchConsole


def main()->int:
    multiprocessing.freeze_support()

    application = QApplication.instance() or QApplication(sys.argv)
    application.setStyle("Fusion")
    window = ResearchConsole()
    window.show()
    return application.exec()


if __name__ == "__main__":
    raise SystemExit(main())
