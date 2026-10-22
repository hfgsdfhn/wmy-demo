"""Robocon Chassis Debug Assistant 程序入口。"""
from __future__ import annotations

import os
import sys
from pathlib import Path

os.environ.setdefault("QT_API", "pyqt5") 
from PyQt5 import QtCore, QtGui, QtWidgets

ROOT = Path(__file__).resolve().parent
if __package__:
    from .ui.main_window import MainWindow
    from .ui.theme import APP_STYLE
else:
    sys.path.insert(0, str(ROOT.parent))
    from Robocon_Chassis_Debugger.ui.main_window import MainWindow
    from Robocon_Chassis_Debugger.ui.theme import APP_STYLE


def main() -> int:
    app = QtWidgets.QApplication(sys.argv); app.setApplicationName("Robocon Chassis Debug Assistant"); app.setOrganizationName("Robocon Team"); app.setStyle("Fusion"); app.setStyleSheet(APP_STYLE)
    font = QtGui.QFont("Microsoft YaHei UI", 9); app.setFont(font)
    window = MainWindow(ROOT / "config" / "robot_config.json"); window.show()
    return app.exec_()

if __name__ == "__main__":
    raise SystemExit(main())
