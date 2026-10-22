"""调试工作台主题。"""

APP_STYLE = r"""
QWidget { background: #f7f9fc; color: #1f2937; font-family: "Microsoft YaHei UI"; font-size: 12px; }
QMainWindow, QSplitter { background: #f7f9fc; }
QFrame#statusPanel, QFrame#controlPanel { background: #ffffff; border: 1px solid #d7dee8; }
QFrame#healthItem { background: #ffffff; border: 1px solid #d7dee8; }
QLabel#appTitle { font-size: 20px; font-weight: 700; color: #172033; }
QLabel#appSubtitle, QLabel#mutedLabel { color: #64748b; }
QLabel#sectionTitle { font-size: 11px; font-weight: 700; color: #0369a1; }
QLabel#metricValue { color: #172033; font-family: Consolas; font-size: 13px; font-weight: 600; }
QLabel#statusBadge { color: #15803d; background: #ecfdf5; border: 1px solid #86efac; padding: 7px; }
QLabel#statusBadge[fault="true"] { color: #b91c1c; background: #fef2f2; border-color: #fca5a5; }
QLabel#healthValue { color: #b91c1c; font-family: Consolas; font-weight: 700; }
QLabel#healthValue[online="true"] { color: #15803d; }
QGroupBox { border: 1px solid #d7dee8; margin-top: 9px; padding-top: 9px; color: #475569; font-size: 10px; font-weight: 700; }
QGroupBox::title { subcontrol-origin: margin; left: 8px; padding: 0 4px; }
QPushButton { background: #ffffff; border: 1px solid #cbd5e1; padding: 7px 10px; color: #1f2937; }
QPushButton:hover { background: #eff6ff; border-color: #38bdf8; }
QPushButton:pressed, QPushButton:checked { background: #e0f2fe; border-color: #0284c7; }
QPushButton#primaryButton { background: #0369a1; border-color: #0ea5e9; color: #ffffff; font-weight: 700; }
QPushButton#dangerButton { background: #b91c1c; border-color: #ef4444; color: #ffffff; }
QPushButton#enableButton { min-height: 32px; font-weight: 700; }
QPushButton#enableButton[armed="true"] { background: #15803d; border-color: #22c55e; color: #ffffff; }
QPushButton#driveButton { min-width: 88px; min-height: 30px; font-weight: 600; }
QPushButton#commandButton { font-family: Consolas; font-weight: 700; padding: 7px 5px; }
QComboBox, QDoubleSpinBox { background: #ffffff; border: 1px solid #cbd5e1; padding: 5px; min-height: 20px; }
QComboBox:focus, QDoubleSpinBox:focus { border-color: #38bdf8; }
QTabWidget::pane { border: 1px solid #d7dee8; }
QTabBar::tab { background: #ffffff; padding: 8px 18px; border: 1px solid #d7dee8; }
QTabBar::tab:selected { background: #eff6ff; color: #0369a1; }
QStatusBar { background: #ffffff; border-top: 1px solid #d7dee8; color: #64748b; }
QToolBar { background: #ffffff; border-bottom: 1px solid #d7dee8; spacing: 6px; padding: 6px; }
QToolButton { background: #ffffff; border: 1px solid #cbd5e1; padding: 6px 10px; }
QToolButton:hover { border-color: #38bdf8; }
QScrollBar:vertical { background: #eef2f7; width: 10px; }
QScrollBar::handle:vertical { background: #cbd5e1; min-height: 24px; }
"""
