"""
Simple label-per-line telemetry/statistics readout, styled with the
sci-fi theme's technical (monospace) font so numeric columns line up.
Game-agnostic -- callers pass whatever (label, value) pairs they want
displayed each refresh; this widget has no notion of Match/Robot.
"""
from __future__ import annotations

from pyqtgraph.Qt import QtCore, QtWidgets

from gui_utils import theme

__all__ = ["TelemetryPanel"]


class TelemetryPanel(QtWidgets.QWidget):
    def __init__(self, title: str = "TELEMETRY", parent=None):
        super().__init__(parent)
        self.setMinimumWidth(220)

        layout = QtWidgets.QVBoxLayout(self)
        self.title_label = QtWidgets.QLabel(title)
        self.title_label.setStyleSheet(f"color: {theme.ACCENT_CYAN}; font-weight: bold;")
        self.title_label.setFont(theme.technical_font(12, bold=True))
        layout.addWidget(self.title_label)

        self.body_label = QtWidgets.QLabel("")
        self.body_label.setFont(theme.technical_font(10))
        self.body_label.setStyleSheet(f"color: {theme.TEXT_PRIMARY};")
        self.body_label.setWordWrap(True)
        self.body_label.setTextInteractionFlags(QtCore.Qt.TextSelectableByMouse)
        layout.addWidget(self.body_label)
        layout.addStretch(1)

    def set_lines(self, pairs: list[tuple[str, str]]) -> None:
        self.body_label.setText("\n".join(f"{label}: {value}" for label, value in pairs))
