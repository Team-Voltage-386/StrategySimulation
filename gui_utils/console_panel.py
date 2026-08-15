"""
Scrolling log/action console -- renders common_sim.match.events.MatchEvent
entries as text lines. Game-agnostic: it only knows the generic event
`kind`/`data` shape Match already logs (intake/deposit/score/phase_change/
match_end), not anything REEFSCAPE-specific, so it works unmodified for
any game_specific package the same way FieldCanvas does.
"""
from __future__ import annotations

from pyqtgraph.Qt import QtWidgets

from gui_utils import theme

__all__ = ["ConsolePanel"]

_KIND_COLOR = {
    "score": theme.ACCENT_AMBER,
    "deposit": theme.ACCENT_CYAN,
    "intake": theme.ACCENT_CYAN,
    "phase_change": theme.TEXT_PRIMARY,
    "match_end": theme.ACCENT_RED,
}
_MAX_LINES = 500  # caps memory for a long-running/unattended sim


class ConsolePanel(QtWidgets.QWidget):
    """Read-only scrolling text log. Callers append new MatchEvents each
    tick via append_events(); reset() clears it for a fresh match."""

    def __init__(self, parent=None):
        super().__init__(parent)
        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(0, 4, 0, 0)

        title = QtWidgets.QLabel("CONSOLE")
        title.setFont(theme.technical_font(11, bold=True))
        title.setStyleSheet(f"color: {theme.ACCENT_CYAN};")
        layout.addWidget(title)

        self.text = QtWidgets.QPlainTextEdit()
        self.text.setReadOnly(True)
        self.text.setMaximumBlockCount(_MAX_LINES)
        self.text.setFont(theme.technical_font(9))
        self.text.setStyleSheet(
            f"background-color: {theme.BG_PANEL}; color: {theme.TEXT_PRIMARY}; border: 1px solid {theme.BORDER};"
        )
        layout.addWidget(self.text, stretch=1)

    def reset(self) -> None:
        self.text.clear()

    def append_events(self, events) -> None:
        for event in events:
            self._append_line(event)

    def _append_line(self, event) -> None:
        color = _KIND_COLOR.get(event.kind, theme.TEXT_PRIMARY)
        text = f'<span style="color:{theme.TEXT_DIM};">[{event.timestamp:6.1f}s]</span> ' \
               f'<span style="color:{color};">{_format_event(event)}</span>'
        self.text.appendHtml(text)


def _format_event(event) -> str:
    data = event.data
    if event.kind == "intake":
        return f"INTAKE  {data.get('alliance', '?').upper():5} {data.get('piece_type', '?')}"
    if event.kind == "deposit":
        return f"DEPOSIT {data.get('alliance', '?').upper():5} {data.get('piece_type', '?')}"
    if event.kind == "score":
        return (
            f"SCORE   {data.get('alliance', '?').upper():5} {data.get('action', '?'):8} "
            f"+{data.get('points', 0):.0f} ({data.get('piece_type', '?')})"
        )
    if event.kind == "phase_change":
        return f"PHASE -> {data.get('phase', '?').upper()}"
    if event.kind == "match_end":
        return "MATCH END"
    return f"{event.kind.upper()} {data}"
