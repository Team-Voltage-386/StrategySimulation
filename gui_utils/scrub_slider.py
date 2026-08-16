"""A horizontal QSlider where a left-click anywhere on the groove jumps
straight to that position, like a media-player seek bar.

Plain QSlider only does this reliably in click-to-position mode; on
native Windows styling a left click that doesn't land exactly on the
handle just page-steps by a small fixed amount instead of jumping --
on a slider covering a multi-minute match at a fine time resolution
that page-step is visually invisible, which reads as "the slider
doesn't respond to clicks" even though it's fully enabled and wired up
correctly.
"""
from __future__ import annotations

from pyqtgraph.Qt import QtCore, QtWidgets


class ScrubSlider(QtWidgets.QSlider):
    def mousePressEvent(self, event) -> None:
        if event.button() == QtCore.Qt.MouseButton.LeftButton:
            x = event.position().x() if hasattr(event, "position") else event.pos().x()
            fraction = min(max(x / max(1, self.width()), 0.0), 1.0)
            self.setValue(round(self.minimum() + fraction * (self.maximum() - self.minimum())))
            event.accept()
        # Let QSlider's own handling still run -- with the value already
        # warped under the cursor, its hit-test now finds the handle
        # right where the click landed, so its native drag-tracking
        # picks up cleanly for the rest of the gesture.
        super().mousePressEvent(event)
