"""
Standalone side-view (X-Z plane) viewer for the mechanism-orchestration
sandbox -- a separate Java/WPILib project (MechanismOrchestrationSandbox,
a sibling repo) that prototypes elevator/arm/wrist coordination logic in
its own sim. This window is a pure NT4 client: it never touches physics
or timing itself, it just polls the /Mechanism/* topics that project's
MechanismPublisher publishes and draws whatever comes back. Deliberately
a separate app from the strategy sim (run_reefscape.py etc.) -- the two
are related only in that this sandbox's cycle-time findings eventually get
hand-entered as parameters over there.

Run: `python -m apps.run_mechanism_view [--server 127.0.0.1]`, with the
Java project's `./gradlew simulateJava` already running (or a real robot
at that address).
"""
from __future__ import annotations

import argparse
import sys

from pyqtgraph.Qt import QtCore, QtWidgets

from common_sim.telemetry.nt4_client import NT4MechanismClient
from gui_utils import theme
from gui_utils.mechanism_canvas import MechanismCanvas


class MechanismViewWindow(QtWidgets.QMainWindow):
    POLL_HZ = 50

    def __init__(self, server: str):
        super().__init__()
        self.setWindowTitle(f"sparky-sim -- mechanism view [{server}]")

        self.client = NT4MechanismClient(server=server)
        self.canvas = MechanismCanvas()
        self.setCentralWidget(self.canvas)
        self.resize(900, 600)

        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self._tick)
        self.timer.start(int(1000 / self.POLL_HZ))

    def _tick(self) -> None:
        self.canvas.snapshot = self.client.poll()
        self.canvas.update()

    def closeEvent(self, event) -> None:
        self.client.close()
        super().closeEvent(event)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--server", default="127.0.0.1",
                        help="NT4 server address to connect to (default: 127.0.0.1, i.e. a local sim)")
    args = parser.parse_args()

    app = QtWidgets.QApplication(sys.argv)
    theme.apply_app_theme(app)
    window = MechanismViewWindow(args.server)
    window.show()
    app.exec_()


if __name__ == "__main__":
    main()
