"""Right-side info panel: shows the most recent telemetry message (at or
before the current playhead time) for whichever vehicle is the current
selection focus.

This widget holds no vehicle/message state of its own -- EarthPlot wires it
up via set_lookup() (a (vehicle_id, sim_time_s) -> message-dict-or-None
callback into its MessageStore) plus two signals: entity_selected (fires
when the selection focus vehicle changes) and playhead_time_changed (fires
when the displayed time moves by more than 1s, live or scrubbed). Every
update just re-runs the lookup and re-renders -- same "recompute from
current state on every relevant event" pattern the ephemeral overlays in
pyqtgraph_earth_demo.py already follow.

Not hardcoded to any particular app's message schema -- `fields` (passed at
construction) is a list of dicts choosing which keys of the looked-up
message dict to display, in what order, with what label and formatting,
same convention as message_table_model.py's display_columns:

    {"name": "altitude_ft", "label": "Altitude", "format": lambda v: f"{v:,.0f} ft"}

An optional "format" callable(value) -> str overrides the default str()
formatting for that field; fields whose "name" is missing from a given
message are skipped rather than raising. `title_format`, if given, is
callable(message_dict) -> str for the header line; it defaults to
"Vehicle {id_field}".
"""
from pyqtgraph.Qt import QtCore, QtWidgets

__all__ = ["InfoPanel"]


class InfoPanel(QtWidgets.QWidget):
    def __init__(self, fields, id_field="entity_id", title_format=None, parent=None):
        super().__init__(parent)
        self.fields = fields
        self.id_field = id_field
        self.title_format = title_format or (lambda msg: f"Vehicle {msg[self.id_field]}")
        self._lookup = None
        self._vehicle_id = None
        self._time_s = 0.0

        # Word-wrapped labels have a tiny sizeHint on their own (Qt will
        # happily shrink them to a sliver), so without this the dock this
        # widget lives in gets squeezed almost to nothing by the plot's
        # stretch priority.
        self.setMinimumWidth(220)

        layout = QtWidgets.QVBoxLayout(self)
        self.title_label = QtWidgets.QLabel("No selection")
        self.title_label.setStyleSheet("font-weight: bold; font-size: 13px;")
        layout.addWidget(self.title_label)

        self.fields_label = QtWidgets.QLabel("")
        self.fields_label.setWordWrap(True)
        self.fields_label.setTextInteractionFlags(QtCore.Qt.TextSelectableByMouse)
        layout.addWidget(self.fields_label)
        layout.addStretch(1)

        self._refresh()

    def set_lookup(self, fn):
        self._lookup = fn
        self._refresh()

    def on_entity_selected(self, vehicle_id):
        self._vehicle_id = vehicle_id
        self._refresh()

    def on_time_changed(self, time_s):
        self._time_s = time_s
        self._refresh()

    def _refresh(self):
        if self._vehicle_id is None or self._lookup is None:
            self.title_label.setText("No selection")
            self.fields_label.setText("Select a vehicle on the map to see its telemetry.")
            return

        msg = self._lookup(self._vehicle_id, self._time_s)
        if msg is None:
            self.title_label.setText(f"Vehicle {self._vehicle_id}")
            self.fields_label.setText("No telemetry recorded yet at this time.")
            return

        self.title_label.setText(self.title_format(msg))
        lines = []
        for spec in self.fields:
            name = spec["name"]
            if name not in msg:
                continue
            value = msg[name]
            fmt = spec.get("format")
            label = spec.get("label", name)
            lines.append(f"{label}: {fmt(value) if fmt else value}")
        self.fields_label.setText("\n".join(lines))
