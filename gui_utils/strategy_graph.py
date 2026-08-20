"""
QGraphicsView state-machine diagram of a Strategy: one rounded node per
Rule (laid out in priority bands, highest priority at the top) plus a
FALLBACK node at the bottom, colored by tactic type. Edges run from
each rule to every strictly-higher-priority rule (labeled with that
rule's trigger -- the condition that would preempt the source), plus a
completion edge from each rule back to FALLBACK and an activation edge
from FALLBACK to each rule (labeled with the rule's own trigger).

Live mode: `StrategyGraphPanel.set_active_rule(name)` pulses the active
node (None = FALLBACK is active); `record_transition(...)` -- fed from
the "behavior_change" events StrategyController logs to Match.events on
every switch -- flashes the edge that was actually traversed and adds a
line to the transition-history strip underneath the graph. Clicking a
node emits `node_clicked(name)` so a host window can sync the
strategy_editor.py selection.

Game-agnostic: only reads common_sim.control.strategy/tactics types by
name, never game_specific.
"""
from __future__ import annotations

from pyqtgraph.Qt import QtCore, QtGui, QtWidgets

from common_sim.control.strategy import Strategy
from gui_utils import theme
from gui_utils.doc_tags import document

Qt = QtCore.Qt

# Matches the literal string StrategyController._switch_to logs to
# match.events for "no rule active" on either side of a transition --
# keeping this the same string means record_transition can pass event
# data straight through with no translation.
FALLBACK_KEY = "fallback"

TACTIC_COLORS = {
    "Collect": theme.ACCENT_CYAN,
    "Score": theme.ACCENT_AMBER,
    "Defend": theme.ACCENT_RED,
    "RunScript": "#be6ef0",
    "Idle": theme.TEXT_DIM,
}

NODE_W, NODE_H = 150, 46
BAND_V_GAP = 110
NODE_H_GAP = 40

# Edges "bow" sideways from the straight line between their two nodes,
# by an amount that grows with how many priority bands the edge skips.
# Without this, a strategy with one rule per priority band (the common
# case) lays every rule in a single column, and every preemption edge
# would be a straight vertical line stacked exactly on top of every
# other -- collapsing all their labels into the same spot. Fanning
# edges out (short skips hug the column, long skips arc further out,
# alternating left/right) turns that into a readable arc diagram.
ARC_BASE = 26.0
ARC_PER_BAND = 20.0

ZOOM_MIN, ZOOM_MAX = 0.2, 4.0
ZOOM_STEP = 1.15


def _key(name: str | None) -> str:
    return name if name else FALLBACK_KEY


def _tactic_color(tactic) -> QtGui.QColor:
    return QtGui.QColor(TACTIC_COLORS.get(type(tactic).__name__, theme.TEXT_DIM))


class _NodeItem(QtWidgets.QGraphicsObject):
    def __init__(self, key: str, title: str, subtitle: str, color: QtGui.QColor, view: "StrategyGraphView"):
        super().__init__()
        self.key = key
        self.color = color
        self.title = title
        self.subtitle = subtitle
        self.active = False
        self.selected = False
        self._view = view
        self.setAcceptHoverEvents(True)
        self.setCursor(Qt.PointingHandCursor)

    def boundingRect(self) -> QtCore.QRectF:
        return QtCore.QRectF(0, 0, NODE_W, NODE_H)

    def paint(self, painter, option, widget=None) -> None:
        rect = self.boundingRect()
        painter.setRenderHint(QtGui.QPainter.Antialiasing)
        pen_color = QtGui.QColor(theme.ACCENT_CYAN) if self.active else self.color
        pen = QtGui.QPen(pen_color, 3 if self.active else 1.5)
        if self.selected and not self.active:
            pen.setStyle(Qt.DashLine)
            pen.setWidthF(2.0)
        painter.setPen(pen)
        fill = QtGui.QColor(self.color)
        fill.setAlpha(70 if self.active else 32)
        painter.setBrush(fill)
        painter.drawRoundedRect(rect, 8, 8)

        painter.setPen(QtGui.QColor(theme.TEXT_PRIMARY))
        painter.setFont(theme.technical_font(9, bold=True))
        painter.drawText(QtCore.QRectF(5, 3, NODE_W - 10, 18), Qt.AlignLeft | Qt.AlignVCenter, self.title)
        painter.setPen(QtGui.QColor(theme.TEXT_DIM))
        painter.setFont(theme.technical_font(7))
        painter.drawText(
            QtCore.QRectF(5, 21, NODE_W - 10, NODE_H - 23), Qt.AlignLeft | Qt.TextWordWrap, self.subtitle,
        )

    def mousePressEvent(self, event) -> None:
        super().mousePressEvent(event)
        self._view.toggle_selected(self.key)
        # A host window can respond to node_clicked by rebuilding the
        # graph (scene().clear()), which deletes this item's C++ object.
        # Doing that while we're still on the scene's mouse-event-delivery
        # call stack (this item may still be the mouse grabber) corrupts
        # Qt's internal state and crashes -- so defer the notification to
        # the next event-loop turn, and only capture plain values (never
        # `self`) in the deferred call, since this item may be dead by
        # the time it fires.
        key, view = self.key, self._view
        QtCore.QTimer.singleShot(0, lambda: view.node_clicked.emit(key))

    def hoverEnterEvent(self, event) -> None:
        super().hoverEnterEvent(event)
        self._view.set_hovered(self.key)

    def hoverLeaveEvent(self, event) -> None:
        super().hoverLeaveEvent(event)
        self._view.set_hovered(None)

    def set_active(self, active: bool) -> None:
        if active != self.active:
            self.active = active
            self.update()

    def set_selected(self, selected: bool) -> None:
        if selected != self.selected:
            self.selected = selected
            self.update()


class _EdgeItem(QtWidgets.QGraphicsPathItem):
    def __init__(self, src: _NodeItem, dst: _NodeItem, label: str, bow: float = 0.0):
        super().__init__()
        self.src = src
        self.dst = dst
        self.bow = bow
        self._normal_pen = QtGui.QPen(QtGui.QColor(theme.BORDER), 1.4)
        self._flash_pen = QtGui.QPen(QtGui.QColor(theme.ACCENT_CYAN), 3)
        self.setPen(self._normal_pen)
        self.setZValue(-1)
        self.label_bg = QtWidgets.QGraphicsRectItem(self)
        self.label_bg.setPen(QtGui.QPen(Qt.NoPen))
        bg_color = QtGui.QColor(theme.BG_DEEP)
        bg_color.setAlpha(190)
        self.label_bg.setBrush(bg_color)
        self.label_bg.setZValue(0)
        self.label_item = QtWidgets.QGraphicsSimpleTextItem(label, self)
        self.label_item.setFont(theme.technical_font(7))
        self.label_item.setBrush(QtGui.QColor(theme.TEXT_DIM))
        self.label_item.setZValue(1)
        # Labels are hidden until a connected node is hovered/selected --
        # with every rule preempting every higher-priority rule, showing
        # all labels at once buries the graph in overlapping text.
        self.label_item.setVisible(False)
        self.label_bg.setVisible(False)

    def set_label_visible(self, visible: bool) -> None:
        self.label_item.setVisible(visible)
        self.label_bg.setVisible(visible)

    def update_path(self, obstacles: list[QtCore.QRectF] = ()) -> None:
        a = self.src.sceneBoundingRect().center()
        b = self.dst.sceneBoundingRect().center()
        mid = QtCore.QPointF((a.x() + b.x()) / 2.0, (a.y() + b.y()) / 2.0)
        dx, dy = b.x() - a.x(), b.y() - a.y()
        length = (dx * dx + dy * dy) ** 0.5 or 1.0
        # Perpendicular unit vector, so `bow` shifts the control point
        # sideways off the a-b line regardless of the edge's own slope.
        perp = QtCore.QPointF(-dy / length, dx / length)
        control = QtCore.QPointF(mid.x() + perp.x() * self.bow, mid.y() + perp.y() * self.bow)
        path = QtGui.QPainterPath(a)
        path.quadTo(control, b)
        self.setPath(path)

        label_rect = self.label_item.boundingRect()
        label_pos = path.pointAtPercent(0.5)
        # Push the label further out along the same sideways direction as
        # the curve's bow until it clears every other node's box -- the
        # path itself may still graze a node it passes, but the label
        # (the thing that's actually unreadable when it overlaps) won't
        # sit on top of one.
        push_dir = 1.0 if self.bow >= 0 else -1.0
        push = 0.0
        for _ in range(10):
            candidate = QtCore.QRectF(
                label_pos.x() + perp.x() * push - label_rect.width() / 2.0 - 2,
                label_pos.y() + perp.y() * push - label_rect.height() / 2.0 - 1,
                label_rect.width() + 4, label_rect.height() + 2,
            )
            if not any(candidate.intersects(rect) for rect in obstacles):
                break
            push += push_dir * 14.0
        final = QtCore.QPointF(label_pos.x() + perp.x() * push, label_pos.y() + perp.y() * push)
        self.label_item.setPos(final.x() - label_rect.width() / 2.0, final.y() - label_rect.height() / 2.0)
        self.label_bg.setRect(self.label_item.pos().x() - 2, self.label_item.pos().y() - 1,
                               label_rect.width() + 4, label_rect.height() + 2)

    def flash(self) -> None:
        self.setPen(self._flash_pen)
        QtCore.QTimer.singleShot(700, lambda: self.setPen(self._normal_pen))


def _layout(strategy: Strategy) -> dict[str, QtCore.QPointF]:
    bands: dict[int, list] = {}
    for rule in strategy.rules:
        bands.setdefault(rule.priority, []).append(rule)

    positions: dict[str, QtCore.QPointF] = {}
    y = 20.0
    for priority in sorted(bands, reverse=True):
        row = bands[priority]
        total_w = len(row) * NODE_W + (len(row) - 1) * NODE_H_GAP
        x = -total_w / 2.0
        for rule in row:
            positions[rule.name] = QtCore.QPointF(x, y)
            x += NODE_W + NODE_H_GAP
        y += BAND_V_GAP
    positions[FALLBACK_KEY] = QtCore.QPointF(-NODE_W / 2.0, y)
    return positions


class StrategyGraphView(QtWidgets.QGraphicsView):
    node_clicked = QtCore.Signal(str)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setScene(QtWidgets.QGraphicsScene(self))
        self.setRenderHint(QtGui.QPainter.Antialiasing)
        self.setDragMode(QtWidgets.QGraphicsView.ScrollHandDrag)
        self.setTransformationAnchor(QtWidgets.QGraphicsView.AnchorUnderMouse)
        self.setResizeAnchor(QtWidgets.QGraphicsView.AnchorViewCenter)
        self.setStyleSheet(f"background-color: {theme.BG_DEEP}; border: 1px solid {theme.BORDER};")
        self.strategy: Strategy | None = None
        self._nodes: dict[str, _NodeItem] = {}
        self._edges: dict[tuple[str, str], _EdgeItem] = {}
        self._active_key = FALLBACK_KEY
        self._zoom = 1.0
        self._hovered_key: str | None = None
        self._selected_key: str | None = None

    def set_strategy(self, strategy: Strategy) -> None:
        self.strategy = strategy
        self.scene().clear()
        self._nodes = {}
        self._edges = {}
        self._hovered_key = None
        self._selected_key = None
        positions = _layout(strategy)

        for rule in strategy.rules:
            node = _NodeItem(
                rule.name, rule.name, f"{type(rule.tactic).__name__} · P{rule.priority}",
                _tactic_color(rule.tactic), self,
            )
            node.setPos(positions[rule.name])
            self.scene().addItem(node)
            self._nodes[rule.name] = node

        fallback_node = _NodeItem(
            FALLBACK_KEY, "FALLBACK", type(strategy.fallback).__name__, _tactic_color(strategy.fallback), self,
        )
        fallback_node.setPos(positions[FALLBACK_KEY])
        self.scene().addItem(fallback_node)
        self._nodes[FALLBACK_KEY] = fallback_node

        band_rank = {p: i for i, p in enumerate(sorted({r.priority for r in strategy.rules}, reverse=True))}
        fallback_rank = len(band_rank)
        arc_side = 1
        for rule in strategy.rules:
            src = self._nodes[rule.name]
            for other in strategy.rules:
                if other.priority > rule.priority:
                    skip = abs(band_rank[other.priority] - band_rank[rule.priority])
                    bow = arc_side * (ARC_BASE + ARC_PER_BAND * (skip - 1))
                    self._add_edge(src, self._nodes[other.name], other.trigger.describe(), bow)
                    arc_side *= -1
            skip = abs(fallback_rank - band_rank[rule.priority])
            self._add_edge(src, fallback_node, "done / trigger false", arc_side * (ARC_BASE + ARC_PER_BAND * (skip - 1)))
            arc_side *= -1
            self._add_edge(fallback_node, src, rule.trigger.describe(), arc_side * (ARC_BASE + ARC_PER_BAND * (skip - 1)))
            arc_side *= -1

        node_rects = {key: node.sceneBoundingRect() for key, node in self._nodes.items()}
        for edge in self._edges.values():
            obstacles = [r for key, r in node_rects.items() if key not in (edge.src.key, edge.dst.key)]
            edge.update_path(obstacles)

        self._active_key = FALLBACK_KEY
        fallback_node.set_active(True)

        margin = 30
        self.setSceneRect(self.scene().itemsBoundingRect().adjusted(-margin, -margin, margin, margin))
        self.fit_view()

    # -- zoom -------------------------------------------------------------

    def wheelEvent(self, event: QtGui.QWheelEvent) -> None:
        factor = ZOOM_STEP if event.angleDelta().y() > 0 else 1.0 / ZOOM_STEP
        self._apply_zoom(factor)
        event.accept()

    def _apply_zoom(self, factor: float) -> None:
        new_zoom = max(ZOOM_MIN, min(ZOOM_MAX, self._zoom * factor))
        applied = new_zoom / self._zoom
        if applied == 1.0:
            return
        self.scale(applied, applied)
        self._zoom = new_zoom

    def zoom_in(self) -> None:
        self._apply_zoom(ZOOM_STEP)

    def zoom_out(self) -> None:
        self._apply_zoom(1.0 / ZOOM_STEP)

    def fit_view(self) -> None:
        if not self.scene().itemsBoundingRect().isEmpty():
            self.fitInView(self.scene().itemsBoundingRect(), Qt.KeepAspectRatio)
            self._zoom = self.transform().m11()

    def _add_edge(self, src: _NodeItem, dst: _NodeItem, label: str, bow: float = 0.0) -> None:
        edge = _EdgeItem(src, dst, label, bow)
        self.scene().addItem(edge)
        self._edges[(src.key, dst.key)] = edge

    # -- edge-label visibility ---------------------------------------------
    # All labels are hidden by default (see _EdgeItem); a node's edges'
    # labels appear while it's hovered, and stay pinned visible while
    # it's the selected node, so the reader picks one node at a time
    # instead of the whole graph's labels fighting for the same space.

    def set_hovered(self, key: str | None) -> None:
        if key != self._hovered_key:
            self._hovered_key = key
            self._refresh_edge_labels()

    def toggle_selected(self, key: str) -> None:
        if self._selected_key in self._nodes:
            self._nodes[self._selected_key].set_selected(False)
        self._selected_key = None if key == self._selected_key else key
        if self._selected_key in self._nodes:
            self._nodes[self._selected_key].set_selected(True)
        self._refresh_edge_labels()

    def clear_selection(self) -> None:
        if self._selected_key is not None:
            if self._selected_key in self._nodes:
                self._nodes[self._selected_key].set_selected(False)
            self._selected_key = None
            self._refresh_edge_labels()

    def _refresh_edge_labels(self) -> None:
        active = {k for k in (self._hovered_key, self._selected_key) if k}
        for edge in self._edges.values():
            edge.set_label_visible(edge.src.key in active or edge.dst.key in active)

    def mousePressEvent(self, event) -> None:
        if self.itemAt(event.pos()) is None:
            self.clear_selection()
        super().mousePressEvent(event)

    def set_active_rule(self, name: str | None) -> None:
        key = _key(name)
        if key == self._active_key:
            return
        if self._active_key in self._nodes:
            self._nodes[self._active_key].set_active(False)
        self._active_key = key
        if key in self._nodes:
            self._nodes[key].set_active(True)

    def flash_transition(self, from_name: str | None, to_name: str | None) -> None:
        edge = self._edges.get((_key(from_name), _key(to_name)))
        if edge is not None:
            edge.flash()


class TransitionHistory(QtWidgets.QListWidget):
    """Rolling strip of recent behavior_change transitions, newest on
    top -- a lightweight timeline alongside the graph's live pulsing."""

    MAX_ENTRIES = 12

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMaximumHeight(90)
        self.setFont(theme.technical_font(8))

    def add_entry(self, elapsed: float, from_name: str | None, to_name: str | None, trigger_desc: str | None) -> None:
        from_label = _key(from_name).upper() if _key(from_name) == FALLBACK_KEY else from_name
        to_label = _key(to_name).upper() if _key(to_name) == FALLBACK_KEY else to_name
        text = f"t={elapsed:6.1f}s  {from_label} -> {to_label}"
        if trigger_desc:
            text += f"  ({trigger_desc})"
        self.insertItem(0, text)
        while self.count() > self.MAX_ENTRIES:
            self.takeItem(self.count() - 1)

    def clear_history(self) -> None:
        self.clear()


class StrategyGraphPanel(QtWidgets.QWidget):
    """Graph + transition-history strip, bundled -- the unit a STRATEGY
    tab embeds. `record_transition` is the single call a host makes on
    every "behavior_change" event: it flashes the traversed edge and
    appends to the history strip in one go."""

    node_clicked = QtCore.Signal(str)

    def __init__(self, parent=None):
        super().__init__(parent)
        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)

        header_row = QtWidgets.QHBoxLayout()
        header = QtWidgets.QLabel("STRATEGY GRAPH")
        header.setStyleSheet(f"color: {theme.ACCENT_CYAN}; font-weight: bold;")
        header_row.addWidget(header)
        header_row.addStretch(1)
        for text, slot, tip in (
            ("-", lambda: self.graph.zoom_out(), "Zoom out"),
            ("Fit", lambda: self.graph.fit_view(), "Fit graph to view"),
            ("+", lambda: self.graph.zoom_in(), "Zoom in"),
        ):
            button = QtWidgets.QPushButton(text)
            button.setFixedWidth(36)
            button.setToolTip(tip)
            button.clicked.connect(slot)
            header_row.addWidget(button)
        layout.addLayout(header_row)

        self.graph = document(
            StrategyGraphView(), "graph_view", "The state diagram",
            "One box per rule, arranged by priority with the highest at the top and FALLBACK at "
            "the bottom, coloured by what tactic it runs. Lines show which rule can interrupt "
            "which.",
            "While a match is running, the active rule glows and the edge just crossed flashes "
            "-- this is the fastest way to actually see the order your rules fire in, rather "
            "than working it out by reading the list. Click a box to jump the STRATEGY tab's "
            "inspector straight to that rule.")
        self.graph.node_clicked.connect(self.node_clicked)
        layout.addWidget(self.graph, stretch=1)

        self.history = document(
            TransitionHistory(), "history", "Transition history",
            "A running log of every rule switch this robot has made this match, most recent "
            "first.",
            "Useful for answering \"why did it just do that\" after the fact, once the moment "
            "on the graph itself has already flashed and faded.")
        layout.addWidget(self.history)

    def set_strategy(self, strategy: Strategy) -> None:
        self.graph.set_strategy(strategy)
        self.history.clear_history()

    def set_active_rule(self, name: str | None) -> None:
        self.graph.set_active_rule(name)

    def record_transition(
        self, elapsed: float, from_name: str | None, to_name: str | None, trigger_desc: str | None,
    ) -> None:
        self.graph.flash_transition(from_name, to_name)
        self.history.add_entry(elapsed, from_name, to_name, trigger_desc)
