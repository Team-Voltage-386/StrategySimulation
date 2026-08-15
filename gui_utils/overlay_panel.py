"""
Reusable semi-transparent overlay button panel that floats over any
QWidget-based canvas (matplotlib FigureCanvas, pyqtgraph PlotWidget,
etc.). Extracted out of overlay_demo.py / overlay_demo_pyqtgraph.py so
real apps can wire their own controls into the same overlay chrome
instead of copy-pasting it -- see pyqtgraph_earth_demo.py for a live
example (detail-level + marker-size controls).

- Click a main button: it highlights and slides out a row of controls
  to its right. Click it again, or click anywhere else, to collapse.
- Pop-out rows can mix plain toggle/action buttons, a mini spinner
  (value + up/down), and a combo (click to reveal alternatives). Each
  spec can carry a callback ("on_toggle"/"on_click"/"on_change") so the
  control actually drives application state, not just its own label.
- Spinners/combos can optionally be given a caption, which wraps them
  in a frame with the label above the control (double height).
- Sub-panels are positioned as floating overlays (not part of the
  main-button column layout), so opening a taller sub-panel never
  shifts the vertical spacing of the main buttons -- it just extends
  downward from a fixed top edge.
- Toggle buttons get a visible border to distinguish them from
  momentary action buttons.

Usage:
    sub_options = {
        "Detail": [
            {"type": "combo", "options": ["Low", "High"], "default": "Low",
             "caption": "Detail level", "on_change": my_handler},
        ],
    }
    overlay = OverlayPanel(canvas, sub_options)
    overlay.reposition()
    # in the host window's resizeEvent: overlay.reposition()
    # in an app-wide eventFilter: overlay.collapse() when a click lands
    # outside overlay.contains_widget(clicked_widget)

Qt binding: imported via pyqtgraph.Qt rather than a hardcoded PyQt5,
so this stays usable regardless of which binding pyqtgraph itself
resolved to (PyQt5/PySide2/PySide6/...). pyqtgraph.Qt picks whichever
binding is already imported in the process if one is, otherwise falls
back to its own preference order -- mixing a hardcoded PyQt5 import
here with a caller that ends up on a different binding (e.g. PySide6)
produces widgets that look identical but are incompatible C++ types,
which surfaces as a confusing TypeError at the first parent/child call
between them.
"""
from pyqtgraph.Qt import QtCore, QtGui, QtWidgets

Qt = QtCore.Qt
QSize = QtCore.QSize
QVariantAnimation = QtCore.QVariantAnimation
QEasingCurve = QtCore.QEasingCurve
QIcon = QtGui.QIcon
QWidget = QtWidgets.QWidget
QVBoxLayout = QtWidgets.QVBoxLayout
QHBoxLayout = QtWidgets.QHBoxLayout
QPushButton = QtWidgets.QPushButton
QButtonGroup = QtWidgets.QButtonGroup
QSizePolicy = QtWidgets.QSizePolicy
QLabel = QtWidgets.QLabel
QMenu = QtWidgets.QMenu
QFrame = QtWidgets.QFrame
QStyleFactory = QtWidgets.QStyleFactory
QApplication = QtWidgets.QApplication

__all__ = [
    "apply_fusion_style",
    "scaled_font",
    "make_control_widget",
    "MiniSpinner",
    "ComboControl",
    "SubPanel",
    "OverlayPanel",
    "install_click_outside_collapse",
]

# Fusion is required for our RGBA-based QSS backgrounds to render with
# real transparency -- some native styles flatten alpha to an opaque
# color regardless of what QSS says. Rather than calling
# QApplication.setStyle("Fusion") (which also silently swaps the whole
# app's default font, since Fusion doesn't inherit the OS-detected font
# the way native styles do), we scope Fusion to just the overlay
# widgets. Everything else in the app -- including its font -- stays on
# the native platform style.


def apply_fusion_style(widget: QWidget):
    """Recursively apply the Fusion style to `widget` and all its
    descendants, without touching QApplication's global style/font.

    A fresh QStyle is created per call rather than sharing one cached
    instance across widgets: setStyle() parents the style object to
    whichever widget it was last applied to, so a shared instance becomes a
    dangling, silently-corrupting handle once an earlier owner (e.g. a panel
    from a previous test) is destroyed -- QStyleFactory.create() is cheap
    enough that there's no real cost to not reusing one."""
    style = QStyleFactory.create("Fusion")
    widget.setStyle(style)
    for child in widget.findChildren(QWidget):
        child.setStyle(style)


def scaled_font(delta_points: int = 0, bold: bool = False):
    """
    Derive a font from the application's actual default font (which is
    correctly DPI-scaled by Qt/the OS) rather than hardcoding a pixel
    size in QSS. QSS 'px' font sizes are literal pixels and do not
    scale with display DPI the way QFont point sizes do -- that
    mismatch is what made small overlay text look tiny next to a
    native menu bar.
    """
    font = QApplication.font()
    font.setPointSize(max(6, font.pointSize() + delta_points))
    font.setBold(bold)
    return font


MAIN_BTN_SIZE = (120, 42)
ROW_SPACING = 6            # vertical gap between main buttons -- always constant
ROW_GAP = 8                # horizontal gap between a main button and its sub-panel
ROW_HEIGHT = MAIN_BTN_SIZE[1]
FRAME_HEIGHT = ROW_HEIGHT * 2

# Cyan/electric-blue "starship bridge" HUD palette -- shared visual language
# with theme.py's app-wide QSS (ACCENT_CYAN = #00e5ff = rgb(0, 229, 255)).
# Kept as literal rgba() strings rather than importing theme.py so this
# module stays usable standalone (see overlay_demo*.py).
MAIN_BTN_STYLE = """
QPushButton {
    background-color: rgba(8, 14, 20, 170);
    color: rgb(214, 250, 255);
    border: 1px solid rgba(0, 229, 255, 60);
    border-radius: 8px;
    padding: 6px;
    font-weight: 600;
    text-align: left;
    padding-left: 12px;
}
QPushButton:hover { background-color: rgba(20, 40, 54, 200); }
QPushButton:checked {
    background-color: rgba(0, 229, 255, 90);
    border: 1px solid rgba(0, 229, 255, 220);
    color: rgb(6, 14, 18);
}
"""

SUB_TOGGLE_BTN_STYLE = """
QPushButton {
    background-color: rgba(14, 24, 32, 180);
    color: rgb(214, 250, 255);
    border: 1.5px solid rgba(0, 229, 255, 90);
    border-radius: 6px;
    padding: 0 10px;
}
QPushButton:hover { background-color: rgba(24, 46, 60, 200); }
QPushButton:checked {
    background-color: rgba(0, 229, 255, 210);
    border: 1.5px solid rgba(214, 250, 255, 210);
    color: rgb(6, 14, 18);
}
"""

SUB_ACTION_BTN_STYLE = """
QPushButton {
    background-color: rgba(14, 24, 32, 180);
    color: rgb(214, 250, 255);
    border: 1px solid transparent;
    border-radius: 6px;
    padding: 0 10px;
}
QPushButton:hover { background-color: rgba(24, 46, 60, 200); }
QPushButton:pressed { background-color: rgba(0, 229, 255, 160); }
"""

SPIN_BTN_STYLE = """
QPushButton {
    background-color: rgba(14, 24, 32, 180);
    color: rgb(214, 250, 255);
    border: 1px solid rgba(0, 229, 255, 40);
    border-radius: 3px;
}
QPushButton:hover { background-color: rgba(24, 46, 60, 200); }
QPushButton:pressed { background-color: rgba(0, 229, 255, 200); }
"""

COMBO_BTN_STYLE = """
QPushButton {
    background-color: rgba(14, 24, 32, 180);
    color: rgb(214, 250, 255);
    border: 1px solid rgba(0, 229, 255, 40);
    border-radius: 6px;
    padding: 0 10px;
    text-align: left;
}
QPushButton:hover { background-color: rgba(24, 46, 60, 200); }
QPushButton::menu-indicator { image: none; }
"""

COMBO_MENU_STYLE = """
QMenu {
    background-color: rgba(6, 12, 17, 240);
    color: rgb(214, 250, 255);
    border: 1px solid rgba(0, 229, 255, 60);
    border-radius: 6px;
    padding: 4px;
}
QMenu::item { padding: 6px 18px; border-radius: 4px; }
QMenu::item:selected { background-color: rgba(0, 229, 255, 200); color: rgb(6, 14, 18); }
"""

FRAME_STYLE = """
QFrame {
    background-color: rgba(10, 18, 25, 160);
    border: 1px solid rgba(0, 229, 255, 30);
    border-radius: 8px;
}
"""

CAPTION_STYLE = "color: rgba(0, 229, 255, 190); background: transparent;"


def make_control_widget(control: QWidget, caption: str = None) -> QWidget:
    """
    Optionally wrap a bare control in a frame with a caption label
    above it (doubling the height). Without a caption, the control is
    returned as-is at its normal single-row height.
    """
    if not caption:
        return control

    frame = QFrame()
    frame.setStyleSheet(FRAME_STYLE)
    layout = QVBoxLayout(frame)
    layout.setContentsMargins(8, 4, 8, 6)
    layout.setSpacing(2)

    label = QLabel(caption)
    label.setStyleSheet(CAPTION_STYLE)
    label.setFont(scaled_font(-2, bold=True))
    layout.addWidget(label)
    layout.addWidget(control)

    frame.setFixedHeight(FRAME_HEIGHT)
    return frame


class MiniSpinner(QWidget):
    """Bare value + up/down control, at ROW_HEIGHT. No built-in label."""

    def __init__(self, minimum=0, maximum=100, value=0, on_change=None, parent=None):
        super().__init__(parent)
        self.setStyleSheet("background: transparent;")
        self._value = value
        self._min = minimum
        self._max = maximum
        self._on_change = on_change

        layout = QHBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(4)

        self.value_label = QLabel(str(value))
        self.value_label.setFixedSize(30, ROW_HEIGHT)
        self.value_label.setAlignment(Qt.AlignCenter)
        self.value_label.setStyleSheet("""
            background-color: rgba(14, 24, 32, 180);
            color: rgb(214, 250, 255);
            border: 1px solid rgba(0, 229, 255, 40);
            border-radius: 4px;
        """)
        layout.addWidget(self.value_label)

        btn_col = QVBoxLayout()
        btn_col.setContentsMargins(0, 0, 0, 0)
        btn_col.setSpacing(1)
        up = QPushButton("▲")
        down = QPushButton("▼")
        half_h = (ROW_HEIGHT - 1) // 2
        for b in (up, down):
            b.setFixedSize(20, half_h)
            b.setStyleSheet(SPIN_BTN_STYLE)
            b.setFont(scaled_font(-2))
            b.setCursor(Qt.PointingHandCursor)
        btn_col.addWidget(up)
        btn_col.addWidget(down)
        layout.addLayout(btn_col)

        up.clicked.connect(lambda: self._step(1))
        down.clicked.connect(lambda: self._step(-1))
        self.setFixedHeight(ROW_HEIGHT)

    def _step(self, delta):
        self._value = max(self._min, min(self._max, self._value + delta))
        self.value_label.setText(str(self._value))
        if self._on_change:
            self._on_change(self._value)

    def value(self):
        return self._value


class ComboControl(QPushButton):
    """Bare combo button. Click to reveal alternatives below it."""

    def __init__(self, options, default=None, on_change=None, parent=None):
        current = default if default in options else options[0]
        super().__init__(f"{current}  ▾", parent)
        self.options = options
        self._current = current
        self._on_change = on_change
        self.setStyleSheet(COMBO_BTN_STYLE)
        self.setCursor(Qt.PointingHandCursor)
        self.setFixedHeight(ROW_HEIGHT)
        self.setMinimumWidth(110)
        self.clicked.connect(self._show_menu)

    def _show_menu(self):
        menu = QMenu(self)
        apply_fusion_style(menu)
        menu.setStyleSheet(COMBO_MENU_STYLE)
        for opt in self.options:
            action = menu.addAction(opt)
            action.triggered.connect(lambda checked=False, o=opt: self._select(o))
        menu.exec_(self.mapToGlobal(self.rect().bottomLeft()))

    def _select(self, opt):
        self._current = opt
        self.setText(f"{opt}  ▾")
        if self._on_change:
            self._on_change(opt)

    def value(self):
        return self._current

    def set_options(self, options):
        """Replace the alternatives offered by the menu -- e.g. a growing
        list of names discovered at runtime (see MapPanel's sensor-metric
        overlay). If the current selection isn't in the new list, falls back
        to the first entry and fires on_change so callers stay in sync."""
        self.options = list(options)
        if self._current not in self.options:
            self._select(self.options[0] if self.options else None)


class SubPanel(QWidget):
    """
    A floating row of controls that expands/collapses to the right of
    its main button. Deliberately NOT placed inside any layout -- it's
    positioned manually via move() and resized manually via resize(),
    so its own height (which can exceed ROW_HEIGHT when it contains a
    framed/captioned control) never affects the main-button column's
    layout or spacing.
    """

    def __init__(self, specs, parent=None):
        super().__init__(parent)
        self.setStyleSheet("background: transparent;")
        layout = QHBoxLayout(self)
        layout.setContentsMargins(ROW_GAP, 0, 0, 0)
        layout.setSpacing(6)
        # The bare control per spec (not its optional caption-frame wrapper),
        # in spec order -- lets a caller reach back into a live control (e.g.
        # ComboControl.set_options()) after construction. See get_control().
        self.controls = []
        for spec in specs:
            control, widget = self._build_item(spec)
            self.controls.append(control)
            layout.addWidget(widget, alignment=Qt.AlignTop)
        self.adjustSize()
        self.target_size = self.sizeHint()
        self.resize(0, self.target_size.height())
        self.setVisible(False)
        self._anim = None

    @staticmethod
    def _build_item(spec):
        """Returns (control, widget): `widget` is what goes in the layout
        (possibly a caption-framed wrapper), `control` is the bare
        interactive widget itself."""
        kind = spec["type"]
        if kind in ("toggle", "action"):
            b = QPushButton(spec["label"])
            b.setStyleSheet(SUB_TOGGLE_BTN_STYLE if kind == "toggle" else SUB_ACTION_BTN_STYLE)
            b.setCursor(Qt.PointingHandCursor)
            b.setFixedHeight(ROW_HEIGHT)
            if kind == "toggle":
                b.setCheckable(True)
                if spec.get("checked"):
                    b.setChecked(True)
                if spec.get("on_toggle"):
                    b.toggled.connect(spec["on_toggle"])
            else:
                if spec.get("on_click"):
                    b.clicked.connect(spec["on_click"])
            return b, b
        if kind == "spinner":
            control = MiniSpinner(
                spec.get("min", 0), spec.get("max", 100), spec.get("value", 0),
                on_change=spec.get("on_change"),
            )
            return control, make_control_widget(control, spec.get("caption"))
        if kind == "combo":
            control = ComboControl(
                spec["options"], default=spec.get("default"), on_change=spec.get("on_change"),
            )
            return control, make_control_widget(control, spec.get("caption"))
        raise ValueError(f"unknown sub-item type: {kind}")

    def animate(self, expand: bool):
        self.setVisible(True)
        if expand:
            self.raise_()  # draw above whatever main buttons it may overlap
        anim = QVariantAnimation(self)
        anim.setDuration(160)
        anim.setEasingCurve(QEasingCurve.OutCubic)
        anim.setStartValue(self.width())
        anim.setEndValue(self.target_size.width() if expand else 0)
        anim.valueChanged.connect(lambda v: self.resize(int(v), self.target_size.height()))
        if not expand:
            anim.finished.connect(lambda: self.setVisible(False))
        anim.start()
        self._anim = anim  # keep a reference alive


class OverlayPanel(QWidget):
    """
    Column of fixed-size main buttons, each expanding a SubPanel of
    controls to its right. Sub-panels are children of `canvas` (the
    same parent as this panel) rather than children of this widget, so
    they aren't clipped to the button column's bounds when they extend
    to the right and below it -- this widget just tracks their
    positions.

    sub_options: dict of {main_button_label: [control_spec, ...]}, in
        display order. See module docstring for spec shape.
    icons: optional dict of {main_button_label: path-like} for a custom
        icon on that main button.
    """

    def __init__(self, canvas: QWidget, sub_options: dict, icons: dict = None):
        super().__init__(canvas)
        self.setStyleSheet("background: transparent;")
        icons = icons or {}

        self.group = QButtonGroup(self)
        self.group.setExclusive(False)  # exclusivity enforced manually below

        outer = QVBoxLayout(self)
        outer.setContentsMargins(0, 0, 0, 0)
        outer.setSpacing(ROW_SPACING)
        outer.setSizeConstraint(QVBoxLayout.SetMinAndMaxSize)

        self._entries = []  # (button, subpanel)
        self._subpanel_by_label = {}
        self._canvas = canvas
        self._outer = outer

        for label, specs in sub_options.items():
            self.add_section(label, specs, icon=icons.get(label))

    def add_section(self, label, specs, icon=None):
        """Append a new main button + its pop-out control row to this
        overlay, after construction -- lets an owning app (e.g. travel_viz)
        extend MapPanel's default overlay menu with its own controls
        instead of only being able to configure it at __init__ time. See
        the module docstring's Usage section for `specs`' shape."""
        btn = QPushButton(label, self)
        btn.setCheckable(True)
        btn.setStyleSheet(MAIN_BTN_STYLE)
        btn.setCursor(Qt.PointingHandCursor)
        btn.setFixedSize(*MAIN_BTN_SIZE)
        btn.setSizePolicy(QSizePolicy.Fixed, QSizePolicy.Fixed)

        if icon and str(icon):
            from pathlib import Path
            if Path(icon).exists():
                btn.setIcon(QIcon(str(icon)))
                btn.setIconSize(QSize(18, 18))

        self._outer.addWidget(btn)

        sub = SubPanel(specs, self._canvas)  # sibling of overlay, not clipped by it
        btn.toggled.connect(lambda checked, s=sub: s.animate(checked))
        btn.toggled.connect(lambda checked, b=btn: self._on_main_toggled(b, checked))

        self.group.addButton(btn)
        self._entries.append((btn, sub))
        self._subpanel_by_label[label] = sub

        self._reposition_children()
        apply_fusion_style(self)
        apply_fusion_style(sub)
        return sub

    def _on_main_toggled(self, btn, checked):
        # Enforce "only one row open" manually: opening one closes any other.
        if checked:
            for other in self.group.buttons():
                if other is not btn and other.isChecked():
                    other.setChecked(False)

    def get_control(self, label, index=0):
        """The bare control widget (ComboControl/MiniSpinner/QPushButton) at
        `index` within the sub-panel opened by the main button `label` --
        e.g. to call ComboControl.set_options() on it later, after a caller
        discovers new alternatives at runtime. Raises KeyError/IndexError for
        an unknown label/index, same as a plain dict/list lookup would."""
        return self._subpanel_by_label[label].controls[index]

    def collapse(self):
        """Close whichever row is currently open, if any."""
        for btn in self.group.buttons():
            if btn.isChecked():
                btn.setChecked(False)

    def contains_widget(self, widget) -> bool:
        """True if `widget` is this panel, a descendant of it, or a descendant
        of one of its (separately-parented) sub-panels."""
        if widget is self or self.isAncestorOf(widget):
            return True
        for _, sub in self._entries:
            if widget is sub or sub.isAncestorOf(widget):
                return True
        return False

    def reposition(self, margin=12):
        self.move(margin, margin)
        self.raise_()
        self._reposition_children()

    def _reposition_children(self):
        base_x = self.x() + MAIN_BTN_SIZE[0] + ROW_GAP
        for index, (btn, sub) in enumerate(self._entries):
            y = self.y() + index * (MAIN_BTN_SIZE[1] + ROW_SPACING)
            sub.move(base_x, y)


def install_click_outside_collapse(app: QApplication, get_overlays):
    """
    Wires "click anywhere outside an overlay collapses it" for one or
    more OverlayPanels, via a single app-wide event filter. get_overlays
    is a zero-arg callable returning the current list of live
    OverlayPanel instances (a callable rather than a fixed list so
    panels created after this call -- or a changing set of them -- are
    still covered). Returns the filter object; keep a reference to it
    alive for as long as the app runs (Qt does not keep installed event
    filters alive on its own).
    """
    QEvent, QObject = QtCore.QEvent, QtCore.QObject

    class _CollapseFilter(QObject):
        def eventFilter(self, obj, event):
            if event.type() == QEvent.MouseButtonPress:
                # globalPos() was removed in Qt6 in favor of globalPosition()
                # (a QPointF) -- try the Qt6/PySide6 spelling first so this
                # stays usable regardless of which binding pyqtgraph.Qt
                # resolved to (see module docstring).
                if hasattr(event, "globalPosition"):
                    global_pos = event.globalPosition().toPoint()
                else:
                    global_pos = event.globalPos()
                clicked = QApplication.widgetAt(global_pos)
                for overlay in get_overlays():
                    if clicked is None or not overlay.contains_widget(clicked):
                        overlay.collapse()
            return False

    f = _CollapseFilter()
    app.installEventFilter(f)
    return f
