"""
Sci-fi "starship nav console" visual theme for the flight-map app
(pyqtgraph_earth_demo.py / map_panel.py / overlay_panel.py / info_panel.py /
messages_panel.py). One shared palette + one QSS stylesheet applied once at
app startup, so every stock Qt widget across MainWindow, both docks, both
tabs and the messages table gets reskinned via QSS cascading rather than by
touching each widget-construction call site individually.

Bespoke-painted surfaces that QSS can't reach (the pyqtgraph plot canvas,
vehicle markers, the OverlayPanel's own rgba() QSS strings) pick up this
same palette directly -- see map_panel.py's canvas setup and
overlay_panel.py's *_STYLE constants.
"""
import base64

from pyqtgraph.Qt import QtGui, QtWidgets

# -- palette: cyan/electric-blue "starship bridge" HUD -----------------
BG_DEEP = "#05080d"            # deep-space background (windows, canvas)
BG_PANEL = "#0c131c"           # docks/tabs/table panel background
BG_RAISED = "#121b26"          # inputs, table headers, tab bar
BG_HOVER = "#1a2836"
ACCENT_CYAN = "#00e5ff"        # primary accent -- selection, focus, checked
ACCENT_CYAN_DIM = "#0a3a45"    # borders / unfocused accent
ACCENT_AMBER = "#ffb020"       # warnings / secondary accent
ACCENT_RED = "#ff4d4d"         # alerts
TEXT_PRIMARY = "#d6faff"       # near-white cyan-tinted text
TEXT_DIM = "#6f93a0"
GRID_LINE = "#0e2630"          # map grid lines on the dark canvas
BORDER = "#173142"

MONO_FONT_FAMILY = "Consolas"  # Windows-only; see MONO_FONT_FALLBACKS below
# Ordered candidates tried by technical_font() when Consolas isn't installed
# (any non-Windows platform) -- DejaVu Sans Mono/Liberation Mono cover most
# Linux distros, Menlo covers macOS. setStyleHint(Monospace) is the last
# resort if none of these are present either.
MONO_FONT_FALLBACKS = [MONO_FONT_FAMILY, "DejaVu Sans Mono", "Liberation Mono", "Menlo", "Courier New"]

__all__ = [
    "BG_DEEP", "BG_PANEL", "BG_RAISED", "BG_HOVER",
    "ACCENT_CYAN", "ACCENT_CYAN_DIM", "ACCENT_AMBER", "ACCENT_RED",
    "TEXT_PRIMARY", "TEXT_DIM", "GRID_LINE", "BORDER",
    "MONO_FONT_FAMILY", "MONO_FONT_FALLBACKS",
    "SCI_FI_QSS",
    "technical_font", "apply_app_theme",
]


def technical_font(point_size=None, bold=False):
    """QFont for HUD-style readouts -- monospace, so numeric columns and
    telemetry labels line up like an instrument display."""
    font = QtGui.QFont()
    if hasattr(font, "setFamilies"):
        # Qt 5.13+: first family in the list that's actually installed wins,
        # rather than relying solely on styleHint's generic monospace match.
        font.setFamilies(MONO_FONT_FALLBACKS)
    else:
        font.setFamily(MONO_FONT_FAMILY)
    font.setStyleHint(QtGui.QFont.Monospace)
    if point_size is not None:
        font.setPointSize(point_size)
    font.setBold(bold)
    return font


def _triangle_data_uri(points: str, fill: str) -> str:
    """QSS's default QStyleSheetStyle draws NO arrow glyph at all for a
    QSpinBox/QComboBox once its up/down-button geometry is customized via
    QSS (the native style's built-in arrow primitive only fires when the
    subcontrol is left untouched) -- so the up/down-arrow subcontrols need
    an explicit image, not just a width/height, to render anything."""
    svg = f'<svg xmlns="http://www.w3.org/2000/svg" width="10" height="6"><polygon points="{points}" fill="{fill}"/></svg>'
    return "data:image/svg+xml;base64," + base64.b64encode(svg.encode()).decode()


_UP_ARROW_URI = _triangle_data_uri("0,6 5,0 10,6", TEXT_PRIMARY)
_DOWN_ARROW_URI = _triangle_data_uri("0,0 10,0 5,6", TEXT_PRIMARY)


SCI_FI_QSS = f"""
QWidget {{
    background-color: {BG_DEEP};
    color: {TEXT_PRIMARY};
    selection-background-color: {ACCENT_CYAN_DIM};
    selection-color: {TEXT_PRIMARY};
}}

QMainWindow, QDockWidget, QTabWidget::pane {{
    background-color: {BG_DEEP};
    border: 1px solid {BORDER};
}}

QDockWidget {{
    titlebar-close-icon: none;
}}
QDockWidget::title {{
    background-color: {BG_RAISED};
    color: {ACCENT_CYAN};
    padding: 4px 8px;
    border: 1px solid {BORDER};
    font-weight: 600;
}}

QTabWidget::pane {{
    top: -1px;
}}
QTabBar::tab {{
    background-color: {BG_PANEL};
    color: {TEXT_DIM};
    border: 1px solid {BORDER};
    border-bottom: none;
    padding: 6px 16px;
}}
QTabBar::tab:selected {{
    background-color: {BG_RAISED};
    color: {ACCENT_CYAN};
    border-bottom: 2px solid {ACCENT_CYAN};
}}
QTabBar::tab:hover:!selected {{
    background-color: {BG_HOVER};
    color: {TEXT_PRIMARY};
}}

QGroupBox {{
    border: 1px solid {BORDER};
    margin-top: 8px;
    padding-top: 6px;
    color: {ACCENT_CYAN};
}}

QLabel {{
    background: transparent;
}}

QPushButton {{
    background-color: {BG_RAISED};
    color: {TEXT_PRIMARY};
    border: 1px solid {BORDER};
    border-radius: 4px;
    padding: 4px 10px;
}}
QPushButton:hover {{
    background-color: {BG_HOVER};
    border: 1px solid {ACCENT_CYAN_DIM};
}}
QPushButton:pressed, QPushButton:checked {{
    background-color: {ACCENT_CYAN_DIM};
    color: {ACCENT_CYAN};
    border: 1px solid {ACCENT_CYAN};
}}
QPushButton:disabled {{
    color: {TEXT_DIM};
    border: 1px solid {BORDER};
}}

QCheckBox, QRadioButton {{
    background: transparent;
    spacing: 6px;
}}
QCheckBox::indicator, QRadioButton::indicator {{
    width: 13px;
    height: 13px;
    border: 1px solid {BORDER};
    background-color: {BG_RAISED};
}}
QCheckBox::indicator:checked, QRadioButton::indicator:checked {{
    background-color: {ACCENT_CYAN};
    border: 1px solid {ACCENT_CYAN};
}}

QComboBox, QSpinBox, QDoubleSpinBox, QLineEdit {{
    background-color: {BG_RAISED};
    color: {TEXT_PRIMARY};
    border: 1px solid {BORDER};
    border-radius: 3px;
    padding: 2px 6px;
}}
QComboBox:hover, QSpinBox:hover, QLineEdit:hover {{
    border: 1px solid {ACCENT_CYAN_DIM};
}}
QComboBox::drop-down {{
    border: none;
    width: 20px;
}}
QSpinBox::up-button, QDoubleSpinBox::up-button {{
    subcontrol-origin: border;
    subcontrol-position: top right;
    width: 20px;
    height: 11px;
    border-left: 1px solid {BORDER};
    border-bottom: 1px solid {BORDER};
    background-color: {BG_HOVER};
}}
QSpinBox::down-button, QDoubleSpinBox::down-button {{
    subcontrol-origin: border;
    subcontrol-position: bottom right;
    width: 20px;
    height: 11px;
    border-left: 1px solid {BORDER};
    border-top: 1px solid {BORDER};
    background-color: {BG_HOVER};
}}
QSpinBox::up-button:hover, QDoubleSpinBox::up-button:hover,
QSpinBox::down-button:hover, QDoubleSpinBox::down-button:hover {{
    background-color: {ACCENT_CYAN_DIM};
}}
QSpinBox::up-arrow, QDoubleSpinBox::up-arrow {{
    image: url({_UP_ARROW_URI});
    width: 10px;
    height: 6px;
}}
QSpinBox::down-arrow, QDoubleSpinBox::down-arrow {{
    image: url({_DOWN_ARROW_URI});
    width: 10px;
    height: 6px;
}}
QComboBox QAbstractItemView {{
    background-color: {BG_RAISED};
    color: {TEXT_PRIMARY};
    selection-background-color: {ACCENT_CYAN_DIM};
    selection-color: {ACCENT_CYAN};
    border: 1px solid {BORDER};
}}

QSlider::groove:horizontal {{
    height: 4px;
    background: {BORDER};
}}
QSlider::sub-page:horizontal {{
    background: {ACCENT_CYAN_DIM};
}}
QSlider::handle:horizontal {{
    background: {ACCENT_CYAN};
    width: 12px;
    margin: -5px 0;
    border-radius: 6px;
}}

QTableView, QTreeView, QListView {{
    background-color: {BG_PANEL};
    alternate-background-color: {BG_RAISED};
    color: {TEXT_PRIMARY};
    gridline-color: {BORDER};
    border: 1px solid {BORDER};
    selection-background-color: {ACCENT_CYAN_DIM};
    selection-color: {ACCENT_CYAN};
}}
QHeaderView::section {{
    background-color: {BG_RAISED};
    color: {ACCENT_CYAN};
    padding: 4px;
    border: 1px solid {BORDER};
}}

QMenu {{
    background-color: {BG_PANEL};
    color: {TEXT_PRIMARY};
    border: 1px solid {BORDER};
}}
QMenu::item:selected {{
    background-color: {ACCENT_CYAN_DIM};
    color: {ACCENT_CYAN};
}}

QScrollBar:vertical {{
    background: {BG_PANEL};
    width: 14px;
    margin: 0px;
    border: none;
}}
QScrollBar:horizontal {{
    background: {BG_PANEL};
    height: 14px;
    margin: 0px;
    border: none;
}}
QScrollBar::handle:vertical {{
    background: {BORDER};
    border-radius: 3px;
    min-height: 24px;
}}
QScrollBar::handle:horizontal {{
    background: {BORDER};
    border-radius: 3px;
    min-width: 24px;
}}
QScrollBar::handle:hover {{
    background: {ACCENT_CYAN_DIM};
}}
QScrollBar::add-line, QScrollBar::sub-line {{
    height: 0;
    width: 0;
    background: none;
    border: none;
}}
QScrollBar::add-page, QScrollBar::sub-page {{
    background: {BG_PANEL};
}}

QStatusBar {{
    background-color: {BG_PANEL};
    color: {TEXT_DIM};
}}

QToolTip {{
    background-color: {BG_RAISED};
    color: {TEXT_PRIMARY};
    border: 1px solid {ACCENT_CYAN_DIM};
    padding: 2px 6px;
}}
"""


def apply_app_theme(app: QtWidgets.QApplication):
    """Apply the sci-fi palette/QSS/font to the whole application. Call once
    at startup, before showing any windows.

    Process-wide: this overwrites QApplication's own setStyleSheet()/setFont(),
    so it's only appropriate for an app that wants this exact theme applied
    everywhere (as pyqtgraph_earth_demo.py does). Embedding MapPanel/
    EntityTable/InfoPanel/etc. into an app with its own theme should NOT call
    this -- it would clobber that app's existing stylesheet/font wholesale,
    not just style these widgets. Those widgets don't require SCI_FI_QSS to
    function; an integrating app can style them with its own QSS (targeting
    their Qt class names) or leave them on whatever style it already has.
    A scoped, non-clobbering alternative for one specific widget subtree
    exists in overlay_panel.apply_fusion_style(), which recurses over just
    that widget's descendants instead of touching QApplication."""
    app.setStyleSheet(SCI_FI_QSS)
    app.setFont(technical_font(9))
