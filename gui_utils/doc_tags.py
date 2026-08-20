"""
Documentation tags: attach a plain-language explanation to a widget once,
and get a tooltip, a user-guide entry, and a numbered screenshot callout
out of it.

The problem this solves is drift. A tooltip lives in the widget code, the
user guide lives in a doc, and the screenshots in that doc are pictures
of whatever the GUI looked like the afternoon someone took them. Three
copies of the same explanation, and only the first one is ever updated.
Here there is one copy: `document(widget, ...)` is the only place a
control is described, and `apps/build_guide.py` reads the tags back out
to draw the callouts *and* to write the reference table beneath them --
so a control that gets renamed, moved, or removed cannot leave a stale
paragraph behind. Rebuild the guide and it is right again.

Order is declaration order, not tree order. `findChildren` returns
whatever the object hierarchy happens to hold, which is not the order a
person reads a panel in; the module-level counter means callout 1 is
whichever control the panel's author described first.

Written against pyqtgraph.Qt so it follows whatever binding the rest of
the app resolved to, and it imports nothing else -- a tag is data hanging
off a QWidget.
"""
from __future__ import annotations

import itertools
from dataclasses import dataclass

from pyqtgraph.Qt import QtCore, QtWidgets

_ATTRIBUTE = "_sparky_doc_tag"
_counter = itertools.count(1)


@dataclass(frozen=True)
class DocTag:
    """One documented control.

    `key` is the stable identifier the guide anchors on -- rename a label
    freely, but changing a key breaks any link written against it.
    `title` is the callout's short name; `body` is the sentence a
    fourteen-year-old should be able to act on. `detail` is the optional
    second paragraph that belongs in the guide but would make a tooltip
    unreadable.
    """
    key: str
    title: str
    body: str
    detail: str = ""
    order: int = 0

    @property
    def text(self) -> str:
        return self.body if not self.detail else f"{self.body}\n\n{self.detail}"


def document(widget, key: str, title: str, body: str, detail: str = ""):
    """Tag `widget`, set its tooltip from `body`, and return it.

    Returns the widget so it can wrap a construction expression in place
    (`form.addRow(document(QSpinBox(), ...))`) rather than needing a
    separate statement, which is what keeps tagging cheap enough that it
    actually gets done.
    """
    tag = DocTag(key=key, title=title, body=body, detail=detail, order=next(_counter))
    setattr(widget, _ATTRIBUTE, tag)
    widget.setToolTip(body)
    return widget


def tag_of(widget) -> DocTag | None:
    return getattr(widget, _ATTRIBUTE, None)


def tags_for(root) -> list:
    """Every tagged widget under `root`, in declaration order.

    Includes `root` itself if it is tagged. Widgets that are hidden or
    have never been laid out are still returned -- filtering on
    visibility is `collect_callouts`' job, since a tag with no geometry
    is still a valid guide entry.
    """
    found = []
    if tag_of(root) is not None:
        found.append(root)
    found.extend(w for w in root.findChildren(QtWidgets.QWidget) if tag_of(w) is not None)
    return sorted(found, key=lambda w: tag_of(w).order)


@dataclass(frozen=True)
class Callout:
    """A tag plus where it landed, in `root`'s coordinates."""
    number: int
    tag: DocTag
    rect: object          # QRect, or None when the widget is not visible

    @property
    def key(self) -> str:
        return self.tag.key


def collect_callouts(root, *, visible_only: bool = True) -> list:
    """Number the tagged widgets under `root` and locate each one.

    `root` must already have been laid out (shown, or `adjustSize()`d) or
    every rect is the 640x480 default Qt hands an unlaid-out widget --
    which produces a screenshot with all the callouts stacked in one
    corner and no error anywhere.
    """
    callouts = []
    for widget in tags_for(root):
        if visible_only and not widget.isVisibleTo(root):
            continue
        top_left = widget.mapTo(root, QtCore.QPoint(0, 0))
        rect = QtCore.QRect(top_left, widget.size())
        callouts.append(Callout(number=len(callouts) + 1, tag=tag_of(widget), rect=rect))
    return callouts
