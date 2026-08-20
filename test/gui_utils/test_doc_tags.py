"""The documentation-tag mechanism the user guide is generated from.

These are cheap tests of a cheap mechanism, and they earn their place
because the failure mode is silent: a tag that does not get collected
does not raise anything, it just quietly leaves a control out of the
guide -- and nobody notices until a student asks what a button does.
"""
from __future__ import annotations

import os

import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from pyqtgraph.Qt import QtWidgets                     # noqa: E402

from gui_utils.doc_tags import collect_callouts, document, tag_of, tags_for   # noqa: E402


@pytest.fixture(scope="module")
def app():
    return QtWidgets.QApplication.instance() or QtWidgets.QApplication([])


def test_document_returns_the_widget_so_it_can_wrap_a_construction(app):
    """The whole point of returning it: `form.addRow(document(spin, ...))`
    stays one statement, which is what keeps tagging cheap enough to do."""
    spin = QtWidgets.QSpinBox()
    assert document(spin, "k", "Title", "Body") is spin


def test_the_body_becomes_the_tooltip(app):
    widget = document(QtWidgets.QPushButton(), "k", "Title", "What it does")
    assert widget.toolTip() == "What it does"


def test_an_untagged_widget_has_no_tag(app):
    assert tag_of(QtWidgets.QPushButton()) is None


def test_tags_come_back_in_declaration_order_not_tree_order(app):
    """`findChildren` returns object-tree order, which is not the order a
    person reads a panel in. Here the second-declared widget is added to
    the layout first, so tree order and declaration order disagree."""
    root = QtWidgets.QWidget()
    layout = QtWidgets.QVBoxLayout(root)
    first = document(QtWidgets.QPushButton(), "first", "First", "b")
    second = document(QtWidgets.QPushButton(), "second", "Second", "b")
    layout.addWidget(second)
    layout.addWidget(first)

    assert [tag_of(w).key for w in tags_for(root)] == ["first", "second"]


def test_a_tagged_root_includes_itself(app):
    root = document(QtWidgets.QWidget(), "root", "Root", "b")
    QtWidgets.QVBoxLayout(root).addWidget(document(QtWidgets.QPushButton(), "child", "Child", "b"))
    assert {tag_of(w).key for w in tags_for(root)} == {"root", "child"}


def test_callouts_are_numbered_from_one_in_reading_order(app):
    root = QtWidgets.QWidget()
    layout = QtWidgets.QVBoxLayout(root)
    for key in ("a", "b", "c"):
        layout.addWidget(document(QtWidgets.QPushButton(), key, key.upper(), "b"))
    root.show()

    callouts = collect_callouts(root)
    assert [(c.number, c.key) for c in callouts] == [(1, "a"), (2, "b"), (3, "c")]


def test_a_hidden_control_is_left_off_the_screenshot(app):
    """A warning label that is not currently showing must not get a
    numbered circle drawn over empty space."""
    root = QtWidgets.QWidget()
    layout = QtWidgets.QVBoxLayout(root)
    layout.addWidget(document(QtWidgets.QPushButton(), "shown", "Shown", "b"))
    hidden = document(QtWidgets.QPushButton(), "hidden", "Hidden", "b")
    layout.addWidget(hidden)
    hidden.setVisible(False)
    root.show()

    assert [c.key for c in collect_callouts(root)] == ["shown"]
    # ...but it is still a documented control, so the guide can describe it.
    assert "hidden" in {tag_of(w).key for w in tags_for(root)}


def test_rects_are_in_the_roots_coordinates_not_the_parents(app):
    """The annotator paints onto a grab of `root`, so a rect measured
    against an intermediate container lands in the wrong place."""
    root = QtWidgets.QWidget()
    outer = QtWidgets.QVBoxLayout(root)
    outer.setContentsMargins(40, 60, 0, 0)
    inner = QtWidgets.QWidget()
    QtWidgets.QVBoxLayout(inner).setContentsMargins(0, 0, 0, 0)
    button = document(QtWidgets.QPushButton(), "b", "B", "b")
    inner.layout().addWidget(button)
    outer.addWidget(inner)
    root.resize(400, 300)
    root.show()

    rect = collect_callouts(root)[0].rect
    assert rect.left() >= 40 and rect.top() >= 60
    assert rect.size() == button.size()


def test_detail_is_optional_and_text_joins_the_two_paragraphs(app):
    plain = tag_of(document(QtWidgets.QPushButton(), "k", "T", "Body"))
    assert plain.detail == ""
    assert plain.text == "Body"

    full = tag_of(document(QtWidgets.QPushButton(), "k", "T", "Body", "More"))
    assert full.text == "Body\n\nMore"
