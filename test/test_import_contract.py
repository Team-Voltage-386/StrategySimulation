"""
Enforces ARCHITECTURE.md's "Guiding constraint" -- the import direction
that the two-day new-game turnaround rests on:

    game_specific/*  --> imports from --> common_sim/*, gui_utils/*
    common_sim/*     --> never imports from --> game_specific/*
    gui_utils/*      --> never imports from --> game_specific/*

This was true when the tests were written and had never been checked by
anything. A contract nobody enforces holds right up until the first
afternoon somebody is in a hurry -- which, for this repo, is the
afternoon of game reveal, when the cost of finding out is highest.

Two mechanisms, because neither alone is enough:

* The static scan reads import statements without executing anything.
  That's what lets it cover `gui_utils`, whose modules pull in PyQt5 and
  would otherwise need a Qt install and a display policy just to be
  checked. It sees what a module *declares*.
* The runtime scan imports every `common_sim` module for real and looks
  at what actually landed in `sys.modules`. That catches an indirect
  leak -- `common_sim` importing a third module that imports
  `game_specific` -- which no amount of reading `common_sim`'s own
  import statements would reveal.

The runtime scan follows the pattern established by
test_reefscape_sweep.test_sweep_trial_imports_no_qt: assert inside a
subprocess so the check sees a clean interpreter, not one this test
session has already polluted by importing half the repo.
"""
from __future__ import annotations

import ast
import subprocess
import sys
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[1]

# The packages that must not reach into a game, per the constraint above.
GAME_AGNOSTIC_PACKAGES = ("common_sim", "gui_utils")

# What they must not import, and why -- the reason is printed on failure,
# because "you broke the contract" is only useful alongside "and here is the
# shape of the fix".
FORBIDDEN_ROOTS = {
    "game_specific": (
        "If it needs to branch on 'which game is this', the abstraction is wrong: "
        "the branch belongs in a game_specific subclass."
    ),
    # Same direction, one layer out. The maple-sim bridge adapts the simulator
    # onto real robot code and depends on a JVM, a robot project, and pyntcore.
    # A dependency the other way would make running a headless sweep require
    # all three -- and would break the spawn workers, which have none of them.
    "bridge": (
        "The bridge depends on the simulator, never the reverse. If the simulator "
        "needs something the bridge has, that thing belongs in common_sim and the "
        "bridge should import it from there."
    ),
}


def _imported_module_names(source: str) -> set[str]:
    """Every module name `source` imports, absolute form only.

    Relative imports (`from . import x`) are skipped rather than
    resolved: a relative import cannot escape its own package, so it can
    never reach `game_specific` from inside `common_sim`, and resolving
    them would mean reimplementing import machinery for no added
    coverage.
    """
    names: set[str] = set()
    for node in ast.walk(ast.parse(source)):
        if isinstance(node, ast.Import):
            names.update(alias.name for alias in node.names)
        elif isinstance(node, ast.ImportFrom):
            if node.level == 0 and node.module:
                names.add(node.module)
    return names


def _python_files(package: str) -> list[Path]:
    return sorted(p for p in (REPO_ROOT / package).rglob("*.py") if "__pycache__" not in p.parts)


@pytest.mark.parametrize("package", GAME_AGNOSTIC_PACKAGES)
@pytest.mark.parametrize("forbidden", sorted(FORBIDDEN_ROOTS))
def test_no_declared_forbidden_imports(package: str, forbidden: str) -> None:
    """No module under `package` names `forbidden` in an import."""
    files = _python_files(package)
    assert files, f"found no Python files under {package}/ -- has the layout moved?"

    offenders = []
    for path in files:
        imported = _imported_module_names(path.read_text(encoding="utf-8"))
        leaks = sorted(
            name for name in imported
            if name == forbidden or name.startswith(forbidden + ".")
        )
        if leaks:
            offenders.append(f"{path.relative_to(REPO_ROOT).as_posix()} imports {', '.join(leaks)}")

    assert not offenders, (
        "ARCHITECTURE.md's import contract is broken -- "
        f"{package}/ must not depend on {forbidden}/:\n  " + "\n  ".join(offenders)
        + "\n\n" + FORBIDDEN_ROOTS[forbidden]
    )


# Imports every common_sim module for real, then reports what came along
# for the ride. `game_specific` is the contract; Qt is checked in the
# same pass because common_sim is documented Qt-free (see
# analysis/runner.py, "no Qt, no game_specific") and it is the same
# class of mistake -- a headless path quietly acquiring a GUI dependency
# breaks sweeps in a spawn worker, where there is no display at all.
_RUNTIME_SCAN = """
import importlib
import pkgutil
import sys

import common_sim

for info in pkgutil.walk_packages(common_sim.__path__, prefix="common_sim."):
    importlib.import_module(info.name)

leaked_game = sorted(
    m for m in sys.modules if m == "game_specific" or m.startswith("game_specific.")
)
leaked_qt = sorted(m for m in sys.modules if "PyQt" in m or "pyqtgraph" in m)
# The bridge and the WPILib bindings it needs. Catching `ntcore` directly as
# well as `bridge` covers the case where common_sim reaches for NetworkTables
# without going through the bridge at all -- same broken dependency, and it
# would fail on any machine that has not installed bridge/requirements.txt.
leaked_bridge = sorted(
    m for m in sys.modules
    if m == "bridge" or m.startswith("bridge.") or m in ("ntcore", "websocket")
)

assert not leaked_game, "common_sim pulled in game_specific: %s" % leaked_game
assert not leaked_qt, "common_sim pulled in Qt: %s" % leaked_qt
assert not leaked_bridge, "common_sim pulled in the bridge: %s" % leaked_bridge
print("ok")
"""


def test_common_sim_imports_nothing_game_specific_at_runtime() -> None:
    result = subprocess.run(
        [sys.executable, "-c", _RUNTIME_SCAN],
        capture_output=True, text=True, cwd=str(REPO_ROOT),
    )
    assert result.returncode == 0, result.stdout + result.stderr
    assert "ok" in result.stdout
