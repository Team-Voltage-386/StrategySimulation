"""Checks that the interpreter running this script can actually start
sparky-sim, and says something useful when it can't.

This exists because the failure it replaces was silent. The launchers
fall back to a bare `python` on PATH, which on Windows is very often the
Microsoft Store stub: it exits immediately, prints nothing, and takes
the console window with it, so a double-clicked run.bat looks like
nothing happened at all. The next-most-common failure -- a real
interpreter that simply hasn't had `pip install -r requirements.txt` run
against it -- is a traceback that scrolls past before anyone reads it.

Run standalone (`python packaging/preflight.py`) to get a readable
report; the launchers run it quietly first and only show the output when
it fails, so a working setup stays silent.

Exit codes: 0 = ready, 1 = something is missing.
"""
from __future__ import annotations

import os
import sys

# Distribution name (what requirements.txt calls it, and what you'd pip
# install) -> module name (what you'd import). Only the ones that differ
# need an entry; anything else is assumed to import under its own name.
IMPORT_NAMES = {
    "PyQt5": "PyQt5",
    "pytest": "pytest",
    "pyntcore": "ntcore",
    "websocket-client": "websocket",
}

# Needed to run a match and draw a field. pytest is in requirements.txt
# for the test suite, not for the app, so a missing one shouldn't stop
# anybody playing REEFSCAPE. pyntcore and websocket-client are only needed
# by the mechanism sandbox (apps/run_mechanism_view.py,
# apps/run_mechanism_sandbox.py), not the REEFSCAPE viewer.
NOT_NEEDED_TO_RUN = {"pytest", "pyntcore", "websocket-client"}

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def _requirements() -> list[str]:
    """Distribution names from requirements.txt, so this check can't
    drift out of sync with the thing it's checking."""
    path = os.path.join(REPO_ROOT, "requirements.txt")
    try:
        with open(path, encoding="utf-8") as handle:
            lines = handle.read().splitlines()
    except OSError:
        return []

    names = []
    for line in lines:
        line = line.strip()
        if not line or line.startswith("#"):
            continue
        # Strip any version pin -- we only care whether it imports.
        for separator in ("==", ">=", "<=", "~=", ">", "<", "["):
            line = line.split(separator)[0]
        if line:
            names.append(line.strip())
    return names


def _missing(names: list[str]) -> list[str]:
    import importlib.util

    missing = []
    for dist in names:
        module = IMPORT_NAMES.get(dist, dist)
        try:
            found = importlib.util.find_spec(module) is not None
        except (ImportError, ValueError):
            found = False
        if not found:
            missing.append(dist)
    return missing


def _is_store_stub() -> bool:
    """The Microsoft Store's python.exe shim. It reports a version and a
    path like any other interpreter, so the giveaway is where it lives."""
    executable = (sys.executable or "").lower()
    return "windowsapps" in executable


def main() -> int:
    names = _requirements()
    if not names:
        print("preflight: could not read requirements.txt next to this script.")
        print(f"  expected at: {os.path.join(REPO_ROOT, 'requirements.txt')}")
        return 1

    missing = _missing(names)
    blocking = [name for name in missing if name not in NOT_NEEDED_TO_RUN]

    version = ".".join(str(part) for part in sys.version_info[:3])
    if not blocking:
        print(f"preflight: ready -- Python {version} at {sys.executable}")
        optional = [name for name in missing if name in NOT_NEEDED_TO_RUN]
        if optional:
            print(f"  (not installed, only needed for the test suite: {', '.join(optional)})")
        return 0

    print()
    print("sparky-sim can't start with this Python.")
    print()
    print(f"  Interpreter : {sys.executable}")
    print(f"  Version     : {version}")
    print(f"  Missing     : {', '.join(blocking)}")
    print()
    if _is_store_stub():
        print("  This is the Microsoft Store's placeholder python.exe, which has")
        print("  none of the packages this app needs and cannot usefully get them.")
        print("  Install a real Python (python.org or Anaconda), then reopen this")
        print("  window so it picks up the new interpreter.")
    else:
        print("  Install what's missing with:")
        print()
        print(f'      "{sys.executable}" -m pip install -r requirements.txt')
    print()
    return 1


if __name__ == "__main__":
    sys.exit(main())
