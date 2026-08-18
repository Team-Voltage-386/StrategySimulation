"""PyInstaller entry point: same app as `run.bat`, wrapped so PyInstaller
sees a single script instead of the `apps.run_reefscape` module launched
via -m. Keeps the repo root on sys.path so the top-level `apps`,
`common_sim`, `game_specific`, and `gui_utils` packages import the same
way they do in the dev workflow.
"""
from __future__ import annotations

import os
import sys

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if REPO_ROOT not in sys.path:
    sys.path.insert(0, REPO_ROOT)

from apps.run_reefscape import main

if __name__ == "__main__":
    main()
