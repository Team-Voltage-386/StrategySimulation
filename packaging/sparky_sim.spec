# PyInstaller spec for the REEFSCAPE viewer (apps/run_reefscape.py).
# Build with:  packaging\build_release.bat
# Produces a onedir build under dist/SparkySim -- zip that folder to hand
# to teammates; they need no Python install, just Windows + unzip + run.
import os

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(SPEC)))

block_cipher = None

a = Analysis(
    [os.path.join(REPO_ROOT, "packaging", "launch_reefscape.py")],
    pathex=[REPO_ROOT],
    binaries=[],
    datas=[
        (
            os.path.join(REPO_ROOT, "game_specific", "reefscape", "strategies"),
            os.path.join("game_specific", "reefscape", "strategies"),
        ),
    ],
    hiddenimports=[],
    hookspath=[],
    hooksconfig={},
    runtime_hooks=[],
    # This build machine has both PyQt5 and PySide6 installed (pyqtgraph
    # supports either); the app itself imports PyQt5 only, so exclude
    # PySide6 or PyInstaller aborts on conflicting Qt bindings.
    excludes=["PySide6"],
    noarchive=False,
    cipher=block_cipher,
)
pyz = PYZ(a.pure, a.zipped_data, cipher=block_cipher)

exe = EXE(
    pyz,
    a.scripts,
    [],
    exclude_binaries=True,
    name="SparkySim",
    debug=False,
    bootloader_ignore_signals=False,
    strip=False,
    upx=False,
    console=True,
    disable_windowed_traceback=False,
    argv_emulation=False,
    target_arch=None,
    codesign_identity=None,
    entitlements_file=None,
)
coll = COLLECT(
    exe,
    a.binaries,
    a.zipfiles,
    a.datas,
    strip=False,
    upx=False,
    upx_exclude=[],
    name="SparkySim",
)
