@echo off
setlocal

rem Always run from this file's own folder (the repo root), regardless
rem of what directory Explorer/double-click launched it from -- needed
rem both for the "-m apps.build_guide" invocation below and so
rem PYTHONPATH (set relative to here) is correct.
cd /d "%~dp0"

rem Make the repo root importable (common_sim/game_specific/gui_utils
rem are plain top-level packages, not pip-installed) without requiring
rem an editable install.
set "PYTHONPATH=%~dp0;%PYTHONPATH%"

rem Prefer this machine's known Anaconda install; fall back to the `py`
rem launcher, then plain `python` on PATH. Anaconda is checked first
rem because a bare `python`/`py` here can otherwise resolve to the
rem Microsoft Store stub, which exits immediately without the packages
rem this app needs (pymunk, pygame, PyQt5, pyqtgraph).
set "PYTHON_EXE=%USERPROFILE%\anaconda3_2025\python.exe"
if exist "%PYTHON_EXE%" goto :run

for %%P in (py.exe) do (
    if not "%%~$PATH:P"=="" (
        set "PYTHON_EXE=py"
        goto :run
    )
)

set "PYTHON_EXE=python"

:run
rem Run as a module (not a bare script path) so the offscreen-QPA guard
rem in build_guide.py (Windows registers zero font families under
rem QT_QPA_PLATFORM=offscreen -- see build_guide.py) and its relative
rem doc/ output paths resolve the same way they do from the docs on
rem how this is normally invoked. Pass through --guide <name> / --open
rem etc. via %*, e.g. `run_build_guide.bat --guide match --open`.
echo Building REEFSCAPE guide(s) with "%PYTHON_EXE%" ...
"%PYTHON_EXE%" -m apps.build_guide %*
set "EXIT_CODE=%ERRORLEVEL%"

if not "%EXIT_CODE%"=="0" (
    echo.
    echo run_build_guide exited with code %EXIT_CODE%.
    pause
)

endlocal
