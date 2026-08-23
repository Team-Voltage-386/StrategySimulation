@echo off
setlocal

rem Always run from this file's own folder (the repo root), regardless
rem of what directory Explorer/double-click launched it from -- needed
rem both for the relative "benchmarks\bench.py" path below and so
rem PYTHONPATH (set relative to here) is correct.
cd /d "%~dp0"

rem Make the repo root importable (common_sim/game_specific/gui_utils
rem are plain top-level packages, not pip-installed) without requiring
rem an editable install.
set "PYTHONPATH=%~dp0;%PYTHONPATH%"

rem Finds an interpreter that can actually run this, or explains what's
rem missing and leaves PYTHON_EXE empty. See packaging\pick_python.bat.
call packaging\pick_python.bat
if not defined PYTHON_EXE exit /b 1

"%PYTHON_EXE%" benchmarks\bench.py %*
set "EXIT_CODE=%ERRORLEVEL%"

if not "%EXIT_CODE%"=="0" (
    echo.
    echo run_bench exited with code %EXIT_CODE%.
)

endlocal
