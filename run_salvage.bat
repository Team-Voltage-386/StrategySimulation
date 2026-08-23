@echo off
setlocal

rem Launches SALVAGE 2027, the invented dry-run game (see
rem DRY_RUN_LOG.md), in its drive-it-yourself window. An Xbox
rem controller and the keyboard both work, at the same time -- there
rem is no mode to pick and no setting to get wrong.

rem Always run from this file's own folder (the repo root), regardless
rem of what directory Explorer/double-click launched it from -- needed
rem both for the relative "apps\run_salvage.py" path below and so
rem PYTHONPATH (set relative to here) is correct.
cd /d "%~dp0"

rem Make the repo root importable (common_sim/game_specific/gui_utils
rem are plain top-level packages, not pip-installed) without requiring
rem an editable install.
set "PYTHONPATH=%~dp0;%PYTHONPATH%"

rem Finds an interpreter that can actually run this, or explains what's
rem missing and leaves PYTHON_EXE empty. See packaging\pick_python.bat.
call packaging\pick_python.bat
if not defined PYTHON_EXE (
    pause
    exit /b 1
)

echo Launching SALVAGE with "%PYTHON_EXE%" ...
"%PYTHON_EXE%" apps\run_salvage.py %*
set "EXIT_CODE=%ERRORLEVEL%"

if not "%EXIT_CODE%"=="0" (
    echo.
    echo run_salvage exited with code %EXIT_CODE%.
    pause
)

endlocal
