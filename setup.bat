@echo off
setlocal EnableDelayedExpansion

rem One-time setup for a new machine: finds a real Python, builds a
rem private virtual environment in .venv\, and installs requirements.txt
rem into it. After this, run.bat and the other launchers pick up .venv
rem automatically (see packaging\pick_python.bat) -- nothing to activate.
rem
rem Safe to re-run at any time: it reuses a working .venv, rebuilds a
rem broken one, and reinstalls to match requirements.txt. Re-run it after
rem pulling a change to requirements.txt.

cd /d "%~dp0"

set "VENV_DIR=%~dp0.venv"
set "VENV_PY=%VENV_DIR%\Scripts\python.exe"
set "WANT_VERSION=3.13"

echo.
echo === sparky-sim setup ===
echo.

rem ---------------------------------------------------------------------
rem 1. Find a base interpreter to build the venv from.
rem
rem 3.13 first (what the sim is developed and pinned against), then
rem older-but-supported versions. Each candidate has to actually start --
rem that is what skips the Microsoft Store placeholder python.exe, which
rem exits immediately and prints nothing.
rem ---------------------------------------------------------------------
set "BASE_PY="
set "BASE_ARGS="

call :try_base "py" "-3.13"
call :try_base "%LOCALAPPDATA%\Programs\Python\Python313\python.exe" ""
call :try_base "%ProgramFiles%\Python313\python.exe" ""
call :try_base "python" ""
call :try_base "py" "-3.12"
call :try_base "%LOCALAPPDATA%\Programs\Python\Python312\python.exe" ""
call :try_base "py" "-3.11"
call :try_base "%LOCALAPPDATA%\Programs\Python\Python311\python.exe" ""
call :try_base "py" ""
call :try_base "%USERPROFILE%\anaconda3_2025\python.exe" ""
call :try_base "%USERPROFILE%\anaconda3\python.exe" ""

if not defined BASE_PY goto :no_python

rem Via a temp file rather than for /f: cmd mangles a for /f command that
rem has both a quoted executable and a quoted argument.
set "INFO_FILE=%TEMP%\sparky_setup_python.txt"
"%BASE_PY%" %BASE_ARGS% -c "import platform, sys; print(platform.python_version()); print(sys.executable)" > "%INFO_FILE%"
set /p BASE_VERSION=< "%INFO_FILE%"
for /f "usebackq skip=1 delims=" %%p in ("%INFO_FILE%") do set "BASE_EXE=%%p"
del "%INFO_FILE%" >nul 2>&1
echo Found Python %BASE_VERSION% at:
echo     %BASE_EXE%
echo %BASE_VERSION% | findstr /b /c:"%WANT_VERSION%." >nul
if errorlevel 1 (
    echo.
    echo   Note: the sim is developed on Python %WANT_VERSION%. This version should
    echo   work, but if anything behaves differently from a teammate's machine,
    echo   install %WANT_VERSION% from python.org and re-run setup.bat.
)
echo.

rem ---------------------------------------------------------------------
rem 2. Create .venv, or rebuild it if it no longer starts (e.g. the Python
rem    it was made from has since been uninstalled or upgraded).
rem ---------------------------------------------------------------------
if exist "%VENV_PY%" (
    "%VENV_PY%" -c "import sys" >nul 2>&1
    if errorlevel 1 (
        echo Existing .venv is broken -- rebuilding it.
        rmdir /s /q "%VENV_DIR%"
    ) else (
        echo Reusing existing .venv
    )
)

if not exist "%VENV_PY%" (
    echo Creating virtual environment in .venv ...
    "%BASE_PY%" %BASE_ARGS% -m venv "%VENV_DIR%"
    if errorlevel 1 goto :venv_failed
)

rem ---------------------------------------------------------------------
rem 3. Install the pinned requirements.
rem ---------------------------------------------------------------------
echo.
echo Installing packages (first run downloads ~150 MB; this can take a
echo few minutes) ...
echo.
"%VENV_PY%" -m pip install --upgrade pip --disable-pip-version-check --quiet
if errorlevel 1 goto :pip_failed
"%VENV_PY%" -m pip install -r requirements.txt --disable-pip-version-check
if errorlevel 1 goto :pip_failed

rem ---------------------------------------------------------------------
rem 4. Prove it: the same check the launchers run before starting.
rem ---------------------------------------------------------------------
echo.
"%VENV_PY%" packaging\preflight.py
if errorlevel 1 goto :pip_failed

echo.
echo === Setup complete ===
echo.
echo   Double-click run.bat to start the REEFSCAPE viewer.
echo   To run the test suite:  .venv\Scripts\python -m pytest -q
echo.
pause
exit /b 0

rem =====================================================================

:try_base
rem %1 = executable, %2 = extra argument (e.g. -3.13 for the py launcher).
if defined BASE_PY goto :eof
set "CAND=%~1"
set "CAND_ARGS=%~2"
rem Absolute path that isn't there: skip without launching anything.
echo !CAND! | findstr /c:"\" >nul
if not errorlevel 1 if not exist "!CAND!" goto :eof
rem Must start, must not be the Store stub, and must be 3.11 or newer.
"!CAND!" !CAND_ARGS! -c "import sys; sys.exit(0 if sys.version_info >= (3, 11) and 'windowsapps' not in sys.executable.lower() else 1)" >nul 2>&1
if errorlevel 1 goto :eof
set "BASE_PY=!CAND!"
set "BASE_ARGS=!CAND_ARGS!"
goto :eof

:no_python
echo No usable Python was found on this machine.
echo.
echo   1. Install Python %WANT_VERSION% from https://www.python.org/downloads/
echo      (the "Windows installer (64-bit)"). On the first installer screen,
echo      tick "Add python.exe to PATH".
echo   2. Close this window and double-click setup.bat again.
echo.
echo If typing "python" in a terminal opens the Microsoft Store, that is a
echo Windows placeholder, not a real Python -- installing from python.org
echo fixes it.
echo.
pause
exit /b 1

:venv_failed
echo.
echo Could not create the virtual environment in .venv
echo Try deleting the .venv folder and running setup.bat again. If it still
echo fails, reinstall Python %WANT_VERSION% from python.org.
echo.
pause
exit /b 1

:pip_failed
echo.
echo Package installation failed -- see the messages above.
echo.
echo   Most common causes:
echo   * No internet connection, or a network that blocks pypi.org
echo     (school networks sometimes do -- try from home or a phone hotspot).
echo   * A Python version too new for the pinned packages to have
echo     prebuilt downloads yet ("building wheel" / "Microsoft Visual C++
echo     is required" errors). Install Python %WANT_VERSION% and re-run.
echo.
pause
exit /b 1
