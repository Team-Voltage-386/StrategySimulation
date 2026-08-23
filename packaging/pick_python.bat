@echo off
rem Picks an interpreter that can actually run sparky-sim and leaves it
rem in PYTHON_EXE (empty if there isn't one). Deliberately has no
rem setlocal -- the caller needs the variable back.
rem
rem Every launcher used to carry its own copy of this, hardcoded to one
rem machine's Anaconda path and falling through to a bare `python` on
rem PATH. On a machine without that exact Anaconda, `python` is very
rem often the Microsoft Store stub, which exits immediately with no
rem output -- so a double-clicked launcher looked like nothing happened.
rem
rem Each candidate has to both start AND pass preflight.py, so an
rem interpreter that runs but has no pymunk/PyQt5 is skipped in favour of
rem one further down the list rather than being launched into a
rem traceback.

set "PYTHON_EXE="
set "PY_FIRST_RUNNABLE="

call :try "%USERPROFILE%\anaconda3_2025\python.exe"
call :try "%USERPROFILE%\anaconda3\python.exe"
call :try "%USERPROFILE%\miniconda3\python.exe"
call :try "%LOCALAPPDATA%\Programs\Python\Python313\python.exe"
call :try "%LOCALAPPDATA%\Programs\Python\Python312\python.exe"
call :try "%LOCALAPPDATA%\Programs\Python\Python311\python.exe"
call :try "C:\Python313\python.exe"
call :try "C:\Python312\python.exe"
call :try "py"
call :try "python"
goto :report

:try
rem Already settled, or this candidate is an absolute path that isn't
rem there -- skip without paying for a process launch.
if defined PYTHON_EXE goto :eof
set "PY_CAND=%~1"
echo %PY_CAND% | findstr /c:"\" >nul
if not errorlevel 1 if not exist "%PY_CAND%" goto :eof

rem Does it start at all? This is what weeds out the Store stub.
"%PY_CAND%" -c "import sys" >nul 2>&1
if errorlevel 1 goto :eof
if not defined PY_FIRST_RUNNABLE set "PY_FIRST_RUNNABLE=%PY_CAND%"

rem Does it have the packages? Quiet on success, so a working setup
rem prints nothing.
"%PY_CAND%" "%~dp0preflight.py" >nul 2>&1
if errorlevel 1 goto :eof

set "PYTHON_EXE=%PY_CAND%"
goto :eof

:report
if defined PYTHON_EXE goto :eof

if defined PY_FIRST_RUNNABLE (
    rem Found a real interpreter, but it's missing packages -- let
    rem preflight say which ones, and how to fix it, in its own words.
    "%PY_FIRST_RUNNABLE%" "%~dp0preflight.py"
    goto :eof
)

echo.
echo No usable Python was found on this machine.
echo.
echo sparky-sim needs Python 3.11 or newer with the packages in
echo requirements.txt. Install one from python.org or Anaconda, tick
echo "Add Python to PATH" during setup, then reopen this window.
echo.
echo Note that the "python" that ships with Windows by default is a
echo Microsoft Store placeholder, not a real interpreter -- if typing
echo "python" opens the Store, that's what happened.
echo.
goto :eof
