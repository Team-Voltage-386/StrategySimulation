@echo off
setlocal

rem Builds a standalone Windows folder (dist\SparkySim) that teammates can
rem run with no Python install. Run this from a machine with the app's
rem dependencies installed (see ..\requirements.txt) plus pyinstaller.

cd /d "%~dp0\.."

rem Build from an isolated venv with only requirements.txt installed --
rem NOT the full Anaconda base env, which pulls Jupyter/boto3/numba/bokeh
rem etc. into the frozen app and bloats it from ~150MB to 600+MB.
set "BUILD_VENV=%~dp0..\.build_venv"
if not exist "%BUILD_VENV%\Scripts\python.exe" (
    echo Creating build venv at %BUILD_VENV% ...
    set "BASE_PYTHON=%USERPROFILE%\anaconda3_2025\python.exe"
    if not exist "%BASE_PYTHON%" set "BASE_PYTHON=python"
    "%BASE_PYTHON%" -m venv "%BUILD_VENV%" || goto :error
)
set "PYTHON_EXE=%BUILD_VENV%\Scripts\python.exe"

"%PYTHON_EXE%" -m pip show pyinstaller >nul 2>&1
if errorlevel 1 (
    echo Installing build dependencies into venv...
    "%PYTHON_EXE%" -m pip install -r requirements.txt pyinstaller -q || goto :error
)

echo Building SparkySim...
"%PYTHON_EXE%" -m PyInstaller --noconfirm --clean packaging\sparky_sim.spec || goto :error

set "VERSION_TAG="
for /f %%i in ('powershell -NoProfile -Command "Get-Date -Format yyyyMMdd_HHmmss"') do set "VERSION_TAG=%%i"

set "ZIP_NAME=SparkySim_%VERSION_TAG%.zip"
echo Zipping dist\SparkySim to dist\%ZIP_NAME% ...
powershell -NoProfile -Command "Compress-Archive -Path 'dist\SparkySim\*' -DestinationPath 'dist\%ZIP_NAME%' -Force" || goto :error

echo.
echo Done. Hand teammates dist\%ZIP_NAME% -- unzip anywhere and run SparkySim.exe.
goto :eof

:error
echo.
echo Build failed.
exit /b 1
