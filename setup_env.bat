@echo off
REM SmartTrafficLights - create the Python virtual environment on Windows.
REM Double-click this file, or run it from a terminal in the project folder.
REM Picks the newest installed Python between 3.13 and 3.10 (pygame has no 3.14 wheel yet).
setlocal
cd /d "%~dp0"

set "PYCMD="
for %%V in (3.13 3.12 3.11 3.10) do (
    if not defined PYCMD (
        py -%%V -c "import sys" >nul 2>&1 && set "PYCMD=py -%%V"
    )
)
if not defined PYCMD (
    python -c "import sys; sys.exit(0 if (3,10)<=sys.version_info[:2]<=(3,13) else 1)" >nul 2>&1 && set "PYCMD=python"
)
if not defined PYCMD (
    echo [ERROR] No suitable Python found. Install Python 3.13 from https://www.python.org/downloads/
    echo         and tick "Add python.exe to PATH" during install. Then run this file again.
    goto :end
)

echo Using: %PYCMD%
%PYCMD% --version

if not exist ".venv\Scripts\python.exe" (
    echo Creating virtual environment in .venv ...
    %PYCMD% -m venv .venv || (echo [ERROR] venv creation failed & goto :end)
) else (
    echo .venv already exists, reusing it.
)

echo Installing dependencies ...
".venv\Scripts\python.exe" -m pip install --upgrade pip || goto :fail
".venv\Scripts\python.exe" -m pip install -r requirements.txt || goto :fail

echo.
echo Verifying environment ...
".venv\Scripts\python.exe" check_env.py || goto :fail
".venv\Scripts\python.exe" -m pytest -q tests || goto :fail

echo.
echo ===== SETUP OK =====
echo Activate later with:  .venv\Scripts\activate
goto :end

:fail
echo.
echo ===== SETUP FAILED - copy the output above and send it to Claude =====

:end
echo.
pause
endlocal
