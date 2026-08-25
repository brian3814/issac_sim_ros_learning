@echo off
:: Shared helper for the .cmd launchers.
::
:: These exist because Windows' default PowerShell execution policy
:: (Restricted) refuses to run .ps1 files, and changing that is a machine-wide
:: setting this project has no business touching. Batch files are not subject
:: to execution policy, so these work out of the box.
::
:: Like the .ps1 versions, nothing here is exported anywhere: `setlocal` scopes
:: every variable to this script, and the Python side applies what it needs
:: inside its own process.
::
:: Usage:  call "%~dp0_common.cmd" <script-relative-path> [args...]

setlocal enabledelayedexpansion

set "PROJECT_ROOT=%~dp0.."
set "TARGET=%~1"
if "%TARGET%"=="" (
    echo [run] internal error: no script given to _common.cmd
    exit /b 1
)

:: Collect the remaining arguments, preserving quoting.
set "ARGS="
shift
:collect_args
if "%~1"=="" goto args_done
set "ARGS=!ARGS! %1"
shift
goto collect_args
:args_done

:: ISAACSIM_ROOT comes from .env, same as everything else. Lines beginning
:: with # never match the key test below, so comments take care of themselves.
set "ISAACSIM_ROOT=C:\isaacsim"
if exist "%PROJECT_ROOT%\.env" (
    for /f "usebackq eol=# tokens=1,* delims==" %%A in ("%PROJECT_ROOT%\.env") do (
        if /i "%%A"=="ISAACSIM_ROOT" set "ISAACSIM_ROOT=%%B"
    )
) else (
    echo [run] No .env found. Copy .env.example to .env first:
    echo [run]     copy .env.example .env
    exit /b 1
)

:: Strip a trailing space, which a line like "KEY=value " would otherwise keep.
if "!ISAACSIM_ROOT:~-1!"==" " set "ISAACSIM_ROOT=!ISAACSIM_ROOT:~0,-1!"

set "PY=!ISAACSIM_ROOT!\python.bat"
if not exist "!PY!" (
    echo [run] Isaac Sim python.bat not found at "!PY!"
    echo [run] Fix ISAACSIM_ROOT in "%PROJECT_ROOT%\.env"
    exit /b 1
)

pushd "%PROJECT_ROOT%"
:: -u keeps Python's output interleaved with Kit's native logging rather than
:: block-buffered (and lost if the process dies).
call "!PY!" -u "%TARGET%"!ARGS!
set "CODE=%ERRORLEVEL%"
popd

exit /b %CODE%
