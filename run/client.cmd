@echo off
:: Run a ROS 2 client script against the running simulation.
::
::   run\client.cmd monitor          :: what is published, and how fast
::   run\client.cmd teleop           :: drive it yourself with WASD
::   run\client.cmd patrol           :: autonomous patrol with lidar stopping
::   run\client.cmd ros2 topic list  :: the bundled ros2 CLI
::
:: These need no ROS 2 installation: Isaac Sim ships rclpy and the CLI, and
:: the scripts add them to their own sys.path at start-up.

setlocal enabledelayedexpansion

:: Capture this script's directory before any shift. `shift` moves %0 along
:: with everything else, so %~dp0 below the arg loop would point at an
:: argument rather than at this file.
set "HERE=%~dp0"

set "NAME=%~1"
if "%NAME%"=="" goto usage

set "SCRIPT="
if /i "%NAME%"=="monitor" set "SCRIPT=ros2_client\topic_monitor.py"
if /i "%NAME%"=="teleop"  set "SCRIPT=ros2_client\teleop_key.py"
if /i "%NAME%"=="patrol"  set "SCRIPT=ros2_client\patrol.py"
if /i "%NAME%"=="ros2"    set "SCRIPT=ros2_client\ros2cli.py"

if "!SCRIPT!"=="" (
    echo unknown client '%NAME%'
    goto usage
)

:: Drop the client name and forward the rest.
set "ARGS="
shift
:collect
if "%~1"=="" goto run
set "ARGS=!ARGS! %1"
shift
goto collect

:run
call "!HERE!_common.cmd" "!SCRIPT!"!ARGS!
exit /b %ERRORLEVEL%

:usage
echo usage: run\client.cmd ^<monitor^|teleop^|patrol^|ros2^> [args...]
exit /b 1
