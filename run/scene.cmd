@echo off
:: Lesson 1-2: build the warehouse + robot and watch the physics settle.
::
::   run\scene.cmd
::   run\scene.cmd --spin --save
::   run\scene.cmd --headless --seconds 4

call "%~dp0_common.cmd" "scripts\01_build_scene.py" %*
exit /b %ERRORLEVEL%
