@echo off
:: Lesson 3-4: run the simulation with the ROS 2 bridge live.
::
:: Leave this running and open a second terminal for the client scripts.
::
::   run\sim.cmd
::   run\sim.cmd --headless
::   run\sim.cmd --self-test      :: verify the whole pipeline, then exit

call "%~dp0_common.cmd" "scripts\02_run_sim_ros2.py" %*
exit /b %ERRORLEVEL%
