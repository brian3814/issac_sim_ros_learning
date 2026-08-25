# Lesson 3-4: run the simulation with the ROS 2 bridge live.
#
# Leave this running and open a second terminal for the client scripts.
#
#   .\run\sim.ps1
#   .\run\sim.ps1 --headless
#   .\run\sim.ps1 --self-test      # verify the whole pipeline, then exit

. (Join-Path $PSScriptRoot '_common.ps1')
Invoke-IsaacPython -Script 'scripts\02_run_sim_ros2.py' -Arguments $args
exit $LASTEXITCODE
