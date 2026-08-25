# Lesson 1-2: build the warehouse + robot and watch the physics settle.
#
#   .\run\scene.ps1
#   .\run\scene.ps1 --spin --save
#   .\run\scene.ps1 --headless --seconds 4

. (Join-Path $PSScriptRoot '_common.ps1')
Invoke-IsaacPython -Script 'scripts\01_build_scene.py' -Arguments $args
exit $LASTEXITCODE
