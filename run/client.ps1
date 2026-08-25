# Run a ROS 2 client script against the running simulation.
#
#   .\run\client.ps1 monitor         # what is published, and how fast
#   .\run\client.ps1 teleop          # drive it yourself with WASD
#   .\run\client.ps1 patrol          # autonomous patrol with lidar stopping
#   .\run\client.ps1 ros2 topic list # the bundled ros2 CLI
#
# These need no ROS 2 installation: Isaac Sim ships rclpy and the CLI, and
# the scripts add them to their own sys.path at start-up.

. (Join-Path $PSScriptRoot '_common.ps1')

$map = @{
    'monitor' = 'ros2_client\topic_monitor.py'
    'teleop'  = 'ros2_client\teleop_key.py'
    'patrol'  = 'ros2_client\patrol.py'
    'ros2'    = 'ros2_client\ros2cli.py'
}

if ($args.Count -lt 1) {
    Write-Host 'usage: .\run\client.ps1 <monitor|teleop|patrol|ros2> [args...]'
    exit 1
}

$name = $args[0]
if (-not $map.ContainsKey($name)) {
    Write-Host "unknown client '$name'. options: $($map.Keys -join ', ')"
    exit 1
}

$rest = @()
if ($args.Count -gt 1) { $rest = $args[1..($args.Count - 1)] }
Invoke-IsaacPython -Script $map[$name] -Arguments $rest
exit $LASTEXITCODE
