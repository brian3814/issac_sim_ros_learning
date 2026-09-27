r"""The ROS 2 bridge, expressed as an OmniGraph action graph.

This is the heart of the project. Isaac Sim does not talk ROS 2 through a
Python loop -- it talks through a dataflow graph that PhysX ticks every frame.
Each node is a publisher, a subscriber, or a small computation, and the graph
is what you would otherwise hand-write as a rospy node.

The graph built here:

    OnPlaybackTick ---------------------------------------------------.
        |                                                             |
        |-> ROS2SubscribeTwist (/cmd_vel) -> Break -> DifferentialController
        |          [drive_from_graph only]               |
        |                                                v
        |                                     IsaacArticulationController --> PhysX
        |
        |-> IsaacComputeOdometry -> ROS2PublishOdometry (/odom)
        |                        \-> ROS2PublishRawTransformTree (odom -> base_link)
        |
        |-> ROS2PublishJointState (/joint_states)
        |-> ROS2PublishTransformTree (base_link -> wheels, camera, lidar)
        |-> ROS2PublishClock (/clock)
        |
        |-> IsaacCreateRenderProduct -> ROS2CameraHelper      (/rgb, /depth)
        |                            \-> ROS2CameraInfoHelper (/camera_info)
        |
        '-> ROS2RtxLidarHelper (/scan)

The /cmd_vel branch is optional. ``run\\sim.cmd`` builds the graph without it
and drives the wheels from ``base_controller.py`` instead, because that branch
has no command timeout; see ``_add_drive``.

Every ROS node in the graph is fed the same ROS2Context, which is created with
``useDomainIDEnvVar = False`` so the DDS domain comes from the project's
``.env`` and not from whatever the shell happened to export.
"""

from __future__ import annotations

import omni.graph.core as og  # type: ignore
import usdrt.Sdf  # type: ignore

GRAPH_PATH = "/World/ROS2Graph"

ODOM_FRAME = "odom"
BASE_FRAME = "base_link"
CAMERA_FRAME = "front_camera"
LIDAR_FRAME = "front_lidar"


def build_ros2_graph(cfg, robot, sensors, drive_from_graph: bool = True) -> str:
    """Create the bridge graph. Returns the graph prim path.

    ``drive_from_graph=False`` leaves out the command half of the graph
    (/cmd_vel -> wheels) because something else owns the wheels -- in
    ``run\\sim.cmd`` that is ``base_controller.py``, which adds a command
    timeout. Only one writer of wheel targets may exist, or they overwrite
    each other every frame. The GUI path (``gui.py``) keeps the graph drive.
    """
    ns = cfg.ros_namespace.strip("/")
    # Topic names given to the bridge nodes are *relative*: the node prepends
    # nodeNamespace itself, so passing "/rgb" here would defeat the namespace.
    frame_skip = max(0, cfg.camera_publish_every_n_frames - 1)

    keys = og.Controller.Keys
    spec = {
        keys.CREATE_NODES: [
            ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
            ("Context", "isaacsim.ros2.bridge.ROS2Context"),
            ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
            # --- time -------------------------------------------------
            ("PublishClock", "isaacsim.ros2.bridge.ROS2PublishClock"),
            # --- state out --------------------------------------------
            ("ComputeOdometry", "isaacsim.core.nodes.IsaacComputeOdometry"),
            ("PublishOdometry", "isaacsim.ros2.bridge.ROS2PublishOdometry"),
            ("PublishRawTF", "isaacsim.ros2.bridge.ROS2PublishRawTransformTree"),
            ("PublishTF", "isaacsim.ros2.bridge.ROS2PublishTransformTree"),
            ("PublishJointState", "isaacsim.ros2.bridge.ROS2PublishJointState"),
            # --- sensors ----------------------------------------------
            ("CreateRenderProduct", "isaacsim.core.nodes.IsaacCreateRenderProduct"),
            ("CameraHelperRgb", "isaacsim.ros2.bridge.ROS2CameraHelper"),
            ("CameraHelperDepth", "isaacsim.ros2.bridge.ROS2CameraHelper"),
            ("CameraHelperInfo", "isaacsim.ros2.bridge.ROS2CameraInfoHelper"),
            ("LidarHelper", "isaacsim.ros2.bridge.ROS2RtxLidarHelper"),
        ],
        keys.SET_VALUES: [
            # The DDS domain comes from .env, never from the environment.
            ("Context.inputs:domain_id", cfg.ros_domain_id),
            ("Context.inputs:useDomainIDEnvVar", False),

            ("PublishClock.inputs:topicName", "clock"),
            ("PublishClock.inputs:nodeNamespace", ns),

            ("ComputeOdometry.inputs:chassisPrim", [usdrt.Sdf.Path(robot.base_link)]),

            ("PublishOdometry.inputs:topicName", "odom"),
            ("PublishOdometry.inputs:nodeNamespace", ns),
            ("PublishOdometry.inputs:odomFrameId", ODOM_FRAME),
            ("PublishOdometry.inputs:chassisFrameId", BASE_FRAME),

            # odom -> base_link, the transform Nav2 expects the simulator
            # (not the robot description) to provide.
            ("PublishRawTF.inputs:topicName", "tf"),
            ("PublishRawTF.inputs:nodeNamespace", ns),
            ("PublishRawTF.inputs:parentFrameId", ODOM_FRAME),
            ("PublishRawTF.inputs:childFrameId", BASE_FRAME),

            # base_link -> everything bolted to it.
            ("PublishTF.inputs:topicName", "tf"),
            ("PublishTF.inputs:nodeNamespace", ns),
            ("PublishTF.inputs:parentPrim", [usdrt.Sdf.Path(robot.base_link)]),
            ("PublishTF.inputs:targetPrims", [
                usdrt.Sdf.Path(robot.left_wheel),
                usdrt.Sdf.Path(robot.right_wheel),
                usdrt.Sdf.Path(robot.camera),
                # The mount, not the nested OmniLidar: its prim name is
                # what becomes the TF frame id, and it must match
                # LIDAR_FRAME below or /scan lands in an unknown frame.
                usdrt.Sdf.Path(sensors.lidar_mount_prim),
            ]),

            ("PublishJointState.inputs:topicName", "joint_states"),
            ("PublishJointState.inputs:nodeNamespace", ns),
            ("PublishJointState.inputs:targetPrim", [usdrt.Sdf.Path(robot.root)]),

            ("CreateRenderProduct.inputs:cameraPrim", [usdrt.Sdf.Path(robot.camera)]),
            ("CreateRenderProduct.inputs:width", cfg.camera_width),
            ("CreateRenderProduct.inputs:height", cfg.camera_height),

            ("CameraHelperRgb.inputs:type", "rgb"),
            ("CameraHelperRgb.inputs:topicName", "rgb"),
            ("CameraHelperRgb.inputs:frameId", CAMERA_FRAME),
            ("CameraHelperRgb.inputs:nodeNamespace", ns),
            ("CameraHelperRgb.inputs:frameSkipCount", frame_skip),

            ("CameraHelperDepth.inputs:type", "depth"),
            ("CameraHelperDepth.inputs:topicName", "depth"),
            ("CameraHelperDepth.inputs:frameId", CAMERA_FRAME),
            ("CameraHelperDepth.inputs:nodeNamespace", ns),
            ("CameraHelperDepth.inputs:frameSkipCount", frame_skip),

            ("CameraHelperInfo.inputs:topicName", "camera_info"),
            ("CameraHelperInfo.inputs:frameId", CAMERA_FRAME),
            ("CameraHelperInfo.inputs:nodeNamespace", ns),
            ("CameraHelperInfo.inputs:frameSkipCount", frame_skip),

            # The lidar's render product was made in Python (sensors.py)
            # because RTX sensors need one before the helper can wrap them.
            ("LidarHelper.inputs:renderProductPath", sensors.lidar_render_product),
            ("LidarHelper.inputs:type", "laser_scan"),
            ("LidarHelper.inputs:topicName", "scan"),
            ("LidarHelper.inputs:frameId", LIDAR_FRAME),
            ("LidarHelper.inputs:nodeNamespace", ns),
        ],
        keys.CONNECT: [
            # --- context to every ROS node ---------------------------
            ("Context.outputs:context", "PublishClock.inputs:context"),
            ("Context.outputs:context", "PublishOdometry.inputs:context"),
            ("Context.outputs:context", "PublishRawTF.inputs:context"),
            ("Context.outputs:context", "PublishTF.inputs:context"),
            ("Context.outputs:context", "PublishJointState.inputs:context"),
            ("Context.outputs:context", "CameraHelperRgb.inputs:context"),
            ("Context.outputs:context", "CameraHelperDepth.inputs:context"),
            ("Context.outputs:context", "CameraHelperInfo.inputs:context"),
            ("Context.outputs:context", "LidarHelper.inputs:context"),

            # --- one simulation clock for every message ---------------
            ("ReadSimTime.outputs:simulationTime", "PublishClock.inputs:timeStamp"),
            ("ReadSimTime.outputs:simulationTime", "PublishOdometry.inputs:timeStamp"),
            ("ReadSimTime.outputs:simulationTime", "PublishRawTF.inputs:timeStamp"),
            ("ReadSimTime.outputs:simulationTime", "PublishTF.inputs:timeStamp"),
            ("ReadSimTime.outputs:simulationTime", "PublishJointState.inputs:timeStamp"),

            # --- tick everything --------------------------------------
            ("OnPlaybackTick.outputs:tick", "PublishClock.inputs:execIn"),
            ("OnPlaybackTick.outputs:tick", "ComputeOdometry.inputs:execIn"),
            ("OnPlaybackTick.outputs:tick", "PublishTF.inputs:execIn"),
            ("OnPlaybackTick.outputs:tick", "PublishJointState.inputs:execIn"),
            ("OnPlaybackTick.outputs:tick", "CreateRenderProduct.inputs:execIn"),
            ("OnPlaybackTick.outputs:tick", "LidarHelper.inputs:execIn"),

            # --- odometry ---------------------------------------------
            ("ComputeOdometry.outputs:execOut", "PublishOdometry.inputs:execIn"),
            ("ComputeOdometry.outputs:execOut", "PublishRawTF.inputs:execIn"),
            ("ComputeOdometry.outputs:position", "PublishOdometry.inputs:position"),
            ("ComputeOdometry.outputs:orientation", "PublishOdometry.inputs:orientation"),
            ("ComputeOdometry.outputs:linearVelocity", "PublishOdometry.inputs:linearVelocity"),
            ("ComputeOdometry.outputs:angularVelocity", "PublishOdometry.inputs:angularVelocity"),
            ("ComputeOdometry.outputs:position", "PublishRawTF.inputs:translation"),
            ("ComputeOdometry.outputs:orientation", "PublishRawTF.inputs:rotation"),

            # --- camera -----------------------------------------------
            ("CreateRenderProduct.outputs:execOut", "CameraHelperRgb.inputs:execIn"),
            ("CreateRenderProduct.outputs:execOut", "CameraHelperDepth.inputs:execIn"),
            ("CreateRenderProduct.outputs:execOut", "CameraHelperInfo.inputs:execIn"),
            ("CreateRenderProduct.outputs:renderProductPath", "CameraHelperRgb.inputs:renderProductPath"),
            ("CreateRenderProduct.outputs:renderProductPath", "CameraHelperDepth.inputs:renderProductPath"),
            ("CreateRenderProduct.outputs:renderProductPath", "CameraHelperInfo.inputs:renderProductPath"),
        ],
    }
    if drive_from_graph:
        _add_drive(spec, keys, cfg, robot, ns)
    og.Controller.edit({"graph_path": GRAPH_PATH, "evaluator_name": "execution"}, spec)
    return GRAPH_PATH


def _add_drive(spec, keys, cfg, robot, ns) -> None:
    """The command half: /cmd_vel -> differential controller -> wheel targets.

    Note what this chain does *not* do: time out. ROS2SubscribeTwist keeps its
    last message and the articulation keeps its last target, so if the sender
    dies without publishing a zero the robot keeps going. ``run\\sim.cmd``
    therefore uses ``base_controller.py`` instead; this path remains for the
    GUI, where there is no Python loop to run it.
    """
    spec[keys.CREATE_NODES] += [
        ("SubscribeTwist", "isaacsim.ros2.bridge.ROS2SubscribeTwist"),
        ("BreakLinear", "omni.graph.nodes.BreakVector3"),
        ("BreakAngular", "omni.graph.nodes.BreakVector3"),
        ("DiffController", "isaacsim.robot.wheeled_robots.DifferentialController"),
        ("ArticulationController", "isaacsim.core.nodes.IsaacArticulationController"),
    ]
    spec[keys.SET_VALUES] += [
        ("SubscribeTwist.inputs:topicName", "cmd_vel"),
        ("SubscribeTwist.inputs:nodeNamespace", ns),

        # Wheel geometry must match the robot we built, or odometry
        # and commanded speed quietly disagree.
        ("DiffController.inputs:wheelRadius", cfg.wheel_radius),
        ("DiffController.inputs:wheelDistance", cfg.wheel_base),
        ("DiffController.inputs:maxLinearSpeed", cfg.max_linear_speed),
        ("DiffController.inputs:maxAngularSpeed", cfg.max_angular_speed),

        ("ArticulationController.inputs:targetPrim", [usdrt.Sdf.Path(robot.root)]),
        ("ArticulationController.inputs:jointNames", ["left_wheel_joint", "right_wheel_joint"]),
    ]
    spec[keys.CONNECT] += [
        ("Context.outputs:context", "SubscribeTwist.inputs:context"),
        ("OnPlaybackTick.outputs:tick", "SubscribeTwist.inputs:execIn"),
        ("OnPlaybackTick.outputs:tick", "DiffController.inputs:execIn"),
        ("OnPlaybackTick.outputs:tick", "ArticulationController.inputs:execIn"),

        # Twist carries 3-vectors; the differential controller wants
        # two scalars, so we tap linear.x and angular.z.
        ("SubscribeTwist.outputs:linearVelocity", "BreakLinear.inputs:tuple"),
        ("SubscribeTwist.outputs:angularVelocity", "BreakAngular.inputs:tuple"),
        ("BreakLinear.outputs:x", "DiffController.inputs:linearVelocity"),
        ("BreakAngular.outputs:z", "DiffController.inputs:angularVelocity"),
        ("DiffController.outputs:velocityCommand", "ArticulationController.inputs:velocityCommand"),
    ]


def expected_topics(cfg) -> "list[str]":
    """What a healthy run should advertise -- used by the smoke test."""
    return [
        cfg.topic("clock"),
        cfg.topic("cmd_vel"),
        cfg.topic("odom"),
        cfg.topic("joint_states"),
        cfg.topic("scan"),
        cfg.topic("rgb"),
        cfg.topic("depth"),
        cfg.topic("camera_info"),
        # The bridge nodes prepend nodeNamespace themselves, so with a
        # namespace set this becomes /<ns>/tf. Conventional ROS keeps /tf and
        # /clock global; if you need that, clear ROS_NAMESPACE or remap.
        cfg.topic("tf"),
    ]
