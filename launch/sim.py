# RTX Lidar Anotators: https://docs.omniverse.nvidia.com/isaacsim/latest/features/sensors_simulation/isaac_sim_sensors_rtx_based_lidar/annotator_descriptions.html
# Extension API docs: https://docs.omniverse.nvidia.com/py/isaacsim/source/extensions/omni.isaac.core_nodes/docs/index.html
# Replicator (annotator, writer): https://docs.omniverse.nvidia.com/extensions/latest/ext_replicator.html
# ROS: https://docs.omniverse.nvidia.com/isaacsim/latest/ros_ros2_tutorials.html
# Fix Ubuntu 22 nvidia drivers screwed and laggy: https://www.reddit.com/r/Ubuntu/comments/ub1zun/comment/itbrp2m/
# Python API docs: https://docs.omniverse.nvidia.com/kit/docs/

import argparse
# import asyncio
# import sys
import numpy as np
import json
import time
import random

parser = argparse.ArgumentParser()
# parser.add_argument("-c", "--config", type=str, default="Example_Rotary", help="Name of lidar config.")
parser.add_argument("-g", "--gui", type=str, default="true", help="Enable Isaac Sim Gui")
parser.add_argument("-a", "--assets", type=str, default="./isaac-assets/", help="Assets path")
parser.add_argument("-l", "--lidar", type=str, default="SICK_multiScan136", help="Lidar config name")
args, _ = parser.parse_known_args()

asset_path = args.assets
lidar_conf = args.lidar
# try:
f = open(asset_path + lidar_conf + ".json")
lidar_json = json.load(f)
lidar_timings_ms = np.array(lidar_json['profile']['emitterStates'][0]['fireTimeNs'], dtype=np.uint64).reshape(-1, 1) / 1000
# except Exception as e:
#     print(e)

from isaacsim import SimulationApp

# Example for creating a RTX lidar sensor and publishing PCL data
headless = args.gui != "true"
simulation_app = SimulationApp({"headless": headless})

import carb
import omni
import omni.kit.viewport.utility
import omni.graph.core as og
import omni.replicator.core as rep
from omni.isaac.core import SimulationContext
from omni.isaac.core.utils import stage
from omni.isaac.core.utils.prims import is_prim_path_valid, define_prim, set_prim_property, create_prim, get_prim_at_path
from omni.isaac.core.utils.extensions import enable_extension
from omni.isaac.nucleus import get_assets_root_path
# from omni.isaac.sensor.ogn.python.nodes import OgnIsaacPrintRTXSensorInfo
from pxr import Gf

import rclpy
# from rclpy.node import Node
import sensor_msgs.msg as sensor_msgs
import std_msgs.msg as std_msgs
from builtin_interfaces.msg._time import Time


# enable ROS bridge extension
enable_extension("omni.isaac.debug_draw")
enable_extension("omni.isaac.conveyor")
enable_extension("omni.isaac.ros2_bridge")

simulation_app.update()

PHYSICS_RATE = 60 # fps
RENDER_RATE = 60
carb_settings = carb.settings.get_settings()
carb_settings.set_bool("/app/runLoops/main/rateLimitEnabled", True)
carb_settings.set_int("/app/runLoops/main/rateLimitFrequency", int(PHYSICS_RATE))
carb_settings.set_int("/persistent/simulation/minFrameRate", int(PHYSICS_RATE))
carb_settings.set("/persistent/app/omniverse/gamepadCameraControl", False)

RANDOM_INIT = True

try:
    stage.add_reference_to_stage(
        asset_path + "artemis_arena.usd", "/arena"
    )
    stage.add_reference_to_stage(
        asset_path + "lance.usd", "/lance"
    )
except Exception as e:
    print("Failed to load USD assets:\n\t" + e)
    exit(0)

set_prim_property(
    prim_path = "/arena/artemis_arena",
    property_name = "xformOp:translate",
    property_value = (0, 0, 0)
)
if RANDOM_INIT:
    set_prim_property(
        prim_path = "/lance/lance",
        property_name = "xformOp:translate",
        property_value = (
            random.random() * 0.2 + 0.9,
            random.random() * 0.2 + 0.9,
            0.445) )
    set_prim_property(
        prim_path = "/lance/lance",
        property_name = "xformOp:rotateXYZ",
        property_value = (0, 0, random.random() * 360.) )
else:
    set_prim_property(
        prim_path = "/lance/lance",
        property_name = "xformOp:translate",
        property_value = (1, 1, 0.445) )
create_prim(
    prim_path = "/distantlight",
    prim_type = "DistantLight", 
    orientation = (0.98296, 0.12941, 0.12941, 0.01704),
    attributes = { "inputs:intensity": 1000. } )

simulation_app.update()

# lidar_config = args.config

# Create the lidar sensor that generates data into "RtxSensorCpu"
# Possible config options are Example_Rotary and Example_Solid_State
_, sensor = omni.kit.commands.execute(
    "IsaacSensorCreateRtxLidar",
    path="/lance/lance/lidar_link/sensor",
    parent=None,
    config=lidar_conf,
    translation=(0, 0, 0),
    orientation=Gf.Quatd(1.0, 0.0, 0.0, 0.0),  # Gf.Quatd is w,i,j,k
)
hydra_texture = rep.create.render_product(sensor.GetPath(), [1, 1], name="Isaac")

# Create the debug draw pipeline in the post process graph
if (not headless) :
    debug_writer = rep.writers.get("RtxLidarDebugDrawPointCloudBuffer")
    debug_writer.initialize(color = (0.7, 0.1, 1, 0.7))
    debug_writer.attach([hydra_texture])

lidar_annotator = rep.AnnotatorRegistry.get_annotator("RtxSensorCpuIsaacCreateRTXLidarScanBuffer")
lidar_annotator.initialize(
    keepOnlyPositiveDistance = False,
    outputAzimuth = True,
    outputBeamId = False,
    outputDistance = True,
    outputElevation = True,
    outputEmitterId = False,
    outputIntensity = True,
    outputMaterialId = True,
    outputNormal = False,
    outputObjectId = False,
    outputTimestamp = False,
    outputVelocity = False,
    transformPoints = False
)
lidar_annotator.attach([hydra_texture])

simulation_app.update()

rclpy.init()

try:
    og.Controller.edit(
        {"graph_path": "/graphs/ROSGraph", "evaluator_name": "execution"},
        {
            # og.Controller.Keys.CREATE_VARIABLES: [
            #     ("has_inited", "bool", False),
            #     ("init_inv_tf", "matrixd[4]")
            # ],
            og.Controller.Keys.CREATE_NODES: [
                ("OnPlaybackTick",  "omni.graph.action.OnPlaybackTick"),
                ("ReadSimTime",     "omni.isaac.core_nodes.IsaacReadSimulationTime"),
                ("PublishClock",    "omni.isaac.ros2_bridge.ROS2PublishClock"),
                ("ReadIMU",         "omni.isaac.sensor.IsaacReadIMU"),
                ("PublishIMU",      "omni.isaac.ros2_bridge.ROS2PublishImu"),
                ("ReadRTF",         "omni.isaac.core_nodes.IsaacRealTimeFactor"),
                ("PublishRTF",      "omni.isaac.ros2_bridge.ROS2Publisher"),
                ("DumpJointPub",    "omni.isaac.ros2_bridge.ROS2PublishTransformTree"),

                ("WorldTransform",      "omni.graph.nodes.GetPrimLocalToWorldTransform"),
                ("ExtractQuat",         "omni.graph.nodes.GetMatrix4Quaternion"),
                ("ExtractVec",          "omni.graph.nodes.GetMatrix4Translation"),
                ("WorldTransformPub",   "omni.isaac.ros2_bridge.ROS2PublishRawTransformTree"),

                # ("ReadHasInited",       "omni.graph.core.ReadVariable"),
                # ("InitBranch",          "omni.graph.action.Branch"),
                # ("InvertMatrix",        "omni.graph.nodes.OgnInvertMatrix"),
                # ("WriteInitMatrix",     "omni.graph.core.WriteVariable"),
                # ("WriteInitBool",       "omni.graph.core.WriteVariable"),
                # ("ReadInitMatrix",      "omni.graph.core.ReadVariable"),
                # ("MatrixMul",           "omni.graph.nodes.MatrixMultiply"),
                # ("ExtractOdomQuat",     "omni.graph.nodes.GetMatrix4Quaternion"),
                # ("ExtractOdomVec",      "omni.graph.nodes.GetMatrix4Translation"),
                # ("OdomTransformPub",    "omni.isaac.ros2_bridge.ROS2PublishRawTransformTree"),
            ],
            og.Controller.Keys.CONNECT: [
                # Execution connections
                ("OnPlaybackTick.outputs:tick", "PublishClock.inputs:execIn"),
                ("OnPlaybackTick.outputs:tick", "ReadIMU.inputs:execIn"),
                ("ReadIMU.outputs:execOut",     "PublishIMU.inputs:execIn"),
                ("OnPlaybackTick.outputs:tick", "PublishRTF.inputs:execIn"),
                ("OnPlaybackTick.outputs:tick", "DumpJointPub.inputs:execIn"),
                ("OnPlaybackTick.outputs:tick", "WorldTransformPub.inputs:execIn"),
                # ("OnPlaybackTick.outputs:tick", "InitBranch.inputs:execIn"),
                # Simulation time
                ("ReadSimTime.outputs:simulationTime", "PublishClock.inputs:timeStamp"),
                ("ReadSimTime.outputs:simulationTime", "PublishIMU.inputs:timeStamp"),
                ("ReadSimTime.outputs:simulationTime", "DumpJointPub.inputs:timeStamp"),
                ("ReadSimTime.outputs:simulationTime", "WorldTransformPub.inputs:timeStamp"),
                # ("ReadSimTime.outputs:simulationTime", "OdomTransformPub.inputs:timeStamp"),
                # IMU data
                ("ReadIMU.outputs:angVel",      "PublishIMU.inputs:angularVelocity"),
                ("ReadIMU.outputs:linAcc",      "PublishIMU.inputs:linearAcceleration"),
                ("ReadIMU.outputs:orientation", "PublishIMU.inputs:orientation"),
                # # Connecting the ROS2 Context to the clock publisher node so it will run under the specified ROS2 Domain ID
                # ("Context.outputs:context", "PublishClock.inputs:context"),
                ("WorldTransform.outputs:localToWorldTransform",    "ExtractQuat.inputs:matrix"),
                ("WorldTransform.outputs:localToWorldTransform",    "ExtractVec.inputs:matrix"),
                ("ExtractQuat.outputs:quaternion",                  "WorldTransformPub.inputs:rotation"),
                ("ExtractVec.outputs:translation",                  "WorldTransformPub.inputs:translation"),

                # ("ReadHasInited.outputs:value",                     "InitBranch.inputs:condition"),
                # ("InitBranch.outputs:execFalse",                    "WriteInitMatrix.inputs:execIn"),
                # ("InitBranch.outputs:execTrue",                     "OdomTransformPub.inputs:execIn"),
                # ("WriteInitMatrix.outputs:execOut",                 "WriteInitBool.inputs:execIn"),
                # ("WriteInitBool.outputs:execOut",                   "OdomTransformPub.inputs:execIn"),
                # ("WorldTransform.outputs:localToWorldTransform",    "InvertMatrix.inputs:matrix"),
                # ("InvertMatrix.outputs:invertedMatrix",             "WriteInitMatrix.inputs:value"),
                # ("ReadInitMatrix.outputs:value",                    "MatrixMul.inputs:b"),
                # ("WorldTransform.outputs:localToWorldTransform",    "MatrixMul.inputs:a"),
                # ("MatrixMul.outputs:output",                        "ExtractOdomQuat.inputs:matrix"),
                # ("MatrixMul.outputs:output",                        "ExtractOdomVec.inputs:matrix"),
                # ("ExtractOdomQuat.outputs:quaternion",              "OdomTransformPub.inputs:rotation"),
                # ("ExtractOdomVec.outputs:translation",              "OdomTransformPub.inputs:translation"),
            ],
            og.Controller.Keys.SET_VALUES: [
                # Assigning topic name to clock publisher
                ("PublishClock.inputs:topicName", "/clock"),
                # # Assigning a Domain ID of 1 to Context node
                # ("Context.inputs:domain_id", 1),
                ("ReadSimTime.inputs:resetOnStop", True),

                ("ReadIMU.inputs:imuPrim",      "/lance/lance/lidar_link/imu_sensor"),
                ("PublishIMU.inputs:frameId",   "lidar_link"),
                ("PublishIMU.inputs:queueSize", 1),
                ("PublishIMU.inputs:topicName", "/lance/imu"),

                ("PublishRTF.inputs:messageName",       "Float32"),
                ("PublishRTF.inputs:messagePackage",    "std_msgs"),
                ("PublishRTF.inputs:messageSubfolder",  "msg"),
                ("PublishRTF.inputs:topicName",         "/isaac/rtf"),

                ("DumpJointPub.inputs:parentPrim",  "/lance/lance/frame_link"),
                ("DumpJointPub.inputs:targetPrims", "/lance/lance/collection_link"),

                ("WorldTransform.inputs:prim",          "/lance/lance/base_link"),
                ("WorldTransform.inputs:usePath",           False),
                ("WorldTransformPub.inputs:childFrameId",   "sim_global_link"),
                ("WorldTransformPub.inputs:parentFrameId",  "map"),
                ("WorldTransformPub.inputs:queueSize",      1),
                ("WorldTransformPub.inputs:topicName",      "tf"),

                # ("ReadHasInited.inputs:variableName",       "has_inited"),
                # ("WriteInitBool.inputs:variableName",       "has_inited"),
                # ("ReadInitMatrix.inputs:variableName",      "init_inv_tf"),
                # ("WriteInitMatrix.inputs:variableName",     "init_inv_tf"),
                # ("OdomTransformPub.inputs:childFrameId",    "sim_odom_link"),
                # ("OdomTransformPub.inputs:parentFrameId",   "odom"),
                # ("OdomTransformPub.inputs:queueSize",       1),
                # ("OdomTransformPub.inputs:topicName",       "tf"),
            ],
        },
    )
    og.Controller.connect(
        og.Controller.attribute("/graphs/ROSGraph/ReadRTF.outputs:rtf"),
        og.Controller.attribute("/graphs/ROSGraph/PublishRTF.inputs:data")
    )
except Exception as e:
    print(e)
# stage.add_reference_to_stage(
#     asset_path + "ROSGraph.usd", "/graphs" )
# og.Controller.attribute(graph_id = "/graphs/ROSGraph", node_id = "ReadIMU", attribute_id = "inputs:imuPrim").set("/lance/lance/lidar_link/imu_sensor")
# og.Controller.attribute(graph_id = "/graphs/ROSGraph", node_id = "DumpJointPub", attribute_id = "inputs:parentPrim").set("/lance/lance/frame_link")
# og.Controller.attribute(graph_id = "/graphs/ROSGraph", node_id = "DumpJointPub", attribute_id = "inputs:targetPrims").set("/lance/lance/collection_link")
# og.Controller.attribute(graph_id = "/graphs/ROSGraph", node_id = "WorldTransform", attribute_id = "inputs:prim").set("/lance/lance/base_link")

simulation_app.update()

try:
    og.Controller.edit(
        {
            "graph_path" : "/graphs/ControlGraph",
            "evaluator_name" : "execution",
            "pipeline_stage" : og.GraphPipelineStage.GRAPH_PIPELINE_STAGE_ONDEMAND
        },
        {
            og.Controller.Keys.CREATE_NODES: [
                ("PhysicsStep",     "omni.isaac.core_nodes.OnPhysicsStep"),
                ("TwistSub",        "omni.isaac.ros2_bridge.ROS2SubscribeTwist"),
                ("GetXVel",         "omni.graph.nodes.BreakVector3"),
                ("GetRotVel",       "omni.graph.nodes.BreakVector3"),
                ("DiffController",  "omni.isaac.wheeled_robots.DifferentialController"),
                ("GetLeftVel",      "omni.graph.nodes.ArrayIndex"),
                ("GetRightVel",     "omni.graph.nodes.ArrayIndex"),
                ("LeftController",  "omni.isaac.conveyor.IsaacConveyor"),
                ("RightController", "omni.isaac.conveyor.IsaacConveyor"),
                ("DumpSub",         "omni.isaac.ros2_bridge.ROS2Subscriber"),
                ("JointToken",      "omni.graph.nodes.ConstantToken"),
                ("MakeTokenArray",  "omni.graph.nodes.ConstructArray"),
                ("ToDouble",        "omni.graph.nodes.ToDouble"),
                ("MakeVelArray",    "omni.graph.nodes.ConstructArray"),
                ("DumpController",  "omni.isaac.core_nodes.IsaacArticulationController"),
            ],
            og.Controller.Keys.CONNECT: [
                ("PhysicsStep.outputs:step",    "TwistSub.inputs:execIn"),
                ("PhysicsStep.outputs:step",    "LeftController.inputs:onStep"),
                ("PhysicsStep.outputs:step",    "RightController.inputs:onStep"),
                ("PhysicsStep.outputs:step",    "DumpSub.inputs:execIn"),
                ("TwistSub.outputs:execOut",    "DiffController.inputs:execIn"),
                ("DumpSub.outputs:execOut",     "DumpController.inputs:execIn"),

                ("TwistSub.outputs:angularVelocity",        "GetRotVel.inputs:tuple"),
                ("TwistSub.outputs:linearVelocity",         "GetXVel.inputs:tuple"),
                ("GetRotVel.outputs:z",                     "DiffController.inputs:angularVelocity"),
                ("GetXVel.outputs:x",                       "DiffController.inputs:linearVelocity"),
                ("PhysicsStep.outputs:deltaSimulationTime", "DiffController.inputs:dt"),
                ("DiffController.outputs:velocityCommand",  "GetLeftVel.inputs:array"),
                ("DiffController.outputs:velocityCommand",  "GetRightVel.inputs:array"),
                ("GetLeftVel.outputs:value",                "LeftController.inputs:velocity"),
                ("GetRightVel.outputs:value",               "RightController.inputs:velocity"),
                ("PhysicsStep.outputs:deltaSimulationTime", "LeftController.inputs:delta"),
                ("PhysicsStep.outputs:deltaSimulationTime", "RightController.inputs:delta"),

                ("JointToken.inputs:value",        "MakeTokenArray.inputs:input0"),
                ("ToDouble.outputs:converted",      "MakeVelArray.inputs:input0"),
                ("MakeTokenArray.outputs:array",    "DumpController.inputs:jointNames"),
                ("MakeVelArray.outputs:array",      "DumpController.inputs:velocityCommand"),
            ],
            og.Controller.Keys.SET_VALUES: [
                ("TwistSub.inputs:topicName", "/robot_cmd_vel"),

                ("DiffController.inputs:maxAcceleration",        3.),
                ("DiffController.inputs:maxAngularAcceleration", 10.),
                ("DiffController.inputs:maxAngularSpeed",        5.),
                ("DiffController.inputs:maxDeceleration",        3.5),
                ("DiffController.inputs:maxLinearSpeed",         1.5),
                ("DiffController.inputs:maxWheelSpeed",          2.),
                ("DiffController.inputs:wheelDistance",          0.579),
                ("DiffController.inputs:wheelRadius",            1.),

                ("GetLeftVel.inputs:index",     1),
                ("GetRightVel.inputs:index",    0),

                ("LeftController.inputs:conveyorPrim",  "/lance/lance/left_track_link"),
                ("RightController.inputs:conveyorPrim", "/lance/lance/right_track_link"),

                ("DumpSub.inputs:messageName",      "Float64"),
                ("DumpSub.inputs:messagePackage",   "std_msgs"),
                ("DumpSub.inputs:messageSubfolder", "msg"),
                ("DumpSub.inputs:topicName",        "/dump_cmd_vel"),

                ("JointToken.inputs:value", "dump_joint"),

                ("DumpController.inputs:targetPrim", "/lance/lance"),
            ]
        }
    )
    og.Controller.connect(
        og.Controller.attribute("/graphs/ControlGraph/DumpSub.outputs:data"),
        og.Controller.attribute("/graphs/ControlGraph/ToDouble.inputs:value")
    )
except Exception as e:
    print(e)

simulation_app.update()

def normalize_retro(x : float):
    return 1. if x == 12. or x == 11. else 0.   # https://docs.omniverse.nvidia.com/isaacsim/latest/features/sensors_simulation/isaac_sim_sensors_rtx_sensor_materials.html

np_normalize_retro = np.frompyfunc(normalize_retro, 1, 1)

simulation_context = SimulationContext(
    physics_dt = (1. / PHYSICS_RATE),
    rendering_dt = (1. / RENDER_RATE),
    stage_units_in_meters=1.
)
simulation_app.update()

node = rclpy.create_node("isaac_sim")
pc_pub = node.create_publisher(sensor_msgs.PointCloud2, '/lance/lidar_scan', 10)

pc_dtype = np.dtype( [
    ('xyz', np.float32, (3,)),
    ('reflective', np.float32, (1,)),
    ('range', np.float32, (1,)),
    ('azimuth', np.float32, (1,)),
    ('elevation', np.float32, (1,)),
    ('intensity', np.float32, (1,)),
    ('ts', np.uint64, (1,))
] )
# pc_itemsize = np.dtype(pc_dtype).itemsize
pc_fields = [
    sensor_msgs.PointField(name = 'x',          offset = 0, datatype = sensor_msgs.PointField.FLOAT32, count = 1),
    sensor_msgs.PointField(name = 'y',          offset = 4, datatype = sensor_msgs.PointField.FLOAT32, count = 1),
    sensor_msgs.PointField(name = 'z',          offset = 8, datatype = sensor_msgs.PointField.FLOAT32, count = 1),
    sensor_msgs.PointField(name = 'reflective', offset = 12, datatype = sensor_msgs.PointField.FLOAT32, count = 1),
    sensor_msgs.PointField(name = 'range',      offset = 16, datatype = sensor_msgs.PointField.FLOAT32, count = 1),
    sensor_msgs.PointField(name = 'azimuth',    offset = 20, datatype = sensor_msgs.PointField.FLOAT32, count = 1),
    sensor_msgs.PointField(name = 'elevation',  offset = 24, datatype = sensor_msgs.PointField.FLOAT32, count = 1),
    sensor_msgs.PointField(name = 'intensity',  offset = 28, datatype = sensor_msgs.PointField.FLOAT32, count = 1),
    sensor_msgs.PointField(name = 'tl',         offset = 32, datatype = sensor_msgs.PointField.UINT32, count = 1),
    sensor_msgs.PointField(name = 'th',         offset = 36, datatype = sensor_msgs.PointField.UINT32, count = 1),
]

PC_PUB_RATE = 20
PUB_THRESH_S = 0.002

if (headless) :
    simulation_app.update()
    simulation_context.play()

prev_pub_t = -1
context_changed = True
while simulation_app.is_running():

    simulation_app.update()
    # lidar_annotator.get_data()['data']

    if simulation_context.is_playing() :
        t = simulation_context.current_time
        if context_changed : prev_pub_t = t
        if ((t - prev_pub_t) > (1. / PC_PUB_RATE) - PUB_THRESH_S) :

            # begin = time.perf_counter()
            try:
                lidar_data = lidar_annotator.get_data()
                xyz = lidar_data['data']

                if (len(xyz) > 0) :

                    scan_buff = np.empty(xyz.shape[0], dtype = pc_dtype)
                    scan_buff['xyz'] = xyz
                    scan_buff['reflective'] = np_normalize_retro( lidar_data['materialId'] ).astype(np.float32).reshape(-1, 1)
                    scan_buff['range'] = lidar_data['distance'].astype(np.float32).reshape(-1, 1)
                    scan_buff['azimuth'] = lidar_data['azimuth'].astype(np.float32).reshape(-1, 1)
                    scan_buff['elevation'] = lidar_data['elevation'].astype(np.float32).reshape(-1, 1)
                    scan_buff['intensity'] = lidar_data['intensity'].astype(np.float32).reshape(-1, 1)
                    scan_buff['ts'] = (lidar_timings_ms + (int)(prev_pub_t * 1e6)).astype(np.uint64)

                    header = std_msgs.Header(
                        stamp = Time(sec = int(prev_pub_t),
                        nanosec = int(prev_pub_t * 1e9) % int(1e9)),
                        frame_id = "lidar_link" )
                    pc_pub.publish(
                        sensor_msgs.PointCloud2(
                            header = header,
                            height = 1,
                            width = scan_buff.shape[0],
                            is_dense = True,
                            is_bigendian = False,
                            fields = pc_fields,
                            point_step = 40,
                            row_step = 40 * scan_buff.shape[0],
                            data = scan_buff.tobytes()
                        ) )
                    prev_pub_t = t

            except Exception as e:
                print(e)

            # end = time.perf_counter()
            # print(f"Scan pub dt: {end - begin}", t, flush=True)

        context_changed = False
    else:
        context_changed = True

    rclpy.spin_once(node, timeout_sec = 0.)


# cleanup and shutdown
rclpy.shutdown()
simulation_context.stop()
simulation_app.close()
