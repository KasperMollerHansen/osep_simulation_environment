import omni.graph.core as og

class OmniGraphs:

    @staticmethod
    def camera_graph(camera, topic_name, frame_id, resolution: tuple = (640, 480)):
        og.Controller.edit(
        {"graph_path": "/Graphs/ROS_Camera", "evaluator_name": "execution"},
        {
            og.Controller.Keys.CREATE_NODES: [
                ("ros2_context", "omni.isaac.ros2_bridge.ROS2Context"),
                ("on_playback_tick", "omni.graph.action.OnPlaybackTick"),
                ("isaac_run_one_simulation_frame","omni.isaac.core_nodes.OgnIsaacRunOneSimulationFrame"),
                ("isaac_create_render_product","omni.isaac.core_nodes.IsaacCreateRenderProduct"),
                ("camera_publish_image","omni.isaac.ros2_bridge.ROS2CameraHelper"),
                ("camera_publish_depth","omni.isaac.ros2_bridge.ROS2CameraHelper"),
                ("ros2_camera_info_helper","omni.isaac.ros2_bridge.ROS2CameraInfoHelper"),
                ("ros2_qos_profile", "omni.isaac.ros2_bridge.ROS2QoSProfile"),
            ],
            og.Controller.Keys.CONNECT: [
                # isaac_run_one_simulation_frame inputs
                ("on_playback_tick.outputs:tick", "isaac_run_one_simulation_frame.inputs:execIn"),
                # isaac_create_render_product inputs
                ("isaac_run_one_simulation_frame.outputs:step", "isaac_create_render_product.inputs:execIn"),
                # camera_publish_image inputs
                ("ros2_context.outputs:context", "camera_publish_image.inputs:context"),
                ("ros2_qos_profile.outputs:qosProfile", "camera_publish_image.inputs:qosProfile"),
                ("isaac_create_render_product.outputs:execOut", "camera_publish_image.inputs:execIn"),
                ("isaac_create_render_product.outputs:renderProductPath","camera_publish_image.inputs:renderProductPath"),
                # camera_publish_depth inputs
                ("ros2_context.outputs:context", "camera_publish_depth.inputs:context"),
                ("ros2_qos_profile.outputs:qosProfile", "camera_publish_depth.inputs:qosProfile"),
                ("isaac_create_render_product.outputs:execOut", "camera_publish_depth.inputs:execIn"),
                ("isaac_create_render_product.outputs:renderProductPath","camera_publish_depth.inputs:renderProductPath"),
                # ros2_camera_info_helper inputs
                ("ros2_context.outputs:context", "ros2_camera_info_helper.inputs:context"),
                ("ros2_qos_profile.outputs:qosProfile", "ros2_camera_info_helper.inputs:qosProfile"),
                ("isaac_create_render_product.outputs:execOut", "ros2_camera_info_helper.inputs:execIn"),
                ("isaac_create_render_product.outputs:renderProductPath","ros2_camera_info_helper.inputs:renderProductPath"),
            ],
            og.Controller.Keys.SET_VALUES: [
                # camera_publish_image inputs
                ("camera_publish_image.inputs:topicName", f"{topic_name}/rgb"),
                ("camera_publish_image.inputs:type", "rgb"),
                ("camera_publish_image.inputs:frameId", f"{frame_id}"),
                # camera_publish_depth inputs
                ("camera_publish_depth.inputs:topicName", f"{topic_name}/depth"),
                ("camera_publish_depth.inputs:type", "depth"),
                ("camera_publish_depth.inputs:frameId", f"{frame_id}"),
                # ros2_camera_info_helper inputs
                ("ros2_camera_info_helper.inputs:topicName", f"{topic_name}/info"),
                ("ros2_camera_info_helper.inputs:frameId", f"{frame_id}"),
                # isaac_create_render_product inputs
                ("isaac_create_render_product.inputs:cameraPrim",f"{camera.prim_path}",),
                ("isaac_create_render_product.inputs:height", resolution[1]),
                ("isaac_create_render_product.inputs:width", resolution[0]),
            ],
        },
    )
        
    @staticmethod
    def stereo_camera_graph(prim_path, namespace, camera_prims:tuple, camera_frame_ids:tuple, resolution:tuple = (640, 480)):
        og.Controller.edit(
        {"graph_path": f"{prim_path}/ros_stereo_camera", "evaluator_name": "execution"},
        {
            og.Controller.Keys.CREATE_NODES: [
                ("ros2_context", "omni.isaac.ros2_bridge.ROS2Context"),
                ("on_playback_tick", "omni.graph.action.OnPlaybackTick"),
                ("isaac_run_one_simulation_frame","omni.isaac.core_nodes.OgnIsaacRunOneSimulationFrame"),
                ("left_camera_render_product","omni.isaac.core_nodes.IsaacCreateRenderProduct"),
                ("left_camera_publish_image","omni.isaac.ros2_bridge.ROS2CameraHelper"),
                ("left_camera_frame_id","omni.graph.nodes.ConstantString"),
                ("left_camera_publish_depth","omni.isaac.ros2_bridge.ROS2CameraHelper"),
                ("right_camera_render_product","omni.isaac.core_nodes.IsaacCreateRenderProduct"),
                ("right_camera_publish_image","omni.isaac.ros2_bridge.ROS2CameraHelper"),
                ("right_camera_frame_id","omni.graph.nodes.ConstantString"),
                ("ros2_camera_info_helper","omni.isaac.ros2_bridge.ROS2CameraInfoHelper"),
                ("ros2_qos_profile", "omni.isaac.ros2_bridge.ROS2QoSProfile"),
                ("camera_namespace", "omni.graph.nodes.ConstantString"),
            ],
            og.Controller.Keys.CONNECT: [
                # isaac_run_one_simulation_frame inputs
                ("on_playback_tick.outputs:tick", "isaac_run_one_simulation_frame.inputs:execIn"),
                # left_camera_render_product inputs
                ("isaac_run_one_simulation_frame.outputs:step", "left_camera_render_product.inputs:execIn"),
                # right_camera_render_product inputs
                ("isaac_run_one_simulation_frame.outputs:step", "right_camera_render_product.inputs:execIn"),
                # left_camera_publish_image inputs
                ("camera_namespace.inputs:value", "left_camera_publish_image.inputs:nodeNamespace"),
                ("left_camera_frame_id.inputs:value", "left_camera_publish_image.inputs:frameId"),
                ("ros2_context.outputs:context", "left_camera_publish_image.inputs:context"),
                ("ros2_qos_profile.outputs:qosProfile", "left_camera_publish_image.inputs:qosProfile"),
                ("left_camera_render_product.outputs:execOut", "left_camera_publish_image.inputs:execIn"),
                ("left_camera_render_product.outputs:renderProductPath","left_camera_publish_image.inputs:renderProductPath"),
                # left_camera_publish_depth inputs
                ("camera_namespace.inputs:value", "left_camera_publish_depth.inputs:nodeNamespace"),
                ("left_camera_frame_id.inputs:value", "left_camera_publish_depth.inputs:frameId"),
                ("ros2_context.outputs:context", "left_camera_publish_depth.inputs:context"),
                ("ros2_qos_profile.outputs:qosProfile", "left_camera_publish_depth.inputs:qosProfile"),
                ("left_camera_render_product.outputs:execOut", "left_camera_publish_depth.inputs:execIn"),
                ("left_camera_render_product.outputs:renderProductPath","left_camera_publish_depth.inputs:renderProductPath"),
                # right_camera_publish_image inputs
                ("camera_namespace.inputs:value", "right_camera_publish_image.inputs:nodeNamespace"),
                ("right_camera_frame_id.inputs:value", "right_camera_publish_image.inputs:frameId"),
                ("ros2_context.outputs:context", "right_camera_publish_image.inputs:context"),
                ("ros2_qos_profile.outputs:qosProfile", "right_camera_publish_image.inputs:qosProfile"),
                ("right_camera_render_product.outputs:execOut", "right_camera_publish_image.inputs:execIn"),
                ("right_camera_render_product.outputs:renderProductPath","right_camera_publish_image.inputs:renderProductPath"),
                # ros2_camera_info_helper inputs
                ("camera_namespace.inputs:value", "ros2_camera_info_helper.inputs:nodeNamespace"),
                ("ros2_context.outputs:context", "ros2_camera_info_helper.inputs:context"),
                ("ros2_qos_profile.outputs:qosProfile", "ros2_camera_info_helper.inputs:qosProfile"),
                ("left_camera_render_product.outputs:execOut", "ros2_camera_info_helper.inputs:execIn"),
                ("left_camera_render_product.outputs:renderProductPath","ros2_camera_info_helper.inputs:renderProductPath"),
                ("left_camera_frame_id.inputs:value", "ros2_camera_info_helper.inputs:frameId"),
                ("right_camera_render_product.outputs:execOut", "ros2_camera_info_helper.inputs:execIn"),
                ("right_camera_render_product.outputs:renderProductPath","ros2_camera_info_helper.inputs:renderProductPathRight"),
                ("right_camera_frame_id.inputs:value", "ros2_camera_info_helper.inputs:frameIdRight"),
            ],
            og.Controller.Keys.SET_VALUES: [
                # camera_namespace inputs
                ("camera_namespace.inputs:value", f"{namespace}"),
                # left_camera_frame_id inputs
                ("left_camera_frame_id.inputs:value", f"{camera_frame_ids[0]}"),
                # right_camera_frame_id inputs
                ("right_camera_frame_id.inputs:value", f"{camera_frame_ids[1]}"),
                # left_camera_publish_image inputs
                ("left_camera_publish_image.inputs:topicName", f"left/image"),
                ("left_camera_publish_image.inputs:type", "rgb"),
                # left_camera_publish_depth inputs
                ("left_camera_publish_depth.inputs:topicName", f"left/depth"),
                ("left_camera_publish_depth.inputs:type", "depth"),
                # right_camera_publish_image inputs
                ("right_camera_publish_image.inputs:topicName", f"right/image"),
                ("right_camera_publish_image.inputs:type", "rgb"),
                # ros2_camera_info_helper inputs
                ("ros2_camera_info_helper.inputs:topicName", "left/camera_info"),
                ("ros2_camera_info_helper.inputs:topicNameRight", "right/camera_info"),
                # left_camera_render_product inputs
                ("left_camera_render_product.inputs:cameraPrim", f"{camera_prims[0].prim_path}"),
                ("left_camera_render_product.inputs:height", resolution[1]),
                ("left_camera_render_product.inputs:width", resolution[0]),
                # right_camera_render_product inputs
                ("right_camera_render_product.inputs:cameraPrim", f"{camera_prims[1].prim_path}"),
                ("right_camera_render_product.inputs:height", resolution[1]),
                ("right_camera_render_product.inputs:width", resolution[0]),

            ],
        },
    )
        
    @staticmethod
    def lidar_graph(prim_path, namespace, lidar_prim_path, lidar_id):
        og.Controller.edit(
            {"graph_path": f"{prim_path}/ros_lidar", "evaluator_name": "execution"},
            {
                og.Controller.Keys.CREATE_NODES: [
                    ("ros2_context", "omni.isaac.ros2_bridge.ROS2Context"),
                    ("on_playback_tick", "omni.graph.action.OnPlaybackTick"),
                    ("isaac_run_one_simulation_frame","omni.isaac.core_nodes.OgnIsaacRunOneSimulationFrame"),
                    ("isaac_create_render_product","omni.isaac.core_nodes.IsaacCreateRenderProduct"),
                    ("ros2_qos_profile", "omni.isaac.ros2_bridge.ROS2QoSProfile"),
                    ("rtx_lidar", "omni.isaac.ros2_bridge.ROS2RtxLidarHelper"),
                    ("lidar_namespace", "omni.graph.nodes.ConstantString"),
                ],
                og.Controller.Keys.CONNECT: [
                    # isaac_run_one_simulation_frame inputs
                    ("on_playback_tick.outputs:tick", "isaac_run_one_simulation_frame.inputs:execIn"),
                    # isaac_create_render_product inputs
                    ("isaac_run_one_simulation_frame.outputs:step", "isaac_create_render_product.inputs:execIn"),
                    # rtx_lidar inputs
                    ("lidar_namespace.inputs:value", "rtx_lidar.inputs:nodeNamespace"),
                    ("ros2_qos_profile.outputs:qosProfile", "rtx_lidar.inputs:qosProfile"),
                    ("isaac_create_render_product.outputs:execOut", "rtx_lidar.inputs:execIn"),
                    ("isaac_create_render_product.outputs:renderProductPath","rtx_lidar.inputs:renderProductPath"),
                    ("ros2_context.outputs:context", "rtx_lidar.inputs:context"),
                ],
                og.Controller.Keys.SET_VALUES: [
                    # lida_namespace inputs
                    ("lidar_namespace.inputs:value", f"{namespace}"),
                    # rtx_lidar inputs
                    ("rtx_lidar.inputs:topicName", "raw/pointcloud"),
                    ("rtx_lidar.inputs:type", "point_cloud"),
                    ("rtx_lidar.inputs:frameId", f"{lidar_id}"),
                    ("rtx_lidar.inputs:fullScan", True),
                    # isaac_create_render_product inputs
                    ("isaac_create_render_product.inputs:cameraPrim",f"{lidar_prim_path}",),
                ],
            },
        )
        
    @staticmethod
    def tf_graph(prim_path, base_link_prim, sensor_prims, body_prim_path, topic_prefix):
        og.Controller.edit(
            {"graph_path": f"{prim_path}/transform_tree_odometry", "evaluator_name": "execution"},
            {
                og.Controller.Keys.CREATE_NODES: [
                    ("ros2_context", "omni.isaac.ros2_bridge.ROS2Context"),
                    ("on_playback_tick", "omni.graph.action.OnPlaybackTick"),
                    ("ros2_qos_profile", "omni.isaac.ros2_bridge.ROS2QoSProfile"),
                    ("isaac_read_simulation_time", "omni.isaac.core_nodes.IsaacReadSimulationTime"),
                    ("tf_tree_base_link_to_sensors", "omni.isaac.ros2_bridge.ROS2PublishTransformTree"),
                    ("tf_tree_base_link_to_body", "omni.isaac.ros2_bridge.ROS2PublishTransformTree"),
                    ("isaac_compute_odometry_node", "omni.isaac.core_nodes.IsaacComputeOdometry"),
                    ("ros2_publish_odometry", "omni.isaac.ros2_bridge.ROS2PublishOdometry"),
                    ("ros2_publish_raw_transform_tree", "omni.isaac.ros2_bridge.ROS2PublishRawTransformTree"),
                    ("tf_namespace", "omni.graph.nodes.ConstantString"),
                ],
                og.Controller.Keys.CONNECT: [
                    # tf_tree_base_link_to_sensors inputs
                    ("on_playback_tick.outputs:tick", "tf_tree_base_link_to_sensors.inputs:execIn"),
                    ("ros2_context.outputs:context", "tf_tree_base_link_to_sensors.inputs:context"),
                    ("isaac_read_simulation_time.outputs:simulationTime", "tf_tree_base_link_to_sensors.inputs:timeStamp"),
                    ("ros2_qos_profile.outputs:qosProfile", "tf_tree_base_link_to_sensors.inputs:qosProfile"),
                    # tf_tree_base_link_to_body inputs
                    ("on_playback_tick.outputs:tick", "tf_tree_base_link_to_body.inputs:execIn"),
                    ("ros2_context.outputs:context", "tf_tree_base_link_to_body.inputs:context"),
                    ("isaac_read_simulation_time.outputs:simulationTime", "tf_tree_base_link_to_body.inputs:timeStamp"),
                    ("ros2_qos_profile.outputs:qosProfile", "tf_tree_base_link_to_body.inputs:qosProfile"),
                    # isaac_compute_odometry_node inputs
                    ("on_playback_tick.outputs:tick", "isaac_compute_odometry_node.inputs:execIn"),
                    # ros2_publish_odometry inputs
                    ("tf_namespace.inputs:value", "ros2_publish_odometry.inputs:nodeNamespace"),
                    ("on_playback_tick.outputs:tick", "ros2_publish_odometry.inputs:execIn"),
                    ("ros2_context.outputs:context", "ros2_publish_odometry.inputs:context"),
                    ("ros2_qos_profile.outputs:qosProfile", "ros2_publish_odometry.inputs:qosProfile"),
                    ("isaac_read_simulation_time.outputs:simulationTime", "ros2_publish_odometry.inputs:timeStamp"),
                    ("isaac_compute_odometry_node.outputs:angularVelocity", "ros2_publish_odometry.inputs:angularVelocity"),
                    ("isaac_compute_odometry_node.outputs:linearVelocity", "ros2_publish_odometry.inputs:linearVelocity"),
                    ("isaac_compute_odometry_node.outputs:orientation", "ros2_publish_odometry.inputs:orientation"),
                    ("isaac_compute_odometry_node.outputs:position", "ros2_publish_odometry.inputs:position"),
                    # ros2_publish_raw_transform_tree inputs
                    ("on_playback_tick.outputs:tick", "ros2_publish_raw_transform_tree.inputs:execIn"),
                    ("ros2_context.outputs:context", "ros2_publish_raw_transform_tree.inputs:context"),
                    ("isaac_read_simulation_time.outputs:simulationTime", "ros2_publish_raw_transform_tree.inputs:timeStamp"),
                    ("ros2_qos_profile.outputs:qosProfile", "ros2_publish_raw_transform_tree.inputs:qosProfile"),
                    ("isaac_compute_odometry_node.outputs:position", "ros2_publish_raw_transform_tree.inputs:translation"),
                    ("isaac_compute_odometry_node.outputs:orientation", "ros2_publish_raw_transform_tree.inputs:rotation"),
                                     
                ],
                og.Controller.Keys.SET_VALUES: [
                    # tf_namespace inputs
                    ("tf_namespace.inputs:value", f"{topic_prefix}"),
                    # tf_tree_base_link_to_sensors inputs
                    ("tf_tree_base_link_to_sensors.inputs:parentPrim", f"{base_link_prim}"),
                    ("tf_tree_base_link_to_sensors.inputs:targetPrims", [f"{prims}" for prims in sensor_prims]),
                    # tf_tree_base_link_to_body inputs
                    ("tf_tree_base_link_to_body.inputs:parentPrim", f"{base_link_prim}"),
                    ("tf_tree_base_link_to_body.inputs:targetPrims", f"{body_prim_path}"),
                    # # isaac_compute_odometry_node inputs
                    ("isaac_compute_odometry_node.inputs:chassisPrim", f"{body_prim_path}"),
                    # ros2_publish_odometry inputs
                    ("ros2_publish_odometry.inputs:topicName", "odom"),
                ],

            },
        )

    @staticmethod
    def clock_graph(topic_name):
        og.Controller.edit(
            {"graph_path": "/ros_clock", "evaluator_name": "execution"},
            {
                og.Controller.Keys.CREATE_NODES: [
                    ("ros2_context", "omni.isaac.ros2_bridge.ROS2Context"),
                    ("isaac_read_simulation_time", "omni.isaac.core_nodes.IsaacReadSimulationTime"),
                    ("on_playback_tick", "omni.graph.action.OnPlaybackTick"),
                    ("ros2_publish_clock", "omni.isaac.ros2_bridge.ROS2PublishClock"),
                ],
                og.Controller.Keys.CONNECT: [
                    # ros2_publish_clock inputs
                    ("ros2_context.outputs:context", "ros2_publish_clock.inputs:context"),
                    ("isaac_read_simulation_time.outputs:simulationTime","ros2_publish_clock.inputs:timeStamp"),
                    ("on_playback_tick.outputs:tick", "ros2_publish_clock.inputs:execIn"),
                ],
                og.Controller.Keys.SET_VALUES: [
                    # ros2_publish_clock inputs
                    ("ros2_publish_clock.inputs:topicName", f"{topic_name}"),
                ],
            },
        )
