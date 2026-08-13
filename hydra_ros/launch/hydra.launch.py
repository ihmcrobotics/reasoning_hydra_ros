#!/usr/bin/env python3
"""Launch the standard or reasoning-enabled Hydra pipeline with ROS 2.

The reasoning fork still uses config_utilities for its module configuration.
Consequently, configuration files are passed as executable arguments rather
than as ROS parameters.  Files listed later override values from earlier files,
matching the precedence of the former ROS 1 launch file.

Reasoning is opt-in through ``enable_reasoning``. When enabled, the backend
subscriber configuration is loaded under ``backend/reasoning``.
"""

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _value(context, name):
    return LaunchConfiguration(name).perform(context)


def _config_file(path, namespace=None):
    return f"{path}@{namespace}" if namespace else path


def _launch_hydra(context):
    hydra_share = get_package_share_directory("hydra")
    hydra_ros_share = get_package_share_directory("hydra_ros")

    dataset = _value(context, "dataset")
    config_name = _value(context, "config_name")
    labelspace_name = _value(context, "labelspace_name")
    config_dir = os.path.join(hydra_share, "config", config_name)

    files = [
        os.path.join(hydra_ros_share, "config", "ros_pipeline.yaml"),
        os.path.join(
            hydra_share,
            "config",
            "label_spaces",
            f"{labelspace_name}_label_space.yaml",
        ),
        _config_file(
            os.path.join(config_dir, "reconstruction_config.yaml"),
            "reconstruction",
        ),
        _config_file(os.path.join(config_dir, "frontend_config.yaml"), "frontend"),
        _config_file(os.path.join(config_dir, "backend_config.yaml"), "backend"),
    ]

    optional_files = [
        (os.path.join(config_dir, "object_search_config.yaml"), "object_search"),
        (os.path.join(config_dir, "lcd_config.yaml"), None),
    ]
    if _value(context, "enable_reasoning").lower() == "true":
        optional_files.append(
            (
            os.path.join(config_dir, "backend_subscriber_config.yaml"),
            "backend/reasoning",
            )
        )
    files.extend(
        _config_file(path, namespace)
        for path, namespace in optional_files
        if os.path.isfile(path)
    )

    missing = [entry.split("@", 1)[0] for entry in files if not os.path.isfile(entry.split("@", 1)[0])]
    if missing:
        raise RuntimeError("Missing Hydra configuration files: " + ", ".join(missing))

    # Replace the ROS 1 $(arg ...) expressions present in the original dataset
    # files.  Overriding the complete receiver list also makes the effective
    # sensor configuration explicit in the launch description.
    overrides = {
        "robot_id": int(_value(context, "robot_id")),
        "config_verbosity": int(_value(context, "config_verbosity")),
        "exit_after_clock": _value(context, "exit_after_clock").lower() == "true",
        "robot_frame": _value(context, "robot_frame"),
        "odom_frame": _value(context, "odom_frame"),
        "map_frame": _value(context, "map_frame"),
        "input": {
            "type": "RosInput",
            "tf_verbosity": 1,
            "clear_queue_on_fail": True,
            "receivers": [
                {
                    "type": "ImageReceiver",
                    "sensor": {
                        "type": "camera_info",
                        "min_range": float(_value(context, "sensor_min_range")),
                        "max_range": float(_value(context, "sensor_max_range")),
                        "camera_info_topic": _value(context, "camera_info_topic"),
                        "extrinsics": {
                            "type": "ros",
                            "sensor_frame": _value(context, "sensor_frame"),
                        },
                    },
                }
            ]
        },
        "enable_frontend_output": _value(context, "enable_frontend_output").lower()
        == "true",
        "enable_reasoning": _value(context, "enable_reasoning").lower() == "true",
        "active_object_edges_topic": _value(
            context, "active_object_edges_topic"
        ),
        "vlm_relationship_service": _value(context, "vlm_relationship_service"),
        "vlm_encodings_topic": _value(context, "vlm_encodings_topic"),
        "vlm_labels_topic": _value(context, "vlm_labels_topic"),
        "enable_lcd": _value(context, "enable_lcd").lower() == "true",
        "log_path": _value(context, "log_path"),
        "print_missing": False,
        "show_config": _value(context, "show_config").lower() == "true",
    }

    arguments = []
    for config_file in files:
        arguments.extend(["--config-utilities-file", config_file])
    arguments.extend(
        [
            "--config-utilities-yaml",
            yaml.safe_dump(overrides, default_flow_style=True).strip(),
        ]
    )

    namespace = _value(context, "namespace").strip("/")
    input_ns = f"/{namespace}/input" if namespace else "/input"
    return [
        Node(
            package="hydra_ros",
            executable="hydra_ros_node",
            name="hydra_ros_node",
            namespace=namespace,
            output="screen",
            parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
            arguments=arguments,
            remappings=[
                (f"{input_ns}/rgb/image_raw", _value(context, "rgb_topic")),
                (
                    f"{input_ns}/depth_registered/image_rect",
                    _value(context, "depth_topic"),
                ),
                (f"{input_ns}/rgb/camera_info", _value(context, "camera_info_topic")),
                (f"{input_ns}/panoptic/image_raw", _value(context, "panoptic_topic")),
                (f"{input_ns}/semantic", _value(context, "semantic_topic")),
                (f"{input_ns}/image_feature", _value(context, "image_feature_topic")),
                (f"{input_ns}/relations", _value(context, "relations_topic")),
            ],
        )
    ]


def generate_launch_description():
    defaults = {
        "namespace": "hydra",
        "dataset": "uhumans2",
        "config_name": "uhumans2",
        "labelspace_name": "uhumans2_office",
        "robot_id": "0",
        "config_verbosity": "1",
        "robot_frame": "base_link",
        "odom_frame": "world",
        "map_frame": "world",
        "sensor_frame": "sensor",
        "sensor_min_range": "0.1",
        "sensor_max_range": "5.0",
        "rgb_topic": "/rgb/image_raw",
        "depth_topic": "/depth_registered/image_rect",
        "camera_info_topic": "/rgb/camera_info",
        "panoptic_topic": "/panoptic/image_raw",
        "semantic_topic": "/semantic_inference/semantic_color/feature_image",
        "image_feature_topic": "/semantic_inference/image_feature",
        "relations_topic": "/semantic_inference/relation_features",
        "active_object_edges_topic":
            "/hydra_ros_node/backend/active_object_edges",
        "vlm_relationship_service":
            "/hydra_ros_node/backend/vlm_relationships",
        "vlm_encodings_topic":
            "/hydra_ros_node/backend/vlm_relationships/visual_relationships_encodings",
        "vlm_labels_topic": "/semantic_inference/labeled_relationships",
        "enable_frontend_output": "true",
        "enable_lcd": "false",
        "enable_reasoning": "false",
        "exit_after_clock": "false",
        "show_config": "true",
        "use_sim_time": "false",
        "log_path": os.path.expanduser("~/.hydra/uhumans2"),
    }

    return LaunchDescription(
        [
            *[
                DeclareLaunchArgument(name, default_value=value)
                for name, value in defaults.items()
            ],
            OpaqueFunction(function=_launch_hydra),
        ]
    )
