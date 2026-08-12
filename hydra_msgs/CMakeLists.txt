cmake_minimum_required(VERSION 3.16)
project(hydra_msgs)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(semantic_inference_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(vision_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ActiveLayer.msg"
  "msg/ActiveObjectRelationships.msg"
  "msg/Detections3D.msg"
  "msg/DsgUpdate.msg"
  "msg/LabeledRelationships.msg"
  "msg/Navigation.msg"
  "msg/NavigationPath.msg"
  "msg/ObjectSearch.msg"
  "msg/ObjectSearchPair.msg"
  "msg/VisualRelationshipsEncodings.msg"
  "srv/GetDsg.srv"
  "srv/GetNavigation.srv"
  "srv/Load.srv"
  "srv/QueryFreespace.srv"
  "srv/VLMRelationship.srv"
  DEPENDENCIES
    geometry_msgs
    semantic_inference_msgs
    std_msgs
    vision_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
