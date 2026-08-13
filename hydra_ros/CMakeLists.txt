cmake_minimum_required(VERSION 3.22)
project(hydra_ros)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

option(BUILD_SHARED_LIBS "Build shared libraries" ON)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(gflags REQUIRED)
find_package(hydra REQUIRED)
find_package(hydra_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(ianvs REQUIRED)
find_package(kimera_pgmo_msgs REQUIRED)
find_package(kimera_pgmo_ros REQUIRED)
find_package(message_filters REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc)
find_package(PCL REQUIRED)
find_package(pose_graph_tools_msgs REQUIRED)
find_package(pose_graph_tools_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(semantic_inference_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(vision_msgs REQUIRED)

ament_python_install_package(${PROJECT_NAME} PACKAGE_DIR src/${PROJECT_NAME})

add_library(${PROJECT_NAME}
  src/hydra_ros_pipeline.cpp
  src/backend/ros_backend_publisher.cpp
  src/backend/ros_vlm_relationships.cpp
  src/input/enhanced_pointcloud_adaptor.cpp
  src/input/image_receiver.cpp
  src/input/pointcloud_adaptor.cpp
  src/input/pointcloud_receiver.cpp
  src/input/ros_input_module.cpp
  src/input/ros_sensors.cpp
  src/navigation/ros_navigation_interface.cpp
  src/utils/lookup_tf.cpp)
target_include_directories(${PROJECT_NAME}
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>"
  PRIVATE ${OpenCV_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME}
  PUBLIC
    hydra::hydra
    ianvs::ianvs
    kimera_pgmo_ros::kimera_pgmo_ros
    pose_graph_tools_ros::pose_graph_tools_ros
    tf2_eigen::tf2_eigen
    tf2_ros::tf2_ros
  PRIVATE
    cv_bridge::cv_bridge
    ${OpenCV_LIBRARIES}
    ${PCL_LIBRARIES})
ament_target_dependencies(${PROJECT_NAME}
  PUBLIC
    geometry_msgs
    hydra_msgs
    image_transport
    kimera_pgmo_msgs
    message_filters
    pose_graph_tools_msgs
    rclcpp
    sensor_msgs
    semantic_inference_msgs
    std_msgs
    std_srvs
    visualization_msgs
    vision_msgs)

add_executable(hydra_ros_node src/nodes/hydra_node.cpp)
target_include_directories(hydra_ros_node PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(hydra_ros_node ${PROJECT_NAME} gflags)
ament_target_dependencies(hydra_ros_node
  hydra_msgs rclcpp std_srvs)

install(
  TARGETS ${PROJECT_NAME} hydra_ros_node
  EXPORT ${PROJECT_NAME}-targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include/)
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

ament_export_targets(${PROJECT_NAME}-targets HAS_LIBRARY_TARGET)
ament_export_dependencies(
  geometry_msgs hydra hydra_msgs image_transport ianvs kimera_pgmo_msgs
  kimera_pgmo_ros message_filters pose_graph_tools_msgs pose_graph_tools_ros
  rclcpp sensor_msgs semantic_inference_msgs std_msgs tf2_eigen tf2_ros
  std_srvs visualization_msgs vision_msgs)
ament_package()
