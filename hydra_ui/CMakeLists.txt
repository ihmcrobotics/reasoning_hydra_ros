cmake_minimum_required(VERSION 3.16)
project(hydra_ui)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_AUTOMOC ON)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra)
endif()

find_package(ament_cmake REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rviz_common REQUIRED)
find_package(semantic_inference_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)

add_definitions(-DQT_NO_KEYWORDS)

add_library(${PROJECT_NAME} SHARED
  src/hydra_ui.cpp
  src/hydra_ui.h)
target_include_directories(${PROJECT_NAME} PRIVATE src)
target_link_libraries(${PROJECT_NAME} Qt5::Widgets)
ament_target_dependencies(${PROJECT_NAME}
  pluginlib rclcpp rviz_common semantic_inference_msgs std_srvs)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(FILES plugin_description.xml DESTINATION share/${PROJECT_NAME})
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/icons")
  install(DIRECTORY icons/ DESTINATION share/${PROJECT_NAME}/icons)
endif()

pluginlib_export_plugin_description_file(rviz_common plugin_description.xml)
ament_package()
