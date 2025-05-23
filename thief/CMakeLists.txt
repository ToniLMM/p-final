cmake_minimum_required(VERSION 3.5)
project(thief)

set(CMAKE_CONFIG_PATH ${CMAKE_MODULE_PATH}  "${CMAKE_CURRENT_LIST_DIR}/cmake")
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CONFIG_PATH}")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(yolo_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(kobuki_ros_interfaces REQUIRED)

find_package(ZMQ)
if(ZMQ_FOUND)
    message(STATUS "ZeroMQ found.")
    add_definitions(-DZMQ_FOUND)
else()
  message(WARNING "ZeroMQ NOT found. Not including PublisherZMQ.")
endif()

set(CMAKE_CXX_STANDARD 17)

set(dependencies
    rclcpp
    behaviortree_cpp_v3
    sensor_msgs
    geometry_msgs
    ament_index_cpp
    rclcpp_action
    nav2_msgs
    yolo_msgs
    tf2_geometry_msgs
    tf2_ros
    lifecycle_msgs
    kobuki_ros_interfaces
)

include_directories(include ${ZMQ_INCLUDE_DIRS})

add_library(turn_bt_node SHARED src/thief/Turn.cpp)
add_library(getwp_bt_node SHARED src/thief/GetWaypoint.cpp)
add_library(move_bt_node SHARED src/thief/Move.cpp)
add_library(move2_bt_node SHARED src/thief/Move2.cpp)
add_library(getwp2_bt_node SHARED src/thief/GetWaypoint2.cpp)

list(APPEND plugin_libs
  getwp_bt_node
  turn_bt_node
  move_bt_node
  move2_bt_node
  getwp2_bt_node
)

foreach(bt_plugin ${plugin_libs})
  ament_target_dependencies(${bt_plugin} ${dependencies})
  target_compile_definitions(${bt_plugin} PRIVATE BT_PLUGIN_EXPORT)
endforeach()

add_executable(thief src/bt_thief_main.cpp)
ament_target_dependencies(thief ${dependencies})
target_link_libraries(thief ${ZMQ_LIBRARIES})

install(TARGETS
  ${plugin_libs}
  thief
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY behavior_tree_xml
  launch
  DESTINATION share/${PROJECT_NAME}
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
endif()

ament_export_include_directories(include)
ament_export_dependencies(${dependencies})

ament_package()
