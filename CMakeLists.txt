cmake_minimum_required(VERSION 3.16)
project(hand_sdk_ros2)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake REQUIRED)
find_package(hand_sdk REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)

find_package(Qt5 REQUIRED COMPONENTS Widgets)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SetGains.srv"
)
ament_export_dependencies(rosidl_default_runtime)

# The node carries the diagnostic board, so it links Qt too. src/ is on the
# include path for the uic output the board header pulls in.
add_executable(hand_sdk_ros2_node
  src/hand_sdk_ros2_node.cpp
  src/hand_sdk_node_board.cpp
)
target_include_directories(hand_sdk_ros2_node PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
target_compile_features(hand_sdk_ros2_node PRIVATE cxx_std_17)
target_link_libraries(hand_sdk_ros2_node hand_sdk::hand_sdk Qt5::Widgets)
ament_target_dependencies(hand_sdk_ros2_node rclcpp sensor_msgs)

# The node uses the service type this same package generates. Interfaces and
# executables in one package need the typesupport target linked by hand --
# ament_target_dependencies cannot see a target that does not exist yet at
# configure time.
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(hand_sdk_ros2_node "${cpp_typesupport_target}")

add_executable(hand_sdk_app src/hand_sdk_app.cpp)
target_include_directories(hand_sdk_app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
target_compile_features(hand_sdk_app PRIVATE cxx_std_17)
target_link_libraries(hand_sdk_app
  Qt5::Widgets
  hand_sdk::hand_sdk
)

install(TARGETS hand_sdk_ros2_node hand_sdk_app
  DESTINATION lib/${PROJECT_NAME})

ament_package()