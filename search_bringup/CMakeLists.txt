cmake_minimum_required(VERSION 3.8)
project(search_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Generate one scoped model SDF and one bridge config per robot from a
# single template each. gazebo-sim does not scope explicitly-declared
# plugin/sensor topics, so each instance needs its own topic and frame names
# to avoid cross-robot collisions.
set(ROBOT_NAMES robot_1 robot_2 robot_3)
foreach(robot_name ${ROBOT_NAMES})
  set(CURRENT_ROBOT ${robot_name})
  configure_file(
    models/turtlebot3_burger.template.sdf
    ${CMAKE_CURRENT_BINARY_DIR}/models/turtlebot3_burger_${robot_name}.sdf
    @ONLY)
  configure_file(
    config/bridge.template.yaml
    ${CMAKE_CURRENT_BINARY_DIR}/config/bridge_${robot_name}.yaml
    @ONLY)
endforeach()

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(FILES
  ${CMAKE_CURRENT_BINARY_DIR}/models/turtlebot3_burger_robot_1.sdf
  ${CMAKE_CURRENT_BINARY_DIR}/models/turtlebot3_burger_robot_2.sdf
  ${CMAKE_CURRENT_BINARY_DIR}/models/turtlebot3_burger_robot_3.sdf
  DESTINATION share/${PROJECT_NAME}/models
)

install(FILES
  ${CMAKE_CURRENT_BINARY_DIR}/config/bridge_robot_1.yaml
  ${CMAKE_CURRENT_BINARY_DIR}/config/bridge_robot_2.yaml
  ${CMAKE_CURRENT_BINARY_DIR}/config/bridge_robot_3.yaml
  config/slam_toolbox.yaml
  DESTINATION share/${PROJECT_NAME}/config
)

ament_package()