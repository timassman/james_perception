cmake_minimum_required(VERSION 3.10)
project(james_perception)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters segmentation search)

include_directories(include ${PCL_INCLUDE_DIRS})

# Bottle detector node
add_executable(bottle_detector_node
  src/bottle_detector_node.cpp
  src/bottle_detector.cpp
)

ament_target_dependencies(bottle_detector_node
  rclcpp
  sensor_msgs
  vision_msgs
  visualization_msgs
  pcl_conversions
  pcl_ros
)

target_link_libraries(bottle_detector_node ${PCL_LIBRARIES})

install(TARGETS bottle_detector_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  # --- Cucumber BDD tests ---
  # Requires cucumber-cpp installed (e.g. from source or apt if available).
  find_package(CucumberCpp QUIET)
  if(CucumberCpp_FOUND)
    find_package(GTest REQUIRED)

    add_executable(bottle_detection_tests
      ../../test/step_definitions/bottle_detection_steps.cpp
      src/bottle_detector.cpp
    )

    target_include_directories(bottle_detection_tests PRIVATE include)

    ament_target_dependencies(bottle_detection_tests
      rclcpp
      sensor_msgs
      vision_msgs
      pcl_conversions
      pcl_ros
    )

    target_link_libraries(bottle_detection_tests
      ${PCL_LIBRARIES}
      CucumberCpp::cucumber-cpp
      GTest::gtest
    )

    # Wire into CTest so `colcon test` picks it up
    add_test(NAME bottle_detection_bdd COMMAND bottle_detection_tests)
  else()
    message(STATUS "cucumber-cpp not found — BDD tests skipped. "
      "Install from https://github.com/cucumber/cucumber-cpp")
  endif()
endif()

ament_package()
