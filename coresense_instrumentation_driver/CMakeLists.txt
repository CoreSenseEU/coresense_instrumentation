cmake_minimum_required(VERSION 3.5)
project(coresense_instrumentation_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

set(dependencies
  rclcpp
  rclcpp_components
  rclcpp_lifecycle
  std_msgs
  sensor_msgs
)

include_directories(include)

add_library(${PROJECT_NAME} SHARED
  src/coresense_instrumentation_driver/CoresenseInstrumentationDriver.cpp
)
ament_target_dependencies(${PROJECT_NAME} ${dependencies})

add_executable(coresense_instrumentation_driver_node src/coresense_instrumentation_driver_node.cpp)
ament_target_dependencies(coresense_instrumentation_driver_node ${dependencies})
target_link_libraries(coresense_instrumentation_driver_node ${PROJECT_NAME})

rclcpp_components_register_nodes(${PROJECT_NAME}
    PLUGIN "coresense_instrumentation_driver::CoresenseInstrumentationDriver"
    EXECUTABLE coresense_instrumentation_driver_node
)

install(TARGETS
  ${PROJECT_NAME}
  coresense_instrumentation_driver_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

ament_export_include_directories(include)
ament_export_dependencies(${dependencies})
ament_export_libraries(${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  find_package(ament_cmake_gtest REQUIRED)
  ament_lint_auto_find_test_dependencies()
  add_subdirectory(test)
endif()

ament_package()