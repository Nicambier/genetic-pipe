cmake_minimum_required(VERSION 3.5)
project(gazebo_ros_simple_motor)

set(library_name
  ${PROJECT_NAME}
)

set(test_name
  test_${PROJECT_NAME}
)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(gazebo_ros_simple_motor_msgs REQUIRED)
find_package(rclcpp REQUIRED)

# gazebo_ros_simple_motor shared library.

add_library(${library_name} SHARED
  src/${library_name}.cpp
)

target_include_directories(${library_name} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  # FIXME The next line is a bit of a hack!
  /usr/include/gazebo-11/gazebo
)

ament_target_dependencies(${library_name}
  "gazebo_dev"
  "gazebo_ros"
  "gazebo_ros_simple_motor_msgs"
  "rclcpp"
)

ament_export_libraries(${library_name})

install(DIRECTORY include/
        DESTINATION include
)

install(TARGETS
  ${library_name}  # FIXME Not sure the library should be installed where ever this is.
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  # Inspired by https://github.com/ros-simulation/gazebo_ros_pkgs/blob/foxy/gazebo_plugins/test/CMakeLists.txt
  ament_add_gtest(${test_name}
    test/${test_name}.cpp
  )
  target_include_directories(${test_name} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  ament_target_dependencies(${test_name}
  "gazebo_ros_simple_motor_msgs"
  "rclcpp"
)
endif()

ament_package()
