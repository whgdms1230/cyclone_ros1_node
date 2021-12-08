cmake_minimum_required(VERSION 3.5.0)

project(cyclone_ros1_node)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  # we dont use add_compile_options with pedantic in message packages
  # because the Python C extensions dont comply with it
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic")
endif()

include(GNUInstallDirs)

#===============================================================================

find_package(catkin QUIET COMPONENTS
  roscpp
  std_srvs
  std_msgs
  message_generation
)

add_message_files(
  FILES
  IntNumber.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs  # Or other packages containing msgs
)

if (catkin_FOUND)

  find_package(cyclone_bridge REQUIRED)

  catkin_package()

  #=============================================================================

  add_executable(cyclone_ros1_node
    src/main.cpp
    src/ROS1Node.cpp
    src/ROS1NodeConfig.cpp
  )
  target_link_libraries(cyclone_ros1_node
    ${cyclone_bridge_LIBRARIES}
    ${catkin_LIBRARIES}
  )
  target_include_directories(cyclone_ros1_node
    PRIVATE
      ${cyclone_bridge_INCLUDE_DIRS}
      ${catkin_INCLUDE_DIRS}
  )

  install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
  )

else()
  message("catkin not found so skipping this ROS1 package")
endif()