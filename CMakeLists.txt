cmake_minimum_required(VERSION 3.0.2)
project(ft_sensor_ros)

# ROS Noetic supports C++14 well; C++11 is also fine. Use C++14 for safety.
add_compile_options(-std=c++17 -Wall -Wextra)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp geometry_msgs
)

# ---- SOEM paths (edit if your SOEM is elsewhere) -----------------------------
# Default: ~/SOEM
set(SOEM_ROOT "$ENV{HOME}/SOEM")
set(SOEM_BUILD "${SOEM_ROOT}/build")
set(SOEM_LIB   "${SOEM_BUILD}/libsoem.a")

# Basic sanity message (visible at configure time)
message(STATUS "SOEM_ROOT: ${SOEM_ROOT}")
message(STATUS "SOEM_LIB : ${SOEM_LIB}")

include_directories(
  ${catkin_INCLUDE_DIRS}

  # SOEM headers (from source tree)
  ${SOEM_ROOT}/soem
  ${SOEM_ROOT}/osal
  ${SOEM_ROOT}/osal/linux
  ${SOEM_ROOT}/oshw
  ${SOEM_ROOT}/oshw/linux
)

# ---- Node -------------------------------------------------------------------
add_executable(sensorFT_publisher
  src/sensorFT_publisher.cpp
  src/SoemMaster.cpp
  src/FtSensor.cpp
)

# Ensure SOEM library exists; fail early at build time if not found
add_custom_command(TARGET sensorFT_publisher PRE_BUILD
  COMMAND ${CMAKE_COMMAND} -E echo "Linking against: ${SOEM_LIB}"
)

target_link_libraries(sensorFT_publisher
  ${catkin_LIBRARIES}
  ${SOEM_LIB}
  pthread
  rt
)

add_dependencies(sensorFT_publisher
  ${catkin_EXPORTED_TARGETS}
)

# ---- Install (optional but recommended) --------------------------------------
install(TARGETS sensorFT_publisher
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
