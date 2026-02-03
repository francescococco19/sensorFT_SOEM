cmake_minimum_required(VERSION 3.16)
project(ft_listener CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_executable(ft_listener
  src/main.cpp
  src/SoemMaster.cpp
  src/SoemMaster.hpp
  src/FtSensor.cpp
  src/FtSensor.hpp
)

# root di SOEM (questa cartella è: SOEM/test/linux/ft_listener)
get_filename_component(SOEM_ROOT "${CMAKE_CURRENT_LIST_DIR}/../../.." ABSOLUTE)
set(SOEM_BUILD "${SOEM_ROOT}/build")

target_include_directories(ft_listener PRIVATE
  "${SOEM_ROOT}/soem"
  "${SOEM_ROOT}/osal"
  "${SOEM_ROOT}/osal/linux"
  "${SOEM_ROOT}/oshw"
  "${SOEM_ROOT}/oshw/linux"
)

# Link diretto alla libreria statica buildata da SOEM
target_link_libraries(ft_listener PRIVATE
  "${SOEM_BUILD}/libsoem.a"
  pthread rt
)
