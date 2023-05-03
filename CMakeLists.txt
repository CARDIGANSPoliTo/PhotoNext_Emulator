cmake_minimum_required(VERSION 2.8.9)
project (smartscanemu)

set(smartscanemu_VERSION_MAJOR 0)
set(smartscanemu_VERSION_MINOR 1)

set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_CXX_FLAGS "-Wall -std=c++11")

find_package (Threads REQUIRED)
find_library(LIBUTILS     utils)
find_library(LIBSMARTSCAN smartscan)

if ( NOT LIBUTILS )
  message(FATAL_ERROR "utils library not found")
endif()

if(NOT LIBSMARTSCAN)
  message(FATAL_ERROR "smartscan library not found")
endif()

include_directories(
  include
)

add_executable(smartscanemu src/smartscanemu.c)
target_link_libraries(smartscanemu -lutils)
target_link_libraries(smartscanemu -lsmartscan)
target_link_libraries(smartscanemu ${CMAKE_THREAD_LIBS_INIT})

# install(TARGETS smartscanemu DESTINATION bin)
