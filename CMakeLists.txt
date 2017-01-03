cmake_minimum_required(VERSION 3.2.0 FATAL_ERROR)
project(nbody)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(CMAKE_COMPILER_IS_GNUCC)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fpermissive")
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS 1)

find_package(PCL 1.3 REQUIRED COMPONENTS common visualization)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "bin/")
add_executable(nbody src/main.cpp)
target_link_libraries(nbody ${PCL_COMMON_LIBRARIES}
  ${PCL_VISUALIZATION_LIBRARIES})
