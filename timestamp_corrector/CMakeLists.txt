cmake_minimum_required(VERSION 3.5.1)
project(TimestampCorrector LANGUAGES CXX)
set(CMAKE_BUILD_TYPE "Release")

find_package(pybind11 REQUIRED)  # pip install pybind11 (or apt)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include 
    /usr/include/python3.8)
# include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include ${pybind11_INCLUDE_DIRS})
pybind11_add_module(TimestampCorrector src/exportTimestampCorrector.cpp)

# (Optional but handy) put the .so right in the build dir
set_target_properties(TimestampCorrector PROPERTIES
  LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)