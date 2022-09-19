set(PROJECT_NAME vio_common)
project(${PROJECT_NAME})
cmake_minimum_required(VERSION 2.8.3)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_VERBOSE_MAKEFILE OFF)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${PROJECT_SOURCE_DIR}/cmake_modules/")

option(USE_ROS "Build this package with catkin?" ON)
set(USE_GPSTK_MSG "Build the package with the gpstk library. If true, "
    "the GPSTK_INCLUDE_DIRS and GPSTK_LIB need to be provided.\nEg."
    "-DGPSTK_INCLUDE_DIRS /workspace/gpstk/include -DGPSTK_LIB "
    "/workspace/gpstk/lib/libgpstk.so")
option(USE_GPSTK "${USE_GPSTK_MSG}" OFF)

# Set build flags. Set IS_ARM on odroid board as environment variable
set(CMAKE_CXX_FLAGS "-Wall -D_LINUX -D_REENTRANT -Wno-unused-variable -Wno-unused-but-set-variable -Wno-unknown-pragmas")
if(DEFINED ENV{ARM_ARCHITECTURE})
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -mfpu=neon -march=armv7-a")
else()
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -mmmx -msse -msse -msse2 -msse3 -mssse3")
endif()
if(CMAKE_COMPILER_IS_GNUCC)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
endif()

# Add plain cmake packages 
find_package(OpenCV REQUIRED)
find_package(Eigen REQUIRED)

# Include dirs
list(APPEND vio_common_INCLUDE_DIR
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${EIGEN_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS}
)

if(${USE_ROS})
  find_package(catkin REQUIRED COMPONENTS roscpp cmake_modules)
  list(APPEND vio_common_INCLUDE_DIR ${catkin_INCLUDE_DIRS})
  catkin_package(
    DEPENDS
    CATKIN_DEPENDS roscpp
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
  )
else()
  set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR}/bin)
  set(LIBRARY_OUTPUT_PATH ${CMAKE_BINARY_DIR}/lib)

  # Install to (e.g. /usr/local/) and look for libraries there.
  set(CMAKE_INSTALL_RPATH ${CMAKE_INSTALL_PREFIX}/lib)
  set(CMAKE_INSTALL_RPATH_USE_LINK_PATH FALSE)
endif()


# Set Sourcefiles
list(APPEND SOURCEFILES
  include/vio/PointStatistics.h
  src/CsvReader.cpp
  src/FrameGrabber.cpp
  src/GpsGrabber.cpp
  src/ImuGrabber.cpp
  src/Sample.cpp
  src/TimeGrabber.cpp
  src/VimapContainer.cpp
  src/derivedLinePatterns.cpp
  src/eigen_utils.cpp
  src/maths_utils.cpp
  src/utils.cpp)

if(${USE_GPSTK})
  add_definitions(-DHAVE_GPSTK)
  list(APPEND vio_common_INCLUDE_DIR ${GPSTK_INCLUDE_DIRS})
endif()

# Create library
add_library(${PROJECT_NAME} SHARED ${SOURCEFILES})
target_include_directories(${PROJECT_NAME} PUBLIC ${vio_common_INCLUDE_DIR})
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})

if(${USE_ROS})
  target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
endif()

if(${USE_GPSTK})
  target_link_libraries(${PROJECT_NAME} ${GPSTK_LIB})
endif()

add_executable(test_gps_grabber test/test_gps_grabber.cpp)
target_link_libraries(test_gps_grabber ${PROJECT_NAME})
# set_target_properties(test_gps_grabber PROPERTIES INSTALL_RPATH_USE_LINK_PATH TRUE)

add_executable(video2image app/video2images_kalibr.cpp )
target_link_libraries(video2image ${OpenCV_LIBS})
# set_target_properties(video2image PROPERTIES INSTALL_RPATH_USE_LINK_PATH TRUE)

add_executable(maplab2tumtraj app/maplab2tumtraj.cpp)
target_link_libraries(maplab2tumtraj ${PROJECT_NAME})

set(TEST_DATA_PATH "${CMAKE_CURRENT_SOURCE_DIR}/test/data")
configure_file(test/test_config.h.in test/test_config.h)
include_directories(${CMAKE_CURRENT_BINARY_DIR})

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(vio_common_test test/test_main.cpp
    test/test_csv_reader.cpp
    test/test_time_grabber.cpp
    test/test_utils.cpp
    test/test_frame_grabber.cpp
    test/test_imu_grabber.cpp
    test/test_eigen_utils.cpp
    test/test_vimap_container.cpp)
  target_link_libraries(vio_common_test ${PROJECT_NAME})
endif()

################################################################################
# Create the vio_commonConfig.cmake file for other cmake projects.
if(NOT ${USE_ROS})
  # Export package for use from the build tree
  export(PACKAGE vio_common)

  # Create the vio_commonConfig.cmake file for other cmake projects.
  # ... for the build tree
  set(CONFIG_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR})
  set(CONFIG_DIR ${CMAKE_CURRENT_BINARY_DIR})
  set(vio_common_LIBRARY ${CMAKE_BINARY_DIR}/lib/libvio_common.so)
  set(vio_common_LIBRARY_DIR ${CMAKE_BINARY_DIR}/lib)
  configure_file(${CMAKE_CURRENT_SOURCE_DIR}/vio_commonConfig.cmake.in
      ${CMAKE_CURRENT_BINARY_DIR}/vio_commonConfig.cmake @ONLY IMMEDIATE)
  
  # ... for the install tree
  set(CMAKECONFIG_INSTALL_DIR lib/cmake/vio_common)
  file(RELATIVE_PATH REL_INCLUDE_DIR
      "${CMAKE_INSTALL_PREFIX}/${CMAKECONFIG_INSTALL_DIR}"
      "${CMAKE_INSTALL_PREFIX}/include")

  set(vio_common_INCLUDE_DIR "\${vio_common_CMAKE_DIR}/${REL_INCLUDE_DIR}") 
  set(vio_common_LIBRARY_DIR "\${vio_common_CMAKE_DIR}/../..")
  set(vio_common_LIBRARY "\${vio_common_CMAKE_DIR}/../../libvio_common.so")
  set(CONFIG_SOURCE_DIR)
  set(CONFIG_DIR)
  configure_file(${CMAKE_CURRENT_SOURCE_DIR}/vio_commonConfig.cmake.in
                 ${CMAKE_CURRENT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/vio_commonConfig.cmake @ONLY)
  install(FILES "${CMAKE_CURRENT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/vio_commonConfig.cmake"
          DESTINATION ${CMAKECONFIG_INSTALL_DIR})

  set(DefaultScriptPermissions OWNER_READ OWNER_WRITE OWNER_EXECUTE    GROUP_READ         GROUP_EXECUTE   WORLD_READ       WORLD_EXECUTE )
  install(DIRECTORY include/vio DESTINATION ${CMAKE_INSTALL_PREFIX}/include FILES_MATCHING PATTERN "*.h")
  install(DIRECTORY python/ DESTINATION ${CMAKE_INSTALL_PREFIX}/bin FILE_PERMISSIONS ${DefaultScriptPermissions} FILES_MATCHING PATTERN "*.py" PATTERN "test" EXCLUDE)
  install(TARGETS ${PROJECT_NAME} DESTINATION ${CMAKE_INSTALL_PREFIX}/lib)
  install(TARGETS test_gps_grabber video2image DESTINATION ${CMAKE_INSTALL_PREFIX}/bin)
endif()
