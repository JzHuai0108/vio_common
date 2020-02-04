SET(PROJECT_NAME vio_common)
PROJECT(${PROJECT_NAME})
CMAKE_MINIMUM_REQUIRED (VERSION 2.8.3)

IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE Release)
ENDIF()

SET(CMAKE_VERBOSE_MAKEFILE OFF)
SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${PROJECT_SOURCE_DIR}/cmake_modules/")

OPTION(USE_ROS "Build this package with catkin?" OFF)
SET(USE_GPSTK_MSG "Build the package with the gpstk library. If true, "
    "the GPSTK_INCLUDE_DIRS and GPSTK_LIB need to be provided.\nEg."
    "-DGPSTK_INCLUDE_DIRS /workspace/gpstk/include -DGPSTK_LIB "
    "/workspace/gpstk/lib/libgpstk.so")
OPTION(USE_GPSTK "${USE_GPSTK_MSG}" OFF)

# Set build flags. Set IS_ARM on odroid board as environment variable
SET(CMAKE_CXX_FLAGS "-Wall -D_LINUX -D_REENTRANT -march=native -Wno-unused-variable -Wno-unused-but-set-variable -Wno-unknown-pragmas")
IF(DEFINED ENV{ARM_ARCHITECTURE})
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -mfpu=neon -march=armv7-a")
ELSE()
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -mmmx -msse -msse -msse2 -msse3 -mssse3")
ENDIF()
IF(CMAKE_COMPILER_IS_GNUCC)
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
ELSE()
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
ENDIF()
SET(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS} -O3 -fsee -fomit-frame-pointer -fno-signed-zeros -fno-math-errno -funroll-loops")

# Add plain cmake packages 
find_package(OpenCV REQUIRED)
FIND_PACKAGE(Eigen REQUIRED)

# Include dirs
LIST(APPEND vio_common_INCLUDE_DIR
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${Eigen_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

IF(${USE_ROS})
  FIND_PACKAGE(catkin REQUIRED COMPONENTS roscpp cmake_modules)
  LIST(APPEND vio_common_INCLUDE_DIR ${catkin_INCLUDE_DIRS})
  catkin_package(
    DEPENDS Eigen OpenCV
    CATKIN_DEPENDS roscpp
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
  )
ELSE()
  SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
  SET(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
ENDIF()


# Set Sourcefiles
LIST(APPEND SOURCEFILES 
src/Sample.cpp
src/utils.cpp
src/TimeGrabber.cpp
src/maths_utils.cpp
src/ImuGrabber.cpp
src/GpsGrabber.cpp
src/CsvReader.cpp
src/FrameGrabber.cpp
src/eigen_utils.cpp
include/vio/PointStatistics.h
)

IF(${USE_GPSTK})
  ADD_DEFINITIONS(-DHAVE_GPSTK)
  LIST(APPEND vio_common_INCLUDE_DIR ${GPSTK_INCLUDE_DIRS})
ENDIF()

# Create library
INCLUDE_DIRECTORIES(${vio_common_INCLUDE_DIR})
ADD_LIBRARY(${PROJECT_NAME} SHARED ${SOURCEFILES})
TARGET_LINK_LIBRARIES(${PROJECT_NAME}
  ${OpenCV_LIBS})

IF(${USE_ROS})
  TARGET_LINK_LIBRARIES(${PROJECT_NAME} ${catkin_LIBRARIES})
ENDIF()

IF(${USE_GPSTK})
  TARGET_LINK_LIBRARIES(${PROJECT_NAME} ${GPSTK_LIB})
ENDIF()

# Tests
ADD_EXECUTABLE(test_csv_reader test/test_csv_reader.cpp)
TARGET_LINK_LIBRARIES(test_csv_reader ${PROJECT_NAME})

ADD_EXECUTABLE(test_gps_grabber test/test_gps_grabber.cpp)
TARGET_LINK_LIBRARIES(test_gps_grabber ${PROJECT_NAME})

find_package(GTest REQUIRED)
include_directories(${GTEST_INCLUDE_DIRS})

add_executable(video2image app/video2images_kalibr.cpp )
target_link_libraries(video2image ${OpenCV_LIBS})

add_executable(vio_common_test test/test_main.cpp
    test/test_time_grabber.cpp
    test/test_utils.cpp
    test/test_frame_grabber.cpp
    test/test_imu_grabber.cpp
    test/test_eigen_utils.cpp
    )
target_link_libraries(vio_common_test ${PROJECT_NAME} ${GTEST_LIBRARIES} glog pthread)

################################################################################
# Create the vio_commonConfig.cmake file for other cmake projects.
IF(NOT ${USE_ROS})
  # Export package for use from the build tree
  EXPORT( PACKAGE vio_common )

  # Create the vio_commonConfig.cmake file for other cmake projects.
  # ... for the build tree
  SET( CONFIG_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR})
  SET( CONFIG_DIR ${CMAKE_CURRENT_BINARY_DIR})
  SET(vio_common_LIBRARY ${PROJECT_SOURCE_DIR}/lib/libvio_common.so)
  SET(vio_common_LIBRARY_DIR ${PROJECT_SOURCE_DIR}/lib)
  CONFIGURE_FILE( ${CMAKE_CURRENT_SOURCE_DIR}/vio_commonConfig.cmake.in
      ${CMAKE_CURRENT_BINARY_DIR}/vio_commonConfig.cmake @ONLY IMMEDIATE )
  
  # ... for the install tree
  SET( CMAKECONFIG_INSTALL_DIR lib/cmake/vio_common )
  FILE( RELATIVE_PATH REL_INCLUDE_DIR
      "${CMAKE_INSTALL_PREFIX}/${CMAKECONFIG_INSTALL_DIR}"
      "${CMAKE_INSTALL_PREFIX}/include" )

  SET( vio_common_INCLUDE_DIR "\${vio_common_CMAKE_DIR}/${REL_INCLUDE_DIR}" ) 
  SET( vio_common_LIBRARY_DIR "\${vio_common_CMAKE_DIR}/../.." )
  SET( vio_common_LIBRARY "\${vio_common_CMAKE_DIR}/../../libvio_common.so" )
  SET( CONFIG_SOURCE_DIR )
  SET( CONFIG_DIR )
  CONFIGURE_FILE( ${CMAKE_CURRENT_SOURCE_DIR}/vio_commonConfig.cmake.in
      ${CMAKE_CURRENT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/vio_commonConfig.cmake @ONLY )
  INSTALL(FILES "${CMAKE_CURRENT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/vio_commonConfig.cmake"
        DESTINATION ${CMAKECONFIG_INSTALL_DIR} )

  INSTALL(DIRECTORY include/vio DESTINATION ${CMAKE_INSTALL_PREFIX}/include FILES_MATCHING PATTERN "*.h" )
  INSTALL(TARGETS ${PROJECT_NAME} DESTINATION ${CMAKE_INSTALL_PREFIX}/lib )
ENDIF()
