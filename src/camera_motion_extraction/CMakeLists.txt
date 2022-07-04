CMAKE_MINIMUM_REQUIRED(VERSION 3.0)

# Set project name
PROJECT(camera_motion_extraction VERSION 1.0)

# find packages
set(DEPENDENCIES image_transport OpenCV sensor_msgs cv_bridge rosconsole)
macro(find_packages)
  foreach(package ${ARGN})
    find_package(${package} REQUIRED)
  endforeach()
endmacro()
find_packages(${DEPENDENCIES})

# Preprocessor
if(DEBUG_MODE)
  add_compile_definitions(DEBUG_MODE)
  unset(DEBUG_MODE CACHE)
endif()

# Add executable
if(TEST_IMAGE_SUB)
  add_executable(camera_motion_extraction test_image_subscriber.cpp image_subscriber.cpp)
  unset(TEST_IMAGE_SUB CACHE)
elseif(TEST_KEYPOINT_DETECTION)
  add_executable(camera_motion_extraction test_feature_detection.cpp feature_detection.cpp
                  image_subscriber.cpp)
  unset(TEST_KEYPOINT_DETECTION CACHE)
elseif(TEST_FEATURE_MATCHING)
  add_executable(camera_motion_extraction test_feature_matching.cpp feature_matching.cpp
                  feature_detection.cpp image_subscriber.cpp)
  unset(TEST_FEATURE_MATCHING CACHE)
else()
  add_executable(camera_motion_extraction test_motion_extraction.cpp motion_extraction.cpp
                  feature_detection.cpp feature_matching.cpp image_subscriber.cpp)
endif()

# Include
set(CAMERA_MOTION_EXTRACTION_INCLUDE ./include ${image_transport_INCLUDE_DIRS}
      ${OpenCV_INCLUDE_DIRS} ${sensor_msgs_INCLUDE_DIRS} ${cv_bridge_INCLUDE_DIRS}
      ${rosconsole_INCLUDE_DIRS})
macro(includes)
  foreach(package ${ARGN})
    target_include_directories(camera_motion_extraction PUBLIC ${package})
  endforeach()
endmacro()
includes(${CAMERA_MOTION_EXTRACTION_INCLUDE})

# Libraries
set(CAMERA_MOTION_EXTRACTION_LIBRARIES ${image_transport_LIBRARIES} ${OpenCV_LIBS}
${sensor_msgs_LIBRARIES} ${cv_bridge_LIBRARIES} ${rosconsole_LIBRARIES})
macro(libs)
  foreach(package ${ARGN})
    target_link_libraries(camera_motion_extraction ${package})
  endforeach()
endmacro()
libs(${CAMERA_MOTION_EXTRACTION_LIBRARIES})