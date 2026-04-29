# object_tracking: detector + tracker (ByteTrack/OC-SORT), publishes boxes and optional Detection2DArray.
cmake_minimum_required(VERSION 3.10)
project(object_tracking LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(vision_msgs QUIET)
if(vision_msgs_FOUND)
  set(HAVE_VISION_MSGS 1)
endif()

if(NOT DEFINED OpenCV_DIR)
  set(OpenCV_DIR "/opt/opencv-spacemit/lib/cmake/opencv4")
endif()
find_package(OpenCV 4 REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc highgui NO_DEFAULT_PATH)
find_package(yaml-cpp REQUIRED)

set(SPACEMIT_ROBOT_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../../..")
set(VISION_SERVICE_INCLUDE_DIR "${SPACEMIT_ROBOT_ROOT}/output/staging/include")
set(VISION_SERVICE_LIBRARY "${SPACEMIT_ROBOT_ROOT}/output/staging/lib/libvision.so")

if(NOT EXISTS "${VISION_SERVICE_INCLUDE_DIR}/vision_service.h")
  message(FATAL_ERROR "vision_service.h not found")
endif()
if(NOT EXISTS "${VISION_SERVICE_LIBRARY}")
  message(FATAL_ERROR "vision library not found. Ensure output/staging is prepared first.")
endif()

add_executable(object_tracking_node
  src/object_tracking_node.cpp
  src/detection_utils.cpp
  src/image_utils.cpp
)
target_include_directories(object_tracking_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${VISION_SERVICE_INCLUDE_DIR}
)
if(HAVE_VISION_MSGS)
  target_compile_definitions(object_tracking_node PRIVATE HAVE_VISION_MSGS=1)
endif()

set(_deps ament_index_cpp rclcpp)
ament_target_dependencies(object_tracking_node ${_deps})
target_link_libraries(object_tracking_node
  sensor_msgs::sensor_msgs__rosidl_typesupport_cpp
  std_msgs::std_msgs__rosidl_typesupport_cpp
  ${VISION_SERVICE_LIBRARY}
  ${OpenCV_LIBS}
  yaml-cpp
)
if(vision_msgs_FOUND)
  target_link_libraries(object_tracking_node vision_msgs::vision_msgs__rosidl_typesupport_cpp)
endif()

get_filename_component(_opencv_lib_dir "${OpenCV_DIR}/../.." REALPATH)
get_filename_component(_vision_lib_dir "${VISION_SERVICE_LIBRARY}" DIRECTORY)
set_target_properties(object_tracking_node PROPERTIES
  INSTALL_RPATH "${_opencv_lib_dir};${_vision_lib_dir}"
  BUILD_WITH_INSTALL_RPATH FALSE
  BUILD_RPATH "${_opencv_lib_dir};${_vision_lib_dir}"
)

install(TARGETS object_tracking_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})
ament_package()
