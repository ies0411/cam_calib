cmake_minimum_required(VERSION 3.0.2)
project(cam_calib)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  cv_bridge
  roscpp
  sensor_msgs
  std_msgs
)

find_package(OpenCV REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(JSONCPP jsoncpp)

add_message_files(
  FILES
  stereo.msg
)
generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs

  # Or other packages containing msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES cam_calib opencv
  CATKIN_DEPENDS roscpp sensor_msgs std_msgs cv_bridge message_generation

  # DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}

  # ${JSONCPP_INCLUDE_DIRS}
)

add_executable(mono_cam_calib src/mono_cam_calib.cpp)
add_executable(stereo_cam_calib src/stereo_cam_calib.cpp)

add_dependencies(mono_cam_calib ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(stereo_cam_calib ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(mono_cam_calib
  ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${JSONCPP_LIBRARIES}
)

target_link_libraries(stereo_cam_calib
  ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${JSONCPP_LIBRARIES}
)
