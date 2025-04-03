cmake_minimum_required(VERSION 3.5)

set(CXX_DISABLE_WERROR 1)
set(CMAKE_CXX_STANDARD 11)

set(PROJECT_NAME MinimumJerkController)
set(PROJECT_DESCRIPTION "MinimumJerkController")
set(PROJECT_URL "")

project(
  ${PROJECT_NAME}
  LANGUAGES CXX
  VERSION 1.0.0)
include(CTest)
enable_testing()

if(NOT TARGET mc_rtc::mc_control)
  find_package(mc_rtc REQUIRED)
endif()

# To use ros
find_package(catkin REQUIRED COMPONENTS roscpp)
catkin_package(CATKIN_DEPENDS geometry_msgs roscpp roslib std_msgs)
message("=========== Catkin include directories : ${catkin_INCLUDE_DIRS}")
include_directories(include ${catkin_INCLUDE_DIRS} $ENV{HOME}/.local/include)
link_directories(${catkin_LIBRARY_DIRS} $ENV{HOME}/.local/lib)

find_package(MinimumJerkTask REQUIRED)

add_subdirectory(src)

# if(BUILD_TESTING) add_subdirectory(tests) endif()

configure_file(etc/MinimumJerkController.in.yaml
               "${CMAKE_CURRENT_BINARY_DIR}/etc/MinimumJerkController.yaml")
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/etc/MinimumJerkController.yaml"
        DESTINATION "${MC_CONTROLLER_RUNTIME_INSTALL_PREFIX}/etc")
