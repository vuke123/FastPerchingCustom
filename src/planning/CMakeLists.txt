cmake_minimum_required(VERSION 3.0.2)
project(planning)

# Set compile options
add_compile_options(-std=c++14)
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall")

# Find required packages
find_package(Eigen3 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  traj_opt
  roscpp
  nodelet
  vis_utils
)
find_package(Python3 REQUIRED COMPONENTS Interpreter Development)

# Define catkin package
catkin_package(
  INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR}
  CATKIN_DEPENDS traj_opt roscpp nodelet
  LIBRARIES ${PROJECT_NAME}
  DEPENDS EIGEN3
)

# Include directories
include_directories(
  ${EIGEN3_INCLUDE_DIR}
  ${DECOMP_UTIL_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# Add the planning nodelet library
add_library(${PROJECT_NAME}_nodelet
  src/${PROJECT_NAME}_nodelet.cpp
)

# Link libraries
target_link_libraries(${PROJECT_NAME}_nodelet
  ${catkin_LIBRARIES}
  Python3::Python
)

# Add dependencies
add_dependencies(${PROJECT_NAME}_nodelet
  ${catkin_EXPORTED_TARGETS}
)
