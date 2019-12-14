cmake_minimum_required(VERSION 2.8.3)
project(wit_imu_driver)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
)

find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(wit_imu_driver src/wit_imu_driver_node.cpp)
add_dependencies(wit_imu_driver ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(wit_imu_driver
  ${catkin_LIBRARIES}
)

find_package(roslint REQUIRED)
set(ROSLINT_CPP_OPTS "--filter=+,-build/c++11")
roslint_cpp()
