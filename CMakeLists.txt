project(Path_Planning)

cmake_minimum_required (VERSION 3.5)

add_definitions(-std=c++11)

set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS, "${CXX_FLAGS}")

set(sources src/obstacle_tracker.cpp src/lane_follow.cpp src/lane_change.cpp src/lane_keep.cpp src/jmt.cpp src/circle.cpp src/virtual_driver.cpp src/vehicle.cpp src/obstacle.cpp src/road.cpp src/map.cpp src/main.cpp)


if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")

include_directories(/usr/local/include)
include_directories(/usr/local/opt/openssl/include)
link_directories(/usr/local/lib)
link_directories(/usr/local/opt/openssl/lib)
link_directories(/usr/local/Cellar/libuv/1.19.1/lib)

endif(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")

add_executable(path_planning ${sources})

target_link_libraries(path_planning z ssl uv uWS)
