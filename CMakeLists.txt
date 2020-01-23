cmake_minimum_required(VERSION 2.8.3)
project(Socket)

add_compile_options(-std=c++11)

find_package(
  catkin REQUIRED 
  COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs
)
###########
## Build ##
###########

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)


add_executable(client_test src/client_test.cpp)
target_link_libraries(client_test ${catkin_LIBRARIES})

add_executable(server_test src/server_test.cpp)
target_link_libraries(server_test ${catkin_LIBRARIES})

add_executable(server_twin src/server_twin.cpp)
target_link_libraries(server_twin ${catkin_LIBRARIES})

add_executable(server_twin_cross src/server_twin_cross.cpp)
target_link_libraries(server_twin_cross ${catkin_LIBRARIES})

add_executable(client_pos src/client_pos.cpp)
target_link_libraries(client_pos ${catkin_LIBRARIES})

add_executable(server_point_cloud src/server_point_cloud.cpp)
target_link_libraries(server_point_cloud ${catkin_LIBRARIES})

add_executable(client_point_cloud src/client_point_cloud.cpp)
target_link_libraries(client_point_cloud ${catkin_LIBRARIES})

add_executable(server_twin_point_cloud src/server_twin_point_cloud.cpp)
target_link_libraries(server_twin_point_cloud ${catkin_LIBRARIES})

add_executable(client_test_point_cloud  src/client_test_point_cloud.cpp)
target_link_libraries(client_test_point_cloud  ${catkin_LIBRARIES})