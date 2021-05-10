cmake_minimum_required(VERSION 3.0.2)
project(agitr)

find_package(catkin REQUIRED roscpp geometry_msgs turtlesim)

catkin_package()


include_directories(
include
${catkin_INCLUDE_DIRS}
)

add_executable(hello src/hello.cpp)
target_link_libraries(hello ${catkin_LIBRARIES})

add_executable(randomv src/random_vel.cpp)
target_link_libraries(randomv ${catkin_LIBRARIES})

add_executable(subscribe_pose src/subscriber.cpp)
target_link_libraries(subscribe_pose ${catkin_LIBRARIES})

add_executable(reverse_vel src/reverse_cmd_vel.cpp)
target_link_libraries(reverse_vel ${catkin_LIBRARIES})

add_executable(publevel_toggle src/publevel_toggle.cpp)
target_link_libraries(publevel_toggle ${catkin_LIBRARIES})