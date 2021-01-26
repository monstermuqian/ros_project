cmake_minimum_required(VERSION 2.8.3)
project(location_monitor)



find_package(
    catkin REQUIRED COMPONENTS
    message_generation
    roscpp
    rospy
    nav_msgs
)

add_message_files(
    FILES
    LandmarkDistance.msg
)

add_service_files(
    FILES
    GetClosest.srv
    GetDistance.srv
)

generate_messages(
    DEPENDENCIES
)


catkin_package(
    CATKIN_DEPENDS 
    message_runtime
    roscpp
    rospy
    nav_msgs
)



include_directories(
    ${catkin_INCLUDE_DIRS}
)


add_executable(location_monitor_node src/location_monitor.cpp)
add_executable(test_subscriber_node src/test_subscriber.cpp)
add_executable(monitor_client_node src/monitor_client.cpp)

add_dependencies(location_monitor_node location_monitor_generate_messages_cpp)
add_dependencies(test_subscriber_node test_subscriber_generate_messages_cpp)
add_dependencies(monitor_client_node monitor_client_generate_messages_cpp)


 target_link_libraries(location_monitor_node
        ${catkin_LIBRARIES})
 target_link_libraries(test_subscriber_node
        ${catkin_LIBRARIES})
 target_link_libraries(monitor_client_node
        ${catkin_LIBRARIES})