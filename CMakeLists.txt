cmake_minimum_required(VERSION 2.8.3)
project(location_monitor)


find_package(Eigen3 REQUIRED) # add Eigen3 library

# add Eigen3 library
find_package(
    catkin REQUIRED COMPONENTS 
    message_generation
    roscpp
    rospy
    nav_msgs
    cmake_modules
)

add_message_files(
    FILES
    LandmarkDistance.msg
    MatrixResult.msg
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
    ${EIGEN3_INCLUDE_DIRS} # add eigen library
)


add_executable(location_monitor_node src/location_monitor.cpp)
add_executable(test_subscriber_node src/test_subscriber.cpp)
add_executable(monitor_client_node src/monitor_client.cpp)
add_executable(test_Eigen_node src/test_Eigen.cpp)

add_dependencies(location_monitor_node location_monitor_generate_messages_cpp)
add_dependencies(test_subscriber_node test_subscriber_generate_messages_cpp)
add_dependencies(monitor_client_node monitor_client_generate_messages_cpp)
add_dependencies(test_Eigen_node test_Eigen_generate_messages_cpp)


 target_link_libraries(location_monitor_node
        ${catkin_LIBRARIES})
 target_link_libraries(test_subscriber_node
        ${catkin_LIBRARIES})
 target_link_libraries(monitor_client_node
        ${catkin_LIBRARIES})
target_link_libraries(test_Eigen_node
        ${catkin_LIBRARIES})

#add_definitions(${EIGEN_DEFINITIONS})


