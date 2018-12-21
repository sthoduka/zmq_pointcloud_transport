cmake_minimum_required(VERSION 2.8.2)
project(zmq_pointcloud_transport)


set(CMAKE_CXX_STANDARD 11)
find_package(PCL REQUIRED)

include_directories(
    ${PCL_INCLUDE_DIRS}
    include
)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(example_pub
    examples/example_pub.cpp
)

add_executable(example_sub
    examples/example_sub.cpp
)

add_executable(example_server
    examples/example_server.cpp
)

add_executable(example_client
    examples/example_client.cpp
)

target_link_libraries(example_pub
    zmq
    ${PCL_LIBRARIES}
)
target_link_libraries(example_sub
    zmq
    ${PCL_LIBRARIES}
)

target_link_libraries(example_server
    zmq
    ${PCL_LIBRARIES}
)
target_link_libraries(example_client
    zmq
    ${PCL_LIBRARIES}
)

