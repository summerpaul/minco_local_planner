add_executable(demo_node demo_node.cpp demo.cpp)
target_link_libraries(demo_node 
    module_manager 
    visualizer
    ${catkin_LIBRARIES}
    )