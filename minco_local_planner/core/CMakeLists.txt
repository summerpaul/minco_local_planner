cmake_minimum_required(VERSION 3.10.0)
project(minco_local_planner_core)





include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}
    
    )


add_subdirectory(config_manager)
add_subdirectory(module_manager)
add_subdirectory(runtime_manager)
add_subdirectory(map_manager)
add_subdirectory(safety_manager)
add_subdirectory(trajectory_optimizer)
add_subdirectory(path_search)
add_subdirectory(plan_manager)
# add_subdirectory(planner_factory)
add_subdirectory(3rd-party)
add_subdirectory(route)
add_subdirectory(plugins)
