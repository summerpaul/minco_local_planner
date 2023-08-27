add_library(module_manager SHARED module_manager.cpp)
target_link_libraries(module_manager 
    config_manager
    spdlog
    runtime_manager
    map_manager
    )