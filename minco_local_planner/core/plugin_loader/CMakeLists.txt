cmake_minimum_required(VERSION 3.5)
project(plugin_loader CXX)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

add_compile_options(-fPIC)



set(${PROJECT_NAME}_SRCS
    plugin_loader.cpp
    plugin_loader_core.cpp
    meta_object.cpp
    multi_library_plugin_loader.cpp
    console.cpp
    )
set(${PROJECT_NAME}_HDRS
    plugin_loader.hpp
    plugin_loader_core.hpp
    exceptions.hpp
     meta_object.hpp
     multi_library_plugin_loader.hpp
     register_macro.hpp
    )

if (UNIX)
    set(${PROJECT_NAME}_SRCS
        ${${PROJECT_NAME}_SRCS}
        shared_library_UNIX.cpp
        )
endif (UNIX)

add_library(${PROJECT_NAME} ${${PROJECT_NAME}_SRCS} ${${PROJECT_NAME}_HDRS})
target_link_libraries(${PROJECT_NAME}  dl)
target_compile_definitions(${PROJECT_NAME} PRIVATE "plugin_loader_BUILDING_DLL")


