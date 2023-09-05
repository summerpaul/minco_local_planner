cmake_minimum_required(VERSION 3.5)



add_library(plugins SHARED plugins.cpp)
target_link_libraries(plugins plugin_loader)


add_executable(plugins_test plugins_test.cpp)
target_link_libraries(plugins_test plugin_loader)