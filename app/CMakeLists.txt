cmake_minimum_required(VERSION 3.20)

add_executable(vo_app vo_app.cpp)
target_link_libraries(vo_app PUBLIC vo_interface vo_core)