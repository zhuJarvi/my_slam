cmake_minimum_required(VERSION 3.20)

add_executable(vo_app vo_app.cpp)
target_sources(vo_app PRIVATE ${CMAKE_SOURCE_DIR}/src/save/recorder.cpp)
target_link_libraries(vo_app PUBLIC vo_interface vo_core)

if(USE_PANGOLIN)
	add_executable(vo_pangolin_viewer trajectory_viewer.cpp)
	target_link_libraries(vo_pangolin_viewer PUBLIC ${Pangolin_LIBRARIES})
endif()

add_executable(vo_live_stereo live_stereo_vo.cpp)
target_sources(vo_live_stereo PRIVATE ${CMAKE_SOURCE_DIR}/src/save/recorder.cpp)
target_link_libraries(vo_live_stereo PUBLIC vo_interface vo_core)