cmake_minimum_required(VERSION 3.20)

project(myslam)

set(CMAKE_BUILD_TYPE Release)

set(CMAKE_CXX_FLAGS "-std=c++17 -Wall")
set(CMAKE_CXX_FLAGS_RELEASE  "-std=c++17 -O3 -fopenmp -pthread")

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_BINARY_DIR}/lib)

# ====================================================

include_directories(${CMAKE_SOURCE_DIR}/include)
include_directories(${CMAKE_SOURCE_DIR}/IncludeMe)

# Eigen
include_directories("/usr/include/eigen3")

# OpenCV
find_package(OpenCV 4 REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
link_libraries(${OpenCV_LIBRARIES})

find_package(Ceres REQUIRED)
include_directories(${Ceres_INCLUDE_DIRS})
link_libraries(Ceres::ceres)

# pangolin
find_package(Pangolin REQUIRED)
include_directories(${Pangolin_INCLUDE_DIRS})
link_libraries(${Pangolin_LIBRARIES})

# Sophus
find_package(Sophus REQUIRED)
include_directories(${Sophus_INCLUDE_DIRS})
link_libraries(Sophus::Sophus)

# G2O
find_package(g2o REQUIRED)
include_directories(${g2o_INCLUDE_DIR})
link_libraries(
    g2o_core 
    g2o_stuff 
    g2o_types_sba 
    g2o_solver_csparse 
    g2o_csparse_extension
) 

# csparse
find_package(CSparse REQUIRED)
include_directories(${CSPARSE_INCLUDE_DIR})
link_libraries(${CSPARSE_LIBRARY})

#spdlog
find_package(spdlog REQUIRED)
link_libraries(spdlog::spdlog)


add_library(vo_interface SHARED
${CMAKE_SOURCE_DIR}/src/interface/camera.cpp
${CMAKE_SOURCE_DIR}/src/interface/viewer.cpp
${CMAKE_SOURCE_DIR}/src/interface/dataset.cpp

)

add_library(vo_core SHARED
${CMAKE_SOURCE_DIR}/src/stereo_vo/frontend.cpp
${CMAKE_SOURCE_DIR}/src/stereo_vo/backend.cpp
${CMAKE_SOURCE_DIR}/src/stereo_vo/map.cpp
${CMAKE_SOURCE_DIR}/src/stereo_vo/struct_base/frame.cpp
${CMAKE_SOURCE_DIR}/src/stereo_vo/struct_base/feature.cpp
${CMAKE_SOURCE_DIR}/src/stereo_vo/struct_base/mappoint.cpp
)
target_link_libraries(vo_core PUBLIC vo_interface)

add_subdirectory(${CMAKE_SOURCE_DIR}/app)
