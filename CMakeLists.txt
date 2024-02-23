cmake_minimum_required(VERSION 3.13)

project(smpl)

# Options
set(CMAKE_CXX_STANDARD 14)
set(LIBRARY_DIR ${CMAKE_CURRENT_SOURCE_DIR}/Libs CACHE PATH "Path to lib folder")
set(Eigen3_DIR ${LIBRARY_DIR}/Eigen/share/eigen3/cmake CACHE PATH "Path to installed Eigen")
#set(Ceres_DIR ${LIBRARY_DIR}/Ceres/CMake CACHE PATH "Path to installed Ceres")
set(Ceres_DIR ${LIBRARY_DIR}/ceres/lib/cmake/Ceres CACHE PATH "Path to installed Ceres")
set(jsoncpp_DIR /usr/lib/x86_64-linux-gnu/cmake/jsoncpp CACHE PATH "Path to installed jsoncpp")

find_package(Eigen3 REQUIRED)
find_package(Ceres REQUIRED)
#find_package(jsoncpp CONFIG REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(JSONCPP jsoncpp)
link_libraries(${JSONCPP_LIBRARIES})

add_definitions("-D_DISABLE_EXTENDED_ALIGNED_STORAGE")

add_executable(${PROJECT_NAME} 
    src/main.cpp 
    src/smpl.cpp 
    src/smpl.h
    src/utils/io.h
    src/utils/blend_shape_reader.h
    src/utils/blend_shape_reader.cpp

)

target_link_libraries(${PROJECT_NAME} ceres)
target_link_libraries(${PROJECT_NAME} ${JSONCPP_LIBRARIES})




