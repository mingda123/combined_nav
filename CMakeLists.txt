cmake_minimum_required(VERSION 3.16)
project(combined_nav LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(MSVC)
    add_compile_options(/utf-8)
endif()

if(MINGW)
    add_compile_options(-Wa,-mbig-obj)
endif()

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_LIST_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_LIST_DIR}/bin)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_CURRENT_LIST_DIR}/lib)

set(IMU_PATH ${CMAKE_SOURCE_DIR}/src/IMU)
set(COMMON_PATH ${CMAKE_SOURCE_DIR}/src/COMMON)

set(EIGEN3_INCLUDE_DIR "D:/my_it/Eigeon/eigen-3.4.0" CACHE PATH "包含 Eigen/Dense 的目录")
include_directories(${EIGEN3_INCLUDE_DIR})

add_subdirectory(src)
add_subdirectory(external)
