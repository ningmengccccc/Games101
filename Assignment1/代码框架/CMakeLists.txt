cmake_minimum_required(VERSION 3.10)
project(Rasterizer)

set(EIGEN_DIR "D:/eigen")
if(NOT EXISTS ${EIGEN_DIR})
    message(FATAL_ERROR "Please check that the set Eigen directory is valid!")
endif()
include_directories(${EIGEN_DIR})

find_package(OpenCV 3.4.13 REQUIRED)
include_directories( ${OpenCV_INCLUDE_DIRS} )

add_executable(Rasterizer main.cpp rasterizer.hpp rasterizer.cpp global.hpp Triangle.hpp Triangle.cpp)
target_link_libraries(Rasterizer ${OpenCV_LIBS})
