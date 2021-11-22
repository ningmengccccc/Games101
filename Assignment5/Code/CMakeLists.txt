cmake_minimum_required(VERSION 3.10)
project(RayTracing)

set(CMAKE_CXX_STANDARD 17)

set(EIGEN_DIR "D:/eigen")
if(NOT EXISTS ${EIGEN_DIR})
    message(FATAL_ERROR "Please check that the set Eigen directory is valid!")
endif()
include_directories(${EIGEN_DIR})

find_package(OpenCV 3.4.13 REQUIRED)
include_directories( ${OpenCV_INCLUDE_DIRS} )

if(CMAKE_COMPILER_IS_GNUCXX)
    message(STATUS "GCC detected, adding compile flags")
    target_compile_options(RayTracing PUBLIC -Wall -Wextra -pedantic -Wshadow -Wreturn-type -fsanitize=undefined)
    target_compile_features(RayTracing PUBLIC cxx_std_17)
endif(CMAKE_COMPILER_IS_GNUCXX)

add_executable(RayTracing main.cpp Object.hpp Vector.hpp Sphere.hpp global.hpp Triangle.hpp Scene.cpp Scene.hpp Light.hpp Renderer.cpp)
target_link_libraries(RayTracing PUBLIC -fsanitize=undefined  ${OpenCV_LIBS})
