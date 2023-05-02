cmake_minimum_required(VERSION 2.8.3)
project(gjk)

add_compile_options(-std=c++11 -g)

set(CMAKE_BUILD_TYPE "Relese")

include_directories(src)
include_directories(${Boost_INCLUDE_DIRS})

set(${PROJECT_NAME}_SOURCES
 src/core/test.cpp
 )

add_executable(
  ${PROJECT_NAME}_node
  ${${PROJECT_NAME}_SOURCES}
  )