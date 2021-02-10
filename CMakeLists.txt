cmake_minimum_required(VERSION 2.8.3)
project(gjk)

add_compile_options(-std=c++11 -g)

set(CMAKE_BUILD_TYPE "Relese")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is uesd, also find other catkin packages
find_package(
  catkin REQUIRED COMPONENTS
  message_runtime
  message_generation
  roscpp
  rospy
  std_msgs
  )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS src ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS}
  LIBRARIES ${PROJECT_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES}
  CATKIN_DEPENDS ${CATKIN_COMPONENTS}
  )

execute_process(
  COMMAND git rev-parse --short HEAD
  WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
  OUTPUT_VARIABLE GIT_COMMIT_HASH
  OUTPUT_STRIP_TRAILING_WHITESPACE
  )

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(src)
include_directories(${catkin_INCLUDE_DIRS})
include_directories(${Boost_INCLUDE_DIRS})

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
#aux_source_directory()

#set(${PROJECT_NAME}_SOURCES
#  src/core/test.cpp
#  )

set(${PROJECT_NAME}_SOURCES
   src/interface/test.cpp
   )

add_executable(
  ${PROJECT_NAME}_node
  ${${PROJECT_NAME}_SOURCES}
  )

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  )