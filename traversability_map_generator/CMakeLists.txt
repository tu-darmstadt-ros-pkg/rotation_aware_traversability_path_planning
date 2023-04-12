cmake_minimum_required(VERSION 3.0.2)
project(traversability_map_generator)

set(CMAKE_CXX_STANDARD 17)

find_package(catkin REQUIRED COMPONENTS roscpp l3_terrain_model_generator)

find_package(Boost REQUIRED COMPONENTS system)

find_package(OpenCV)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp l3_terrain_model_generator
)

include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

set(HEADERS
  include/${PROJECT_NAME}/plugins/std/generator/grid_map_merger.h
  )

set(SOURCES
  src/plugins/std/generator/grid_map_merger.cpp
  )

add_library(${PROJECT_NAME} ${SOURCES} ${HEADERS})

add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBRARIES})

install(FILES plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
  )

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  FILES_MATCHING PATTERN "*.launch"
  )