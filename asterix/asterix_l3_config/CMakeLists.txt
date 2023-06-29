cmake_minimum_required(VERSION 3.0.2)
project(asterix_l3_config)

find_package(catkin REQUIRED COMPONENTS roscpp)


catkin_package(
  #  INCLUDE_DIRS include
  #  LIBRARIES asterix_l3_config
  CATKIN_DEPENDS roscpp
  #  DEPENDS system_lib
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  FILES_MATCHING PATTERN "*.launch"
  PATTERN "*~" EXCLUDE
  )

install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  FILES_MATCHING PATTERN "*"
  PATTERN "*~" EXCLUDE
  )

