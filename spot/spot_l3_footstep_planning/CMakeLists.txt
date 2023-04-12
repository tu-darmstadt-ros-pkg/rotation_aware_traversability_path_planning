cmake_minimum_required(VERSION 3.0.2)
project(spot_l3_footstep_planning)

set(CMAKE_CXX_STANDARD 17)

find_package(catkin REQUIRED COMPONENTS roscpp l3_footstep_planning traversability_rotation_planning_plugins spot_l3_plugins spot_l3_config aico_spot_description)

catkin_package(
  CATKIN_DEPENDS roscpp l3_footstep_planning traversability_rotation_planning_plugins spot_l3_plugins spot_l3_config aico_spot_description
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
