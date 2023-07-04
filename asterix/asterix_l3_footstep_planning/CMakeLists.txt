cmake_minimum_required(VERSION 3.0.2)
project(asterix_l3_footstep_planning)

set(CMAKE_CXX_STANDARD 17)

find_package(catkin REQUIRED COMPONENTS roscpp l3_footstep_planning traversability_rotation_planning_plugins asterix_l3_plugins asterix_l3_config asterix_description)

catkin_package(
  CATKIN_DEPENDS roscpp l3_footstep_planning traversability_rotation_planning_plugins asterix_l3_plugins asterix_l3_config asterix_description
)

catkin_add_env_hooks(50.export.asterix_l3_footstep_planning SHELLS bash)

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
