# Rotation-aware Traversability Path Planning

This repository is based on the [Legged Locomotion Library (L3)](https://github.com/tu-darmstadt-ros-pkg/legged_locomotion_library). 
It extends L3 in the following ways:

* Plugins that consider traversability during path planning
* Configs allowing the use of [Boston Dynamics' Spot](https://www.bostondynamics.com/products/spot) in L3
* Conversion of resulting L3 step plans into [Follow Path Actions](https://github.com/tu-darmstadt-ros-pkg/move_base_lite/blob/master/move_base_lite_msgs/action/FollowPath.action)


## Requirements

This repository is tested under Ubuntu 20.04 and ROS Noetic.


## Installation

Clone this repository into your workspace. It includes the `dependencies.rosinstall` which contains all necessary dependencies, which can be merged using the `wstool merge` command. Finally, build your workspace and everything should be ready.


## Usage

You can start the path planner with the following command:

    roslaunch spot_l3_footstep_planning main.launch
    
It is also possible to start RViz if you change the command to:

    roslaunch spot_l3_footstep_planning main.launch rviz:=true
    
For the planner to work as intended, make sure that a traversability map of the [Grid Map Library](https://github.com/ANYbotics/grid_map) is being published, which is needed for the traversability calculations of the planner.

There are two ways to start the planning process:

* If RViz is running it is possible to use shortcut 'g' to drag a goal pose on the grid.
* Use the command

      roslaunch spot_l3_footstep_planning goal_publisher.launch
      
  which has the following optional arguments to specify position and orientation of the goal pose: `frame_id`, `pos_x`, `pos_y`, `pos_z`, `ori_x`, `ori_y`, `ori_z`, `ori_w`.
