// =================================================================================================
// Copyright (c) 2023, Simon Giegerich, Technische Universität Darmstadt
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice, this
//    list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
//
// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from
//    this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
// =================================================================================================

#ifndef STEP_PLAN_CONVERTER_STEP_PLAN_CONVERTER_H
#define STEP_PLAN_CONVERTER_STEP_PLAN_CONVERTER_H

#include <ros/ros.h>

#include <l3_libs/robot_description/base_info.h>
#include <l3_libs/types/step_queue.h>

#include <l3_footstep_planning_msgs/footstep_planning_msgs.h>
#include <l3_footstep_planning_msgs/StepPlan.h>
#include <l3_footstep_planning_msgs/StepPlanRequestAction.h>

#include <l3_footstep_planning_tools/feet_pose_generator_client.h>

#include <geometry_msgs/PoseStamped.h>

#include <grid_map_ros/grid_map_ros.hpp>

#include <move_base_lite_msgs/FollowPathAction.h>
#include <move_base_lite_msgs/MoveBaseAction.h>

namespace step_plan_converter
{

class StepPlanConverter
{
public:
  StepPlanConverter( ros::NodeHandle &nh );

protected:
  /**
   * @brief Accepts new goal and triggers planning.
   */
  void moveBaseGoalCB();

  /**
   * @brief Cancels the current planning and execution.
   */
  void moveBaseCancelCB();

  /**
   * @brief Converts the step plan to a follow path goal and sends it.
   * @param result The result of the step plan request.
   */
  void stepPlanResultCb( const l3_footstep_planning_msgs::StepPlanRequestResultConstPtr &result );

  /**
   * @brief Sets the status of the move base action server.
   */
  void followPathDoneCB( const actionlib::SimpleClientGoalState &state,
                         const move_base_lite_msgs::FollowPathResultConstPtr &result_in );

  /**
   * @brief Callback for the map. This is only used for replanning if a new map is received.
   * @param map_new The new map (not used).
   */
  void mapCallback( const grid_map_msgs::GridMapConstPtr &/*map_new*/ );

  // The subscriber for the traversability map
  ros::Subscriber traversability_map_sub_;

  // The publisher for the coverted path
  ros::Publisher path_pub_;

  // The time the planning was started
  ros::Time planning_start_time_;

  // The action client for the follow path action
  actionlib::SimpleActionClient<move_base_lite_msgs::FollowPathAction> follow_path_ac_;

  // The action client for the step plan request action
  l3::SimpleActionClient<l3_footstep_planning_msgs::StepPlanRequestAction>::Ptr step_plan_request_ac_;

  // The action server for the move base action
  actionlib::SimpleActionServer<move_base_lite_msgs::MoveBaseAction> move_base_as_;

  // The feet pose generator
  l3_footstep_planning::FeetPoseGeneratorClient::Ptr feet_pose_generator_;

  // The current step plan request
  l3_footstep_planning_msgs::StepPlanRequestGoal current_request_;

  bool still_planning_;
};
}  // namespace step_plan_converter

#endif
