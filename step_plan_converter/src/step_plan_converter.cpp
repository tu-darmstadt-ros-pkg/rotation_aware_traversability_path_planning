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

#include <step_plan_converter/step_plan_converter.h>

namespace step_plan_converter
{
StepPlanConverter::StepPlanConverter( ros::NodeHandle &nh )
  : followPathActionClient_( "/controller/follow_path", false )
{
  ros::NodeHandle pnh( "~" );

  followPathActionClient_.waitForServer( ros::Duration( 1 ));

  step_plan_sub_ = nh.subscribe( "/l3/footstep_planning/step_plan", 1, &StepPlanConverter::stepPlanCallback, this );
}

void StepPlanConverter::stepPlanCallback( const l3_footstep_planning_msgs::StepPlan &step_plan )
{
  l3::StepQueue steps;

  steps.fromMsg( step_plan.plan );

  std::vector<geometry_msgs::PoseStamped> poses;

  for ( const l3::StepQueue::Entry &e: steps )
  {
    l3::Step::ConstPtr step = e.second;
    l3::FloatingBase::ConstPtr fb = step->getFloatingBase( l3::BaseInfo::MAIN_BODY_IDX );
    l3::Pose pose = fb->pose();

    geometry_msgs::PoseStamped poseStamped;
    poseStamped.pose.position.x = pose.x();
    poseStamped.pose.position.y = pose.y();
    poseStamped.pose.position.z = pose.z();
    poseStamped.pose.orientation.x = pose.getQuaternion().x();
    poseStamped.pose.orientation.y = pose.getQuaternion().y();
    poseStamped.pose.orientation.z = pose.getQuaternion().z();
    poseStamped.pose.orientation.w = pose.getQuaternion().w();

    poses.push_back( poseStamped );
  }

  nav_msgs::Path path;
  path.header.frame_id = step_plan.header.frame_id;
  path.poses = poses;

  // create goal for action server
  move_base_lite_msgs::FollowPathActionGoal goal;

  // set goal
  goal.goal.target_path = path;

  // set options
  goal.goal.follow_path_options.use_path_orientation = true;
  goal.goal.follow_path_options.rotate_front_to_goal_pose_orientation = true;
  goal.goal.follow_path_options.reverse_allowed = true;
  goal.goal.follow_path_options.reverse_forced = false;

  goal.goal.follow_path_options.goal_pose_position_tolerance = 0.1;
  goal.goal.follow_path_options.goal_pose_angle_tolerance = 0.15;

  goal.goal.follow_path_options.desired_speed = 0.0;

  goal.goal.follow_path_options.reset_stuck_history = true;
  goal.goal.follow_path_options.is_fixed = true;

  // send goal
  followPathActionClient_.sendGoal( goal.goal );
}
}  // namespace step_plan_converter
