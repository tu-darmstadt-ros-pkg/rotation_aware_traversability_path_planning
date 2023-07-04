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
  : followPathActionClient_( "/controller/follow_path", false ), moveBaseActionServer_( nh, "/l3_planner/move_base",
                                                                                        false )
{
  ros::NodeHandle pnh( "~" );

  goal_pub_ = nh.advertise<geometry_msgs::PoseStamped>( "/goal", 1, true );

  moveBaseActionServer_.registerGoalCallback( boost::bind( &StepPlanConverter::moveBaseGoalCB, this ));
  moveBaseActionServer_.registerPreemptCallback( boost::bind( &StepPlanConverter::moveBaseCancelCB, this ));
  moveBaseActionServer_.start();

  followPathActionClient_.waitForServer( ros::Duration( 1 ));

  step_plan_request_ac_ = l3::SimpleActionClient<l3_footstep_planning_msgs::StepPlanRequestAction>::create(nh, "step_plan_request", false);

  step_plan_sub_ = nh.subscribe( "/l3/footstep_planning/step_plan", 1, &StepPlanConverter::stepPlanCallback, this );

  traversability_map_sub_ = nh.subscribe( "/grid_map", 1, &StepPlanConverter::mapCallback, this );
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
    Eigen::Quaterniond quaternion = pose.getQuaternion();
    poseStamped.pose.orientation.x = quaternion.x();
    poseStamped.pose.orientation.y = quaternion.y();
    poseStamped.pose.orientation.z = quaternion.z();
    poseStamped.pose.orientation.w = quaternion.w();

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
  goal.goal.follow_path_options.is_fixed = false;

  // send goal
  followPathActionClient_.sendGoal( goal.goal, boost::bind( &StepPlanConverter::followPathDoneCB, this, _1, _2 ));
}

void StepPlanConverter::moveBaseGoalCB()
{
  move_base_action_goal_ = moveBaseActionServer_.acceptNewGoal();
  current_goal_ = move_base_action_goal_->target_pose;

  l3_footstep_planning_msgs::StepPlanRequestGoal request;
  request.plan_request.header.frame_id = current_goal_.header.frame_id;
  request.plan_request.header.stamp = ros::Time::now();

  l3_msgs::FloatingBase goal_floating_base = l3_msgs::FloatingBase();
  goal_floating_base.pose = current_goal_.pose;

  request.plan_request.goal_floating_bases.push_back(goal_floating_base);

  //step_plan_request_ac_->sendGoal(request, boost::bind(&StepPlanConverter::followPathDoneCB, this, _1, _2));
}

void StepPlanConverter::moveBaseCancelCB()
{
  if ( moveBaseActionServer_.isActive())
  {
    move_base_lite_msgs::MoveBaseResult result;
    result.result.val = move_base_lite_msgs::ErrorCodes::PREEMPTED;
    moveBaseActionServer_.setPreempted( result, "Action preempted at user request" );
    if ( followPathActionClient_.isServerConnected())
      followPathActionClient_.cancelAllGoals();
  }
  else
  {
    ROS_WARN( "[MoveBaseLiteServer] Cancel called without an active goal" );
  }
}

void StepPlanConverter::followPathDoneCB( const actionlib::SimpleClientGoalState &state,
                                          const move_base_lite_msgs::FollowPathResultConstPtr &result_in )
{
  if ( moveBaseActionServer_.isActive())
  {
    if ( result_in->result.val == move_base_lite_msgs::ErrorCodes::SUCCESS )
    {
      move_base_lite_msgs::MoveBaseResult result;
      result.result.val = move_base_lite_msgs::ErrorCodes::SUCCESS;
      moveBaseActionServer_.setSucceeded( result, "Action succeeded" );
    }
    else if ( result_in->result.val == move_base_lite_msgs::ErrorCodes::PREEMPTED )
    {
      move_base_lite_msgs::MoveBaseResult result;
      result.result.val = move_base_lite_msgs::ErrorCodes::PREEMPTED;
      moveBaseActionServer_.setPreempted( result, "Action preempted" );
    }
    else
    {
      move_base_lite_msgs::MoveBaseResult result;
      result.result.val = result_in->result.val;
      moveBaseActionServer_.setAborted( result, "Action failed with message: " + state.getText());
    }
  }
}

void StepPlanConverter::mapCallback( const grid_map_msgs::GridMapConstPtr &map_new )
{
  if ( moveBaseActionServer_.isActive())
    goal_pub_.publish( current_goal_ );
}
}  // namespace step_plan_converter
