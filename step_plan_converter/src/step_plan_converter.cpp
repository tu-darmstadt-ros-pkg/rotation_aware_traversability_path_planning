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

#include <l3_plugins/robot_model.h>

namespace step_plan_converter
{
StepPlanConverter::StepPlanConverter( ros::NodeHandle &nh )
  : follow_path_ac_( "/controller/follow_path", false ), move_base_as_( nh, "/l3_planner/move_base", false )
    , still_planning_( false )
{
  move_base_as_.registerGoalCallback( boost::bind( &StepPlanConverter::moveBaseGoalCB, this ));
  move_base_as_.registerPreemptCallback( boost::bind( &StepPlanConverter::moveBaseCancelCB, this ));
  move_base_as_.start();

  follow_path_ac_.waitForServer( ros::Duration( 1.0 ));

  ros::NodeHandle nh_l3( "/l3/footstep_planning" );

  feet_pose_generator_ = l3::makeShared<l3_footstep_planning::FeetPoseGeneratorClient>( nh_l3 );

  step_plan_request_ac_ = l3::SimpleActionClient<l3_footstep_planning_msgs::StepPlanRequestAction>::create( nh_l3,
                                                                                                            "step_plan_request",
                                                                                                            false );

  traversability_map_sub_ = nh.subscribe( "/grid_map", 1, &StepPlanConverter::mapCallback, this );

  path_pub_ = nh.advertise<nav_msgs::Path>( "/step_plan_l3", 1 );
}

void StepPlanConverter::moveBaseGoalCB()
{
  // get the new goal
  geometry_msgs::PoseStamped current_goal = move_base_as_.acceptNewGoal()->target_pose;

  // create the header
  std_msgs::Header header;
  header.frame_id = current_goal.header.frame_id;
  header.stamp = ros::Time::now();

  // create the step plan request
  l3_footstep_planning_msgs::StepPlanRequestGoal request;
  request.plan_request.header = header;
  request.plan_request.planning_mode = l3_footstep_planning_msgs::StepPlanRequest::PLANNING_MODE_3D;
  request.plan_request.start_foot_idx = l3_footstep_planning_msgs::StepPlanRequest::AUTO_START_FOOT_IDX;

  l3_msgs::FloatingBase goal_floating_base = l3_msgs::FloatingBase();
  goal_floating_base.header = header;
  goal_floating_base.pose = current_goal.pose;

  // move the goal pose to the z position of the start pose
  l3::FootholdArray start_footholds;
  feet_pose_generator_->getStartFootholds( start_footholds, header.frame_id );
  l3::Pose start_pose = l3::Pose();
  if ( l3::RobotModel::kinematics())
    start_pose = l3::RobotModel::kinematics()->calcBasePose( l3::RobotModel::calcFeetCenter( start_footholds ),
                                                             start_footholds );
  goal_floating_base.pose.position.z = start_pose.z();

  request.plan_request.goal_floating_bases.push_back( goal_floating_base );

  current_request_ = request;

  step_plan_request_ac_->sendGoal( current_request_, boost::bind( &StepPlanConverter::stepPlanResultCb, this, _2 ));
}

void StepPlanConverter::moveBaseCancelCB()
{
  still_planning_ = false;

  if ( move_base_as_.isActive())
  {
    move_base_lite_msgs::MoveBaseResult result;
    result.result.val = move_base_lite_msgs::ErrorCodes::PREEMPTED;
    move_base_as_.setPreempted( result, "Action preempted at user request" );
    if ( step_plan_request_ac_->isServerConnected()) {
      step_plan_request_ac_->cancelAllGoals();
      step_plan_request_ac_->stopTrackingGoal();
    }
    if ( follow_path_ac_.isServerConnected())
      follow_path_ac_.cancelAllGoals();
  }
  else
    ROS_WARN( "[StepPlanConverter] Cancel called without an active goal" );
}

void StepPlanConverter::stepPlanResultCb( const l3_footstep_planning_msgs::StepPlanRequestResultConstPtr &result )
{
  still_planning_ = false;

  if ( l3_footstep_planning::hasError( result->status ))
  {
    ROS_ERROR( "[StepPlanConverter] Failed to generate step plan!" );
    return;
  }

  l3::StepQueue steps;
  steps.fromMsg( result->step_plan.plan );

  // convert the steps to poses for the follow path action
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
  path.header.frame_id = result->step_plan.header.frame_id;
  path.poses = poses;

  // publish the path
  path_pub_.publish( path );

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
  follow_path_ac_.sendGoal( goal.goal, boost::bind( &StepPlanConverter::followPathDoneCB, this, _1, _2 ));
}

void StepPlanConverter::followPathDoneCB( const actionlib::SimpleClientGoalState &state,
                                          const move_base_lite_msgs::FollowPathResultConstPtr &result_in )
{
  still_planning_ = false;

  if ( move_base_as_.isActive())
  {
    move_base_lite_msgs::MoveBaseResult result;
    result.result.val = result_in->result.val;
    if ( result.result.val == move_base_lite_msgs::ErrorCodes::SUCCESS )
      move_base_as_.setSucceeded( result, "Action succeeded" );
    else if ( result.result.val == move_base_lite_msgs::ErrorCodes::PREEMPTED )
      move_base_as_.setPreempted( result, "Action preempted" );
    else
      move_base_as_.setAborted( result, "Action failed with message: " + state.getText());
  }
}

void StepPlanConverter::mapCallback( const grid_map_msgs::GridMapConstPtr &/*map_new*/ )
{
  if ( still_planning_ )
    return;

  if ( move_base_as_.isActive())
  {
    still_planning_ = true;
    step_plan_request_ac_->sendGoal( current_request_, boost::bind( &StepPlanConverter::stepPlanResultCb, this, _2 ));
  }
}
}  // namespace step_plan_converter
