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

#include <traversability_rotation_planning_plugins/std/step_cost_estimator/angular_step_cost_estimator.h>

#include <l3_libs/conversions/l3_msg_conversions.h>
#include <l3_libs/robot_description/base_info.h>

namespace l3_footstep_planning
{
AngularStepCostEstimator::AngularStepCostEstimator() : StepCostEstimatorPlugin( "angular_step_cost_estimator" )
{
}

bool AngularStepCostEstimator::loadParams( const vigir_generic_params::ParameterSet &params )
{
  if ( !StepCostEstimatorPlugin::loadParams( params ))
    return false;

  getParam( "disable_distance", disable_distance_, 0.0, true );
  getParam( "cost_weight", cost_weight_, 1.0, true );

  return true;
}

void AngularStepCostEstimator::preparePlanning( const msgs::StepPlanRequest &req )
{
  // Get goal position
  l3::FloatingBaseArray goal_floating_bases;
  floatingBaseArrayMsgToL3( req.goal_floating_bases, goal_floating_bases );
  FloatingBase goal_fb = goal_floating_bases.front();
  goal_position_ = l3::Position2D( goal_fb.x(), goal_fb.y());
}

bool AngularStepCostEstimator::getCost( const PlanningState &state, double &cost, double &cost_multiplier, double &risk,
                                        double &risk_multiplier ) const
{
  // Get from floating base
  FloatingBase::ConstPtr from_fb = state.getAdjacentState()->getFloatingBase( l3::BaseInfo::MAIN_BODY_IDX );

  // Get to floating base
  FloatingBase::ConstPtr to_fb = state.getState()->getFloatingBase( l3::BaseInfo::MAIN_BODY_IDX );

  // Translation
  double x_trans = to_fb->x() - from_fb->x();
  double y_trans = to_fb->y() - from_fb->y();

  // Set cost to 0 if we are close to the goal
  if ( euclideanDistance( from_fb->x(), from_fb->y(), goal_position_.x(), goal_position_.y()) > disable_distance_ &&
       euclideanDistance( to_fb->x(), to_fb->y(), goal_position_.x(), goal_position_.y()) > disable_distance_ )
    cost = abs( shortestAngularDistance( from_fb->yaw(), std::atan2( y_trans, x_trans ))) * cost_weight_;
  else
    cost = 0.0;
  cost_multiplier = 1.0;
  risk = 0.0;
  risk_multiplier = 1.0;

  return true;
}
}  // namespace l3_footstep_planning

#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS( l3_footstep_planning::AngularStepCostEstimator, l3_footstep_planning::StepCostEstimatorPlugin )
