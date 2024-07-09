// =================================================================================================
// Copyright (c) 2024, Simon Giegerich, Technische Universität Darmstadt
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

#include <traversability_rotation_planning_plugins/std/step_cost_estimator/pitch_step_cost_estimator.h>

namespace l3_footstep_planning
{
PitchStepCostEstimator::PitchStepCostEstimator() : StepCostEstimatorPlugin( "pitch_step_cost_estimator" )
{
}

bool PitchStepCostEstimator::loadParams( const vigir_generic_params::ParameterSet &params )
{
  if ( !StepCostEstimatorPlugin::loadParams( params ))
    return false;

  getParam( "elevation_map_topic", elevation_map_topic_, std::string( "/elevation_estimation/elevation_map" ));
  getParam( "normal_x_layer", normal_x_layer_, std::string( "surface_normal_x" ));
  getParam( "normal_y_layer", normal_y_layer_, std::string( "surface_normal_y" ));
  getParam( "num_sampling_steps", num_sampling_steps_, 10 );
  getParam( "linear_weight", linear_weight_, 1.0 );

  if ( num_sampling_steps_ < 2 )
  {
    ROS_ERROR( "[%s]: num_sampling_steps must be at least 2, but is %d!", getName().c_str(), num_sampling_steps_ );
    return false;
  }

  inverted_sampling_step_size_ = 1.0 / num_sampling_steps_;

  return true;
}

bool PitchStepCostEstimator::initialize( const vigir_generic_params::ParameterSet &params )
{
  if ( !StepCostEstimatorPlugin::initialize( params ))
    return false;

  // Subscribe topics
  elevation_map_sub_ = nh_.subscribe( elevation_map_topic_, 1, &PitchStepCostEstimator::mapCallback,
                                      this );

  return true;
}

void PitchStepCostEstimator::preparePlanning( const l3_footstep_planning_msgs::StepPlanRequest &req )
{
  UniqueLock lock( elevation_map_shared_mutex_ );

  if ( !current_elevation_map_ptr_ )
  {
    ROS_ERROR( "No elevation map received yet!" );
    return;
  }

  grid_map::GridMapRosConverter::fromMessage( *current_elevation_map_ptr_, elevation_map_ );
}

bool PitchStepCostEstimator::getCost( const PlanningState &state, double &cost, double &cost_multiplier,
                                      double &risk, double &risk_multiplier ) const
{
  // Check if the required layers are available
  if ( !elevation_map_.exists( normal_x_layer_ ) || !elevation_map_.exists( normal_y_layer_ ))
  {
    ROS_ERROR_ONCE( "[%s]: Required layers %s and %s are not available in the elevation map!",
                    getName().c_str(), normal_x_layer_.c_str(), normal_y_layer_.c_str());
    return false;
  }

  FloatingBase::ConstPtr from_fb = state.getAdjacentState()->getFloatingBase( l3::BaseInfo::MAIN_BODY_IDX );
  l3::Pose2D from_pose( from_fb->x(), from_fb->y(), from_fb->yaw());
  cost = computeAngleDiff( from_pose );

  FloatingBase::ConstPtr to_fb = state.getState()->getFloatingBase( l3::BaseInfo::MAIN_BODY_IDX );
  l3::Pose2D to_pose( to_fb->x(), to_fb->y(), to_fb->yaw());
  cost = std::max( cost, computeAngleDiff( to_pose ));

  // Compute the 2D vector from the from_fb to the to_fb
  l3::Vector2 direction = l3::Vector2( to_fb->x() - from_fb->x(), to_fb->y() - from_fb->y());
  double direction_yaw = std::atan2( direction.y(), direction.x());

  double step_size = direction.norm() * inverted_sampling_step_size_;

  for ( int i = 1; i < num_sampling_steps_ - 1; i++ )
  {
    double step = i * step_size;
    l3::Pose2D step_pose( from_fb->x() + direction.x() * step, from_fb->y() + direction.y() * step, direction_yaw );
    cost = std::max( cost, computeAngleDiff( step_pose ));
  }

  cost *= linear_weight_;
  cost_multiplier = 1.0;
  risk = 0.0;
  risk_multiplier = 1.0;

  return true;
}

void PitchStepCostEstimator::mapCallback( const grid_map_msgs::GridMapConstPtr &elevation_map_new )
{
  current_elevation_map_ptr_ = elevation_map_new;
}

double PitchStepCostEstimator::computeAngleDiff( const l3::Pose2D &pose ) const
{
  grid_map::Index index;
  if ( !elevation_map_.getIndex( l3::Vector2( pose.x(), pose.y()), index ))
    return 0.0;

  l3::Vector2 normal( elevation_map_.at( normal_x_layer_, index ), elevation_map_.at( normal_y_layer_, index ));

  if ( std::isnan( normal.x()) || std::isnan( normal.y()))
    return 0.0;

  double normal_yaw = std::atan2( normal.y(), normal.x());

  double yaw_diff_1 = std::abs( l3::shortestAngularDistance( pose.yaw(), normal_yaw ));
  double yaw_diff_2 = std::abs( l3::shortestAngularDistance( pose.yaw(), l3::normalizeAngle( normal_yaw + M_PI )));

  return std::min( yaw_diff_1, yaw_diff_2 ) * normal.norm();
}
}  // namespace l3_footstep_planning

#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS( l3_footstep_planning::PitchStepCostEstimator, l3_footstep_planning::StepCostEstimatorPlugin )
