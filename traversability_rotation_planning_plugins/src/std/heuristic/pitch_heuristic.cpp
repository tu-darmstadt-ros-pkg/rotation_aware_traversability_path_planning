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

#include <traversability_rotation_planning_plugins/std/heuristic/pitch_heuristic.h>

#include <l3_libs/conversions/l3_msg_conversions.h>

namespace l3_footstep_planning
{
PitchHeuristic::PitchHeuristic() : HeuristicPlugin( "pitch_heuristic" )
{
}

bool PitchHeuristic::loadParams( const vigir_generic_params::ParameterSet &params )
{
  if ( !HeuristicPlugin::loadParams( params ))
    return false;

  getParam( "disable_distance", disable_distance_, 0.0, true );
  getParam( "elevation_map_topic", elevation_map_topic_, std::string( "/elevation_estimation/elevation_map" ));
  getParam( "normal_x_layer", normal_x_layer_, std::string( "surface_normal_x" ));
  getParam( "normal_y_layer", normal_y_layer_, std::string( "surface_normal_y" ));

  return true;
}

bool PitchHeuristic::initialize( const vigir_generic_params::ParameterSet &params )
{
  if ( !HeuristicPlugin::initialize( params ))
    return false;

  // Subscribe topics
  elevation_map_sub_ = nh_.subscribe( elevation_map_topic_, 1, &PitchHeuristic::mapCallback, this );

  return true;
}

void PitchHeuristic::preparePlanning( const l3_footstep_planning_msgs::StepPlanRequest &req )
{
  UniqueLock lock( elevation_map_shared_mutex_ );

  if ( !current_elevation_map_ptr_ )
  {
    ROS_ERROR( "No elevation map received yet!" );
    return;
  }

  grid_map::GridMapRosConverter::fromMessage( *current_elevation_map_ptr_, elevation_map_ );

  // get goal floating base and position
  l3::FloatingBaseArray goal_floating_bases;
  l3::floatingBaseArrayMsgToL3( req.goal_floating_bases, goal_floating_bases );
  goal_fb_ = goal_floating_bases.front();
  goal_pos_ = l3::Position2D( goal_fb_.x(), goal_fb_.y());
}

double PitchHeuristic::getHeuristicValue( const FloatingBase &from, const FloatingBase &to, const State &start,
                                          const State &goal ) const
{
  if ( l3::euclideanDistance( from.x(), from.y(), goal_fb_.x(), goal_fb_.y()) <= disable_distance_ )
    return 0.0;

  l3::Position2D from_pos( from.x(), from.y());
  grid_map::Index from_index;

  if ( !elevation_map_.getIndex( from_pos, from_index ))
    return 0.0;

  l3::Vector2 normal( elevation_map_.at( normal_x_layer_, from_index ),
                      elevation_map_.at( normal_y_layer_, from_index ));

  if ( std::isnan( normal.x()) || std::isnan( normal.y()))
    return 0.0;

  double normal_yaw = std::atan2( normal.y(), normal.x());

  double yaw_diff_1 = std::abs( l3::shortestAngularDistance( from.yaw(), l3::normalizeAngle( normal_yaw )));
  double yaw_diff_2 = std::abs( l3::shortestAngularDistance( from.yaw(), l3::normalizeAngle( normal_yaw + M_PI )));

  return std::min( yaw_diff_1, yaw_diff_2 ) * normal.norm();
}

void PitchHeuristic::mapCallback( const grid_map_msgs::GridMapConstPtr &elevation_map_new )
{
  current_elevation_map_ptr_ = elevation_map_new;
}
}  // namespace l3_footstep_planning

#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS( l3_footstep_planning::PitchHeuristic, l3_footstep_planning::HeuristicPlugin )
