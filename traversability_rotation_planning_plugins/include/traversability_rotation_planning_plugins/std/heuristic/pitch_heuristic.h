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

#ifndef L3_FOOTSTEP_PLANNING_PLUGINS_PITCH_HEURISTIC_H
#define L3_FOOTSTEP_PLANNING_PLUGINS_PITCH_HEURISTIC_H

#include <l3_footstep_planning_plugins/base/heuristic_plugin.h>

#include <grid_map_ros/grid_map_ros.hpp>

namespace l3_footstep_planning
{
class PitchHeuristic : public HeuristicPlugin
{
public:
  /**
   * @brief Default constructor
   */
  PitchHeuristic();

  bool loadParams( const vigir_generic_params::ParameterSet &params ) override;

  bool initialize( const vigir_generic_params::ParameterSet &params ) override;

  void preparePlanning( const msgs::StepPlanRequest &req ) override;

  double getHeuristicValue( const FloatingBase &from, const FloatingBase &to, const State &start,
                            const State &goal ) const override;

protected:
  /**
   * @brief Callback for the elevation map.
   * @param elevation_map_new The new elevation map
   */
  void mapCallback( const grid_map_msgs::GridMapConstPtr &elevation_map_new );

  // The distance to the goal where the rotation heuristic gets disabled
  double disable_distance_;

  // The floating base of the goal state.
  FloatingBase goal_fb_;

  // The position of the goal state.
  l3::Position2D goal_pos_;

  // The subscriber for the elevation map
  ros::Subscriber elevation_map_sub_;

  // The current elevation map pointer
  grid_map_msgs::GridMapConstPtr current_elevation_map_ptr_;

  // The mutex for the elevation map
  mutable Mutex elevation_map_shared_mutex_;

  // The elevation map topic
  std::string elevation_map_topic_;

  // The elevation map normal x layer
  std::string normal_x_layer_;

  // The elevation map normal y layer
  std::string normal_y_layer_;

  // The elevation map
  grid_map::GridMap elevation_map_;
};
}  // namespace l3_footstep_planning

#endif
