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

#ifndef L3_FOOTSTEP_PLANNING_PLUGINS_TRAVERSABILITY_ROTATION_HEURISTIC_H
#define L3_FOOTSTEP_PLANNING_PLUGINS_TRAVERSABILITY_ROTATION_HEURISTIC_H

#include <l3_footstep_planning_plugins/base/hlut_heuristic_plugin.h>

#include <opencv2/core/eigen.hpp>

namespace l3_footstep_planning
{
class TraversabilityRotationHeuristic : public HLUTHeuristicPlugin
{
public:
  /**
   * @brief Default constructor
   */
  TraversabilityRotationHeuristic();

  bool loadParams( const vigir_generic_params::ParameterSet &params ) override;

  bool initialize( const vigir_generic_params::ParameterSet &params ) override;

  void preparePlanning( const msgs::StepPlanRequest &req ) override;

  double getHeuristicValue( const FloatingBase &from, const FloatingBase &to, const State &start,
                            const State &goal ) const override;

protected:
  std::vector<l3::PositionIndex> getNeighbors( const l3::PositionIndex &current_index ) const override;

  std::vector<l3::PositionIndex> getValidNeighbors( const std::vector<l3::PositionIndex> &neighbors ) const override;

  hlutEntry
  computeHLUTEntryOfNeighbor( const l3::PositionIndex &neighbor, const hlutEntry &current_entry ) const override;

  void visualizeHLUT() const override;

  /**
   * @brief Callback for the traversability map.
   * @param traversability_map_new The new traversability map
   */
  void mapCallback( const grid_map_msgs::GridMapConstPtr &traversability_map_new );

  // Specifies whether the rotation heuristic should be determined
  bool use_rotation_heuristic_;

  // The rotation heuristic lookup table
  HeuristicLookupTable rotation_hlut_;

  // The distance to the goal where the rotation heuristic gets disabled
  double disable_distance_;

  // The subscriber for the traversability map
  ros::Subscriber traversability_map_sub_;

  // The current traversability map pointer
  grid_map_msgs::GridMapConstPtr current_traversability_map_ptr_;

  // The mutex for the traversability map
  mutable Mutex traversability_map_shared_mutex_;

  // The traversability map topic
  std::string traversability_map_topic_;

  // The traversability map layer
  std::string traversability_map_layer_;

  // The traversability map
  grid_map::GridMap traversability_map_;

  // The traversability value of unknown regions
  float unknown_region_value_;

  // The exponential weight for the traversability heuristic
  float exponential_traversability_weight_;

private:
  static constexpr float inf = std::numeric_limits<float>::infinity();
};
}  // namespace l3_footstep_planning

#endif
