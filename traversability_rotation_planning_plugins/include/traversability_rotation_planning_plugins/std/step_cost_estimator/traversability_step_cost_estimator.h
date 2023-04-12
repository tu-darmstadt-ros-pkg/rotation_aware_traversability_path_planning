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

#ifndef L3_FOOTSTEP_PLANNING_PLUGINS_TRAVERSABILITY_STEP_COST_ESTIMATOR_H__
#define L3_FOOTSTEP_PLANNING_PLUGINS_TRAVERSABILITY_STEP_COST_ESTIMATOR_H__

#include <grid_map_ros/grid_map_ros.hpp>

#include <hector_stability_metrics/math/support_polygon.h>

#include <hector_math/iterators/polygon_iterator.h>

#include <l3_footstep_planning_plugins/base/step_cost_estimator_plugin.h>

#include <l3_plugins/robot_model.h>

namespace l3_footstep_planning
{
class TraversabilityStepCostEstimator : public StepCostEstimatorPlugin
{
public:
  TraversabilityStepCostEstimator();

  bool loadParams( const vigir_generic_params::ParameterSet &params ) override;

  bool initialize( const vigir_generic_params::ParameterSet &params ) override;

  void preparePlanning( const msgs::StepPlanRequest &req ) override;

  bool getCost( const PlanningState &state, double &cost, double &cost_multiplier, double &risk,
                double &risk_multiplier ) const override;

protected:
  /**
   * @brief Callback for the traversability map.
   * @param traversability_map_new The new traversability map
   */
  void mapCallback( const grid_map_msgs::GridMapConstPtr &traversability_map_new );

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

  // Half of the robot length plus the safety margin
  double upper_body_x_diff_;

  // Half of the robot width plus the safety margin
  double upper_body_y_diff_;

  // The safety margin in the length direction
  double safety_margin_x_;

  // The safety margin in the width direction
  double safety_margin_y_;

  // Specifies how much the minimum traversability value should be considered for the cost
  float min_traversability_weight_;

  // Specifies how much the mean traversability value should be considered for the cost
  float mean_traversability_weight_;
};
}  // namespace l3_footstep_planning

#endif
