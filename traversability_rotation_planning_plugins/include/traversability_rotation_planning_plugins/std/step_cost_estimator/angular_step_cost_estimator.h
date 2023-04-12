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

#ifndef L3_FOOTSTEP_PLANNING_PLUGINS_ANGULAR_STEP_COST_ESTIMATOR_H__
#define L3_FOOTSTEP_PLANNING_PLUGINS_ANGULAR_STEP_COST_ESTIMATOR_H__

#include <l3_footstep_planning_plugins/base/step_cost_estimator_plugin.h>

namespace l3_footstep_planning
{
class AngularStepCostEstimator : public StepCostEstimatorPlugin
{
public:
  AngularStepCostEstimator();

  bool loadParams( const vigir_generic_params::ParameterSet &params ) override;

  void preparePlanning( const msgs::StepPlanRequest &req ) override;

  bool getCost( const PlanningState &state, double &cost, double &cost_multiplier, double &risk,
                double &risk_multiplier ) const override;

protected:
  // The distance to the goal where the rotation heuristic gets disabled
  double disable_distance_;

  // Linear weight of the cost
  double cost_weight_;

  // Position of the goal
  l3::Position2D goal_position_;
};
}  // namespace l3_footstep_planning

#endif
