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

#ifndef ASTERIX_TURNING_STATE_GENERATOR_PLUGIN_H__
#define ASTERIX_TURNING_STATE_GENERATOR_PLUGIN_H__

#include <l3_footstep_planning_libs/modeling/foot_step_action.h>

#include <l3_footstep_planning_plugins/base/use_mask_generator_plugin.h>
#include <l3_footstep_planning_plugins/std/state_generator/polygonal_state_generator.h>

namespace asterix_footstep_planning
{
using namespace l3_footstep_planning;

class AsterixTurningStateGenerator : public UseMaskGeneratorPlugin, public PolygonalStateGenerator
{
  typedef std::pair<const FootIndex, std::vector<FootStepAction>> FootStepActionSetPair;
  typedef std::map<FootIndex, std::vector<FootStepAction>> FootStepActionSetMap;

public:
  AsterixTurningStateGenerator();

  bool loadParams( const vigir_generic_params::ParameterSet &params ) override;

  UseMask determineStateGenerationUseMask( const PlanningState &state, const State &start,
                                           const State &goal ) const override
  {
    std::list<UseMaskSuperimposition> masks;
    return determineUseMask( *state.getState(), start, goal, false, masks );
  }

  UseMask determineStepCostEstimatorUseMask( const PlanningState &state, const State &start, const State &goal,
                                             std::list<UseMaskSuperimposition> &masks ) const override
  {
    return determineUseMask( *state.getState(), start, goal, use_superimposition_, masks );
  }

  UseMask determineHeuristicUseMask( const State &from, const State & /*to*/, const State &start, const State &goal,
                                     std::list<UseMaskSuperimposition> &masks ) const override
  {
    return determineUseMask( from, start, goal, use_superimposition_, masks );
  }

  virtual std::list<StateGenResult> generatePredStateResults( const PlanningState &state, const State &start,
                                                              const State &goal,
                                                              const ExpandStatesIdx &state_expansion_idx ) const override;

  virtual std::list<StateGenResult> generateSuccStateResults( const PlanningState &state, const State &start,
                                                              const State &goal,
                                                              const ExpandStatesIdx &state_expansion_idx ) const override;

protected:
  UseMask determineUseMask( const State &current, const State &start, const State &goal, bool superimposition,
                            std::list<UseMaskSuperimposition> &masks ) const;

  bool checkIfPositionInTarget( const State &current, const State &target ) const;

  UseMask goal_yaw_use_mask_;
  UseMask travel_yaw_use_mask_;

  bool use_superimposition_;
  double min_weight_;

  // The set of footsteps used
  FootStepActionSetMap footstep_actions_left_turn_;
  FootStepActionSetMap footstep_actions_right_turn_;

  // parameters for trigger turning when in goal
  double max_goal_dist_sq_;
  double min_goal_dyaw_;

  // parameters for trigger turning when facing the opposite direction
  double min_travel_dist_sq_;
  double min_travel_dyaw_;
};
}  // namespace asterix_footstep_planning

#endif
