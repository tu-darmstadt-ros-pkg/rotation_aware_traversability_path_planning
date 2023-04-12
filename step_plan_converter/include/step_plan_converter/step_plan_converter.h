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

#ifndef STEP_PLAN_CONVERTER_STEP_PLAN_CONVERTER_H
#define STEP_PLAN_CONVERTER_STEP_PLAN_CONVERTER_H

#include <ros/ros.h>

#include <l3_libs/robot_description/base_info.h>
#include <l3_libs/types/step_queue.h>

#include <l3_footstep_planning_msgs/StepPlan.h>

#include <geometry_msgs/PoseStamped.h>

#include <move_base_lite_msgs/FollowPathAction.h>

namespace step_plan_converter
{

class StepPlanConverter
{
public:
  StepPlanConverter( ros::NodeHandle &nh );

protected:
  /**
   * @brief Converts a l3 step plan to a follow path action goal and sends it.
   * @param step_plan The l3 step plan to convert
   */
  void stepPlanCallback( const l3_footstep_planning_msgs::StepPlan &step_plan );

  // The subscriber for l3 step plan
  ros::Subscriber step_plan_sub_;

  // The action client for the follow path action
  actionlib::SimpleActionClient<move_base_lite_msgs::FollowPathAction> followPathActionClient_;
};
}  // namespace step_plan_converter

#endif
