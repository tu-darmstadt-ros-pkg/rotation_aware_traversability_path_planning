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

#include <traversability_rotation_planning_plugins/std/step_cost_estimator/traversability_step_cost_estimator.h>

namespace l3_footstep_planning
{
TraversabilityStepCostEstimator::TraversabilityStepCostEstimator()
  : StepCostEstimatorPlugin( "traversability_step_cost_estimator" )
{
}

bool TraversabilityStepCostEstimator::loadParams( const vigir_generic_params::ParameterSet &params )
{
  if ( !StepCostEstimatorPlugin::loadParams( params ))
    return false;

  getParam( "traversability_map_topic", traversability_map_topic_,
            std::string( "/traversability_estimation/traversability_map" ));
  getParam( "traversability_map_layer", traversability_map_layer_, std::string( "traversability" ));
  getParam( "unknown_region_value", unknown_region_value_, 0.9f );
  getParam( "safety_margin_x", safety_margin_x_, 0.0, true );
  getParam( "safety_margin_y", safety_margin_y_, 0.0, true );
  getParam( "min_traversability_weight", min_traversability_weight_, 0.25f );
  getParam( "mean_traversability_weight", mean_traversability_weight_, 0.75f );

  if ( unknown_region_value_ < 0.0f || unknown_region_value_ > 1.0f )
  {
    ROS_ERROR( "[%s]: unknown_region_value must be in [0.0, 1.0]!", getName().c_str());
    return false;
  }

  if ( safety_margin_x_ < 0.0 || safety_margin_y_ < 0.0 )
  {
    ROS_ERROR( "[%s]: The safety margins must be >= 0.0!", getName().c_str());
    return false;
  }

  if ( min_traversability_weight_ + mean_traversability_weight_ != 1.0f )
  {
    ROS_ERROR( "[%s]: min_traversability_weight and mean_traversability_weight must sum up to 1.0!", getName().c_str());
    return false;
  }

  BaseInfo base_info;
  RobotModel::description()->getBaseInfo( BaseInfo::MAIN_BODY_IDX, base_info );
  Vector3 upper_body_size = base_info.size;
  upper_body_x_diff_ = upper_body_size.x() * 0.5 + safety_margin_x_;
  upper_body_y_diff_ = upper_body_size.y() * 0.5 + safety_margin_y_;

  return true;
}

bool TraversabilityStepCostEstimator::initialize( const vigir_generic_params::ParameterSet &params )
{
  if ( !StepCostEstimatorPlugin::initialize( params ))
    return false;

  // Subscribe topics
  traversability_map_sub_ = nh_.subscribe( traversability_map_topic_, 1, &TraversabilityStepCostEstimator::mapCallback,
                                           this );

  return true;
}

void TraversabilityStepCostEstimator::preparePlanning( const l3_footstep_planning_msgs::StepPlanRequest &req )
{
  UniqueLock lock( traversability_map_shared_mutex_ );
  grid_map::GridMapRosConverter::fromMessage( *current_traversability_map_ptr_, traversability_map_ );
}

bool TraversabilityStepCostEstimator::getCost( const PlanningState &state, double &cost, double &cost_multiplier,
                                               double &risk, double &risk_multiplier ) const
{
  FloatingBase::ConstPtr from_fb = state.getAdjacentState()->getFloatingBase( l3::BaseInfo::MAIN_BODY_IDX );
  double from_yaw = from_fb->yaw();
  double from_cos_yaw = std::cos( from_yaw );
  double from_sin_yaw = std::sin( from_yaw );

  double from_x_times_cos = upper_body_x_diff_ * from_cos_yaw;
  double from_x_times_sin = upper_body_x_diff_ * from_sin_yaw;
  double from_y_times_cos = upper_body_y_diff_ * from_cos_yaw;
  double from_y_times_sin = upper_body_y_diff_ * from_sin_yaw;

  FloatingBase::ConstPtr to_fb = state.getState()->getFloatingBase( l3::BaseInfo::MAIN_BODY_IDX );
  double to_yaw = to_fb->yaw();
  double to_cos_yaw = std::cos( to_yaw );
  double to_sin_yaw = std::sin( to_yaw );

  double to_x_times_cos = upper_body_x_diff_ * to_cos_yaw;
  double to_x_times_sin = upper_body_x_diff_ * to_sin_yaw;
  double to_y_times_cos = upper_body_y_diff_ * to_cos_yaw;
  double to_y_times_sin = upper_body_y_diff_ * to_sin_yaw;

  // Compute the corners of the two footprints
  hector_stability_metrics::math::Vector3List<double> polygon_corners = {
    hector_stability_metrics::math::Vector3<double>((from_x_times_cos - from_y_times_sin) + from_fb->x(),
                                                    (from_x_times_sin + from_y_times_cos) + from_fb->y(), 0.0f ),
    hector_stability_metrics::math::Vector3<double>((from_x_times_cos + from_y_times_sin) + from_fb->x(),
                                                    (from_x_times_sin - from_y_times_cos) + from_fb->y(), 0.0f ),
    hector_stability_metrics::math::Vector3<double>((-from_x_times_cos - from_y_times_sin) + from_fb->x(),
                                                    (from_y_times_cos - from_x_times_sin) + from_fb->y(), 0.0f ),
    hector_stability_metrics::math::Vector3<double>((from_y_times_sin - from_x_times_cos) + from_fb->x(),
                                                    (-from_x_times_sin - from_y_times_cos) + from_fb->y(), 0.0f ),
    hector_stability_metrics::math::Vector3<double>((to_x_times_cos - to_y_times_sin) + to_fb->x(),
                                                    (to_x_times_sin + to_y_times_cos) + to_fb->y(), 0.0f ),
    hector_stability_metrics::math::Vector3<double>((to_x_times_cos + to_y_times_sin) + to_fb->x(),
                                                    (to_x_times_sin - to_y_times_cos) + to_fb->y(), 0.0f ),
    hector_stability_metrics::math::Vector3<double>((-to_x_times_cos - to_y_times_sin) + to_fb->x(),
                                                    (to_y_times_cos - to_x_times_sin) + to_fb->y(), 0.0f ),
    hector_stability_metrics::math::Vector3<double>((to_y_times_sin - to_x_times_cos) + to_fb->x(),
                                                    (-to_x_times_sin - to_y_times_cos) + to_fb->y(), 0.0f )
  };

  // Compute the convex hull of the polygon corners
  hector_stability_metrics::math::Vector3List<double> convex_hull = hector_stability_metrics::math::supportPolygonFromUnsortedContactPoints(
    polygon_corners );

  // Generate a polygon from the convex hull
  hector_math::Polygon<int> polygon( 2, convex_hull.size());
  for ( int i = 0; i < convex_hull.size(); i++ )
  {
    grid_map::Index index;
    traversability_map_.getIndex( l3::Position2D( convex_hull[i].x(), convex_hull[i].y()), index );
    polygon.col( i ) << index.x(), index.y();
  }

  // Initialize variables for polygon iteration
  const grid_map::Matrix &layer = traversability_map_[traversability_map_layer_];
  float min_traversability = 1.0f;
  float sum_traversability = 0.0f;
  float num_cells = 0.0f;
  float unknown_region_value = unknown_region_value_;

  // Iterate over the polygon
  hector_math::iteratePolygon( polygon,
                               [ &layer, &min_traversability, &sum_traversability, &num_cells, &unknown_region_value ](
                                 Eigen::Index x, Eigen::Index y )
                               {
                                 num_cells++;

                                 float traversability = layer( x, y );
                                 if ( std::isnan( traversability ))
                                   traversability = unknown_region_value;

                                 sum_traversability += traversability;
                                 min_traversability = std::min( min_traversability, traversability );
                               } );

  cost = 1.0f / (min_traversability_weight_ * min_traversability +
                 mean_traversability_weight_ * sum_traversability / num_cells);
  cost_multiplier = 1.0;
  risk = 0.0;
  risk_multiplier = 1.0;

  return true;
}

void TraversabilityStepCostEstimator::mapCallback( const grid_map_msgs::GridMapConstPtr &traversability_map_new )
{
  current_traversability_map_ptr_ = traversability_map_new;
}
}  // namespace l3_footstep_planning

#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS( l3_footstep_planning::TraversabilityStepCostEstimator,
                        l3_footstep_planning::StepCostEstimatorPlugin )
