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

#include <traversability_rotation_planning_plugins/std/heuristic/traversability_rotation_heuristic.h>

namespace l3_footstep_planning
{
TraversabilityRotationHeuristic::TraversabilityRotationHeuristic() : HLUTHeuristicPlugin(
  "traversability_rotation_heuristic" )
{
}

bool TraversabilityRotationHeuristic::loadParams( const vigir_generic_params::ParameterSet &params )
{
  if ( !HLUTHeuristicPlugin::loadParams( params ))
    return false;

  getParam( "use_rotation_heuristic", use_rotation_heuristic_, false, true );
  getParam( "disable_distance", disable_distance_, 0.0, true );
  getParam( "traversability_map_topic", traversability_map_topic_,
            std::string( "/traversability_estimation/traversability_map" ));
  getParam( "traversability_map_layer", traversability_map_layer_, std::string( "traversability" ));
  getParam( "unknown_region_value", unknown_region_value_, 0.9f, true );
  getParam( "exponential_traversability_weight", exponential_traversability_weight_, 1.0f, true );

  return true;
}

void TraversabilityRotationHeuristic::preparePlanning( const l3_footstep_planning_msgs::StepPlanRequest &req )
{
  UniqueLock lock( traversability_map_shared_mutex_ );
  grid_map::GridMapRosConverter::fromMessage( *current_traversability_map_ptr_, traversability_map_ );

  HLUTHeuristicPlugin::preparePlanning( req );

  if ( use_rotation_heuristic_ )
  {
    rotation_hlut_ = initializeHLUT( start_fb_, goal_fb_, goal_pos_ );

    cv::Mat traversability_map_image;
    cv::eigen2cv( hlut_.getHeuristicMatrix(), traversability_map_image );

    cv::Mat derivative_x;
    cv::Sobel( traversability_map_image, derivative_x, CV_32F, 1, 0, 3 );
    cv::Mat derivative_y;
    cv::Sobel( traversability_map_image, derivative_y, CV_32F, 0, 1, 3 );

    auto *x = derivative_x.ptr<float>( 0 );
    auto *y = derivative_y.ptr<float>( 0 );

    cv::Mat gradient = cv::Mat::zeros( derivative_x.rows, derivative_x.cols, CV_32F );
    auto *gradient_pixel = gradient.ptr<float>( 0 );
    for ( int i = 0; i < derivative_x.rows * derivative_x.cols; i++ )
      gradient_pixel[i] = std::atan2( x[i], y[i] );

    Eigen::MatrixXf gradient_matrix;
    cv::cv2eigen( gradient, gradient_matrix );
    rotation_hlut_.setHeuristicMatrix( gradient_matrix );

    if ( visualize_ )
      visualizeHLUT();
  }
}

double
TraversabilityRotationHeuristic::getHeuristicValue( const FloatingBase &from, const FloatingBase &to,
                                                    const State &start,
                                                    const State &goal ) const
{
  if ( l3::euclideanDistance( from.x(), from.y(), goal_pos_.x(), goal_pos_.y()) <= disable_distance_ )
    return 0.0;

  double heuristic_without_rotation = HLUTHeuristicPlugin::getHeuristicValue( from, to, start, goal );

  l3::Position2D from_pos( from.x(), from.y());

  if ( use_rotation_heuristic_ )
  {
    double yaw_des = rotation_hlut_.getHeuristicEntry( l3::Position2D( from.x(), from.y()));
    double yaw_1 = std::abs( shortestAngularDistance( from.yaw(), normalizeAngle( yaw_des )));
    double yaw_2 = std::abs( shortestAngularDistance( from.yaw(), normalizeAngle( yaw_des + M_PI )));

    return heuristic_without_rotation + std::min( yaw_1, yaw_2 );
  }

  return heuristic_without_rotation;
}

std::vector<l3::PositionIndex> TraversabilityRotationHeuristic::getNeighbors( const PositionIndex &current_index ) const
{
  l3::PositionIndex up = current_index;
  up[0]++;
  l3::PositionIndex down = current_index;
  down[0]--;
  l3::PositionIndex left = current_index;
  left[1]--;
  l3::PositionIndex right = current_index;
  right[1]++;

  return { up, down, left, right };
}

std::vector<l3::PositionIndex>
TraversabilityRotationHeuristic::getValidNeighbors( const std::vector<l3::PositionIndex> &neighbors ) const
{
  std::vector<l3::PositionIndex> valid_neighbors;
  valid_neighbors.reserve( neighbors.size());

  for ( auto &neighbor: neighbors )
  {
    // check if the neighbor is already in the is accessible cache
    if ( is_accessible_.count( neighbor ) && is_accessible_.at( neighbor ))
    {
      valid_neighbors.push_back( neighbor );
      continue;
    }

    // skip if the neighbor is not inside the HLUT
    if ( !hlut_.isIndexInside( neighbor ))
    {
      is_accessible_[neighbor] = false;
      continue;
    }

    // get the position of the neighbor
    l3::Position2D neighbor_pos = hlut_.getPositionFromIndex( neighbor );

    // check if the neighbor is accessible for at least one yaw value
    for ( auto &yaw: angle_bins_ )
    {
      if ( !WorldModel::instance().isAccessible(
        l3::FloatingBase( neighbor_pos.x(), neighbor_pos.y(), 0.0, 0.0, 0.0, yaw )))
        continue;

      is_accessible_[neighbor] = true;
      valid_neighbors.push_back( neighbor );
      break;
    }
  }

  return valid_neighbors;
}

HLUTHeuristicPlugin::hlutEntry
TraversabilityRotationHeuristic::computeHLUTEntryOfNeighbor( const PositionIndex &neighbor,
                                                             const HLUTHeuristicPlugin::hlutEntry &current_entry ) const
{
  l3::PositionIndex neighbor_up = neighbor;
  neighbor_up[0]++;
  l3::PositionIndex neighbor_down = neighbor;
  neighbor_down[0]--;
  l3::PositionIndex neighbor_left = neighbor;
  neighbor_left[1]--;
  l3::PositionIndex neighbor_right = neighbor;
  neighbor_right[1]++;

  float a_1 = hlut_.isIndexInside( neighbor_up ) ? hlut_.getHeuristicEntry( neighbor_up ) : inf;
  float a_2 = hlut_.isIndexInside( neighbor_down ) ? hlut_.getHeuristicEntry( neighbor_down ) : inf;
  float a = std::min( a_1, a_2 );

  float b_1 = hlut_.isIndexInside( neighbor_left ) ? hlut_.getHeuristicEntry( neighbor_left ) : inf;
  float b_2 = hlut_.isIndexInside( neighbor_right ) ? hlut_.getHeuristicEntry( neighbor_right ) : inf;
  float b = std::min( b_1, b_2 );

  l3::Position2D neighbor_pos = hlut_.getPositionFromIndex( neighbor );
  grid_map::Index neighbor_index;

  float traversability = unknown_region_value_;
  if ( traversability_map_.getIndex( neighbor_pos, neighbor_index ))
  {
    traversability = traversability_map_.at( traversability_map_layer_, neighbor_index );
    if ( std::isnan( traversability ))
      traversability = unknown_region_value_;
  }

  float inv_f =
    static_cast<float>(resolution_.resolution().x) / std::pow( traversability, exponential_traversability_weight_ );

  hlutEntry valid_neighbor_entry;
  valid_neighbor_entry.index = neighbor;

  if ( inv_f > std::abs( a - b ))
  {
    float c = a + b;
    valid_neighbor_entry.heuristic_value = 0.5f * (c + std::sqrt( c * c - 2.0f * (a * a + b * b - inv_f * inv_f)));
  }
  else
  {
    valid_neighbor_entry.heuristic_value = std::min( a, b ) + inv_f;
  }

  return valid_neighbor_entry;
}

void TraversabilityRotationHeuristic::visualizeHLUT() const
{
  // publish HLUT
  grid_map::GridMap hlut_grid_map = grid_map::GridMap();
  hlut_grid_map.setFrameId( vis_frame_id_ );
  hlut_grid_map.setGeometry( grid_map::Length( hlut_.getSize().x(), hlut_.getSize().y()),
                             hlut_.getResolution().resolution().x, hlut_.getCenter());
  hlut_grid_map.add( vis_layer_, hlut_.getHeuristicMatrix());

  if ( use_rotation_heuristic_ )
    hlut_grid_map.add( vis_layer_ + "_rotation", rotation_hlut_.getHeuristicMatrix());

  grid_map_msgs::GridMap msg;
  grid_map::GridMapRosConverter::toMessage( hlut_grid_map, msg );
  hlut_pub_.publish( msg );
}

void TraversabilityRotationHeuristic::mapCallback( const grid_map_msgs::GridMapConstPtr &traversability_map_new )
{
  current_traversability_map_ptr_ = traversability_map_new;
}

bool TraversabilityRotationHeuristic::initialize( const vigir_generic_params::ParameterSet &params )
{
  if ( !HLUTHeuristicPlugin::initialize( params ))
    return false;

  // Subscribe topics
  traversability_map_sub_ = nh_.subscribe( traversability_map_topic_, 1, &TraversabilityRotationHeuristic::mapCallback,
                                           this );

  return true;
}
}

#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS( l3_footstep_planning::TraversabilityRotationHeuristic, l3_footstep_planning::HeuristicPlugin )
