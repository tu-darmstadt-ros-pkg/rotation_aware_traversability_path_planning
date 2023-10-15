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

#include <traversability_map_generator/plugins/std/generator/grid_map_merger.h>

namespace l3_terrain_modeling
{
GridMapMerger::GridMapMerger() : GridMapGeneratorPlugin( "grid_map_merger" )
{
}

bool GridMapMerger::loadParams( const vigir_generic_params::ParameterSet &params )
{
  if ( !GridMapGeneratorPlugin::loadParams( params ))
    return false;

  topic_ = param( "topic", std::string( "/traversability_estimation/traversability_map" ));
  if ( !getParam( "in_layers", in_layers_, XmlRpc::XmlRpcValue()))
    return false;
  if ( !getParam( "out_layers", out_layers_, XmlRpc::XmlRpcValue()))
    return false;
  if ( !getParam( "apply_smoothing", apply_smoothing_, XmlRpc::XmlRpcValue()))
    return false;
  resolution_ = param( "resolution", 0.02 );
  if ( !getParam( "unknown_area_value", unknown_area_value_, XmlRpc::XmlRpcValue()))
    return false;
  if ( !getParam( "gaussian_kernel_width", gaussian_kernel_width_, XmlRpc::XmlRpcValue()))
    return false;
  if ( !getParam( "gaussian_kernel_height", gaussian_kernel_height_, XmlRpc::XmlRpcValue()))
    return false;
  if ( !getParam( "sigma_x", sigma_x_, XmlRpc::XmlRpcValue()))
    return false;
  if ( !getParam( "sigma_y", sigma_y_, XmlRpc::XmlRpcValue()))
    return false;
  if ( !getParam( "erosion_kernel_width", erosion_kernel_width_, XmlRpc::XmlRpcValue()))
    return false;
  if ( !getParam( "erosion_kernel_height", erosion_kernel_height_, XmlRpc::XmlRpcValue()))
    return false;
  if ( in_layers_.getType() != XmlRpc::XmlRpcValue::TypeArray ||
       out_layers_.getType() != XmlRpc::XmlRpcValue::TypeArray ||
       apply_smoothing_.getType() != XmlRpc::XmlRpcValue::TypeArray ||
       unknown_area_value_.getType() != XmlRpc::XmlRpcValue::TypeArray ||
       gaussian_kernel_width_.getType() != XmlRpc::XmlRpcValue::TypeArray ||
       gaussian_kernel_height_.getType() != XmlRpc::XmlRpcValue::TypeArray ||
       sigma_x_.getType() != XmlRpc::XmlRpcValue::TypeArray || sigma_y_.getType() != XmlRpc::XmlRpcValue::TypeArray ||
       erosion_kernel_width_.getType() != XmlRpc::XmlRpcValue::TypeArray ||
       erosion_kernel_height_.getType() != XmlRpc::XmlRpcValue::TypeArray || in_layers_.size() != out_layers_.size() ||
       out_layers_.size() != apply_smoothing_.size())
    return false;

  return true;
}

bool GridMapMerger::initialize( const vigir_generic_params::ParameterSet &params )
{
  input_handle_ = DataManager::addData( this, std::string( "in_grid_map" ), std::move( grid_map::GridMap()));
  if ( !GridMapGeneratorPlugin::initialize( params ))
    return false;

  l3::UniqueLockPtr lock;
  auto &grid_map = grid_map_handle_->value<grid_map::GridMap>( lock );

  grid_map_sub_ = nh_.subscribe( topic_, 1, &GridMapMerger::gridMapCb, this );

  return true;
}

void GridMapMerger::processImpl( const Timer &timer, UpdatedHandles &input, const SensorPlugin *sensor )
{
  if ( !grid_map_handle_ )
    return;

  // Call update routine
  update( timer, input, sensor );

  input.insert( grid_map_handle_ );
}

void GridMapMerger::update( const Timer &timer, UpdatedHandles &input, const SensorPlugin *sensor )
{
  l3::SharedLockPtr in_lock;
  const auto &in_grid_map = input_handle_->value<grid_map::GridMap>( in_lock );

  l3::UniqueLockPtr out_lock;
  auto &out_grid_map = grid_map_handle_->value<grid_map::GridMap>( out_lock );

  // Copy header information
  out_grid_map.setFrameId( in_grid_map.getFrameId());
  out_grid_map.setGeometry( in_grid_map.getLength(), in_grid_map.getResolution(), in_grid_map.getPosition());
  out_grid_map.setStartIndex( in_grid_map.getStartIndex());
  out_grid_map.setTimestamp( in_grid_map.getTimestamp());

  for ( int i = 0; i < in_layers_.size(); ++i )
  {
    std::string in_layer = in_layers_[i];
    std::string out_layer = out_layers_[i];
    bool apply_smoothing = apply_smoothing_[i];

    out_grid_map.add( out_layer, in_grid_map.get( in_layer ));

    if ( apply_smoothing )
    {
      double unknown_area_value = unknown_area_value_[i];
      int gaussian_kernel_width = gaussian_kernel_width_[i];
      int gaussian_kernel_height = gaussian_kernel_height_[i];
      double sigma_x = sigma_x_[i];
      double sigma_y = sigma_y_[i];
      int erosion_kernel_width = erosion_kernel_width_[i];
      int erosion_kernel_height = erosion_kernel_height_[i];

      // Convert grid map to image
      cv::Mat grid_map_image;
      grid_map::GridMapCvConverter::toImage<unsigned char, 4>( in_grid_map, in_layer, CV_8UC4, grid_map_image );

      // Use erosion to inflate obstacles
      cv::Mat erosion;
      cv::Mat element = getStructuringElement( cv::MORPH_RECT, cv::Size( erosion_kernel_width, erosion_kernel_height ));
      erode( grid_map_image, erosion, element );

      // Split image into different channels
      cv::Mat grid_map_image_channels[4];
      cv::split( erosion, grid_map_image_channels );

      // Set the value of the image pixels representing unknown values to the specified unknown_area_value
      grid_map_image_channels[1].setTo( unknown_area_value * 255, grid_map_image_channels[3] == 0 );

      // Use Gaussian Blur to smooth the image
      cv::Mat grid_map_image_smoothed;
      cv::GaussianBlur( grid_map_image_channels[1], grid_map_image_smoothed,
                        cv::Size( gaussian_kernel_width, gaussian_kernel_height ), sigma_x, sigma_y );

      // Use the smaller value per pixel of the smoothed and non-smoothed grid map image
      cv::Mat grid_map_image_min;
      cv::min( grid_map_image_channels[1], grid_map_image_smoothed, grid_map_image_min );

      // Convert the image back to a grid map
      grid_map::GridMapCvConverter::addLayerFromImage<unsigned char, 1>( grid_map_image_min, out_layer + "_smoothed",
                                                                         out_grid_map );
    }
  }
}

void GridMapMerger::gridMapCb( const grid_map_msgs::GridMap &grid_map )
{
  // TODO: Use shared pointer instead of deepcopy.
  l3::UniqueLockPtr in_lock;
  auto &in_grid_map = input_handle_->value<grid_map::GridMap>( in_lock );

  grid_map::GridMapRosConverter::fromMessage( grid_map, in_grid_map );

  in_lock.reset();

  // Workaround to get publishers
  UpdatedHandles updates;
  updates.insert( input_handle_ );

  process( in_grid_map.getTimestamp(), updates, nullptr );
}
}  // namespace l3_terrain_modeling

#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS( l3_terrain_modeling::GridMapMerger, l3_terrain_modeling::ProcessorPlugin )
