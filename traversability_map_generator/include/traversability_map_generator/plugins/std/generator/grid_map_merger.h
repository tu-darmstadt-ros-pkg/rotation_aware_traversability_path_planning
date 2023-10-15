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

#pragma once

#include <l3_terrain_model/terrain_model.h>

#include <l3_terrain_model_generator/plugins/base/grid_map_generator_plugin.h>

#include <grid_map_ros/grid_map_ros.hpp>

#include <grid_map_cv/GridMapCvConverter.hpp>

namespace l3_terrain_modeling
{
class GridMapMerger : public GridMapGeneratorPlugin
{
public:
  // typedefs
  typedef l3::SharedPtr<GridMapMerger> Ptr;
  typedef l3::SharedPtr<const GridMapMerger> ConstPtr;

  GridMapMerger();

  bool loadParams( const vigir_generic_params::ParameterSet &params ) override;

  bool initialize( const vigir_generic_params::ParameterSet &params ) override;

protected:
  void processImpl( const Timer &timer, UpdatedHandles &input, const SensorPlugin *sensor ) override;

  void update( const Timer &timer, UpdatedHandles &input, const SensorPlugin *sensor ) override;

  std_msgs::Header getDataHeader() override { return GridMapGeneratorPlugin::getDataHeader(); }

  void getDataBoundary( l3::Vector3 &min, l3::Vector3 &max ) override
  {
    GridMapGeneratorPlugin::getDataBoundary( min, max );
  }

  /**
   * @brief Callback for the input grid map.
   * @param grid_map The new input grid map
   */
  void gridMapCb( const grid_map_msgs::GridMap &grid_map );

  // The topic of the input grid map
  std::string topic_;

  // The layers of the input grid map to merge
  XmlRpc::XmlRpcValue in_layers_;

  // The layers of the output grid map (corresponding to the order of in_layers_)
  XmlRpc::XmlRpcValue out_layers_;

  // Specifies whether to apply smoothing to the layer (corresponding to the order of in_layers_)
  XmlRpc::XmlRpcValue apply_smoothing_;

  // The resolution
  double resolution_;

  // The subscriber for the input grid map
  ros::Subscriber grid_map_sub_;

  // The value for unknown areas (corresponding to the order of in_layers_)
  XmlRpc::XmlRpcValue unknown_area_value_;

  // The erosion kernel width (corresponding to the order of in_layers_)
  XmlRpc::XmlRpcValue erosion_kernel_width_;

  // The erosion kernel height (corresponding to the order of in_layers_)
  XmlRpc::XmlRpcValue erosion_kernel_height_;

  // The gaussian kernel width (corresponding to the order of in_layers_)
  XmlRpc::XmlRpcValue gaussian_kernel_width_;

  // The gaussian kernel height (corresponding to the order of in_layers_)
  XmlRpc::XmlRpcValue gaussian_kernel_height_;

  // The sigma x for the Gaussian Blur (corresponding to the order of in_layers_)
  XmlRpc::XmlRpcValue sigma_x_;

  // The sigma y for the Gaussian Blur (corresponding to the order of in_layers_)
  XmlRpc::XmlRpcValue sigma_y_;
};
}  // namespace l3_terrain_modeling
