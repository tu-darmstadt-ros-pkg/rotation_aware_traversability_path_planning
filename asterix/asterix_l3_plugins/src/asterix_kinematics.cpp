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

#include <asterix_l3_plugins/asterix_kinematics.h>

#include <l3_math/angles.h>

namespace asterix_l3
{
AsterixKinematicsPlugin::AsterixKinematicsPlugin() : KdlKinematics( "asterix_kinematics" )
{
}

bool AsterixKinematicsPlugin::loadParams( const vigir_generic_params::ParameterSet &params )
{
  if ( !KdlKinematics::loadParams( params ))
    return false;

  return true;
}

bool AsterixKinematicsPlugin::initialize( const vigir_generic_params::ParameterSet &params )
{
  if ( !KdlKinematics::initialize( params ))
    return false;

  return true;
}

Pose AsterixKinematicsPlugin::calcFeetCenter( const FootholdArray &footholds ) const
{
  Pose pose = l3::calcFeetCenter( footholds );

  auto map = l3::footholdArrayToMap<FootholdMap>( footholds );

  ROS_ASSERT( map.find( L_TRACK ) != map.end());
  ROS_ASSERT( map.find( R_TRACK ) != map.end());

  // set body yaw to track yaw
  pose.setYaw( map[L_TRACK].pose().yaw());

  return pose;
}

Pose AsterixKinematicsPlugin::calcFeetCenter( const FootholdConstPtrArray &footholds ) const
{
  FootholdArray temp;
  for ( Foothold::ConstPtr f: footholds )
    temp.push_back( *f );
  return calcFeetCenter( temp );
}
}  // namespace asterix_l3

#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS( asterix_l3::AsterixKinematicsPlugin, l3::KinematicsPlugin )
