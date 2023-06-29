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

  std::string foot_type = param( "foot_type", std::string( "ball_foot" ), true );
  if ( foot_type == "ball_foot" )
    use_ball_foot_ = true;
  else
    use_ball_foot_ = false;

  return true;
}

bool AsterixKinematicsPlugin::initialize( const vigir_generic_params::ParameterSet &params )
{
  if ( !KdlKinematics::initialize( params ))
    return false;

  neutral_stance_.resize( 2 );
  neutral_stance_[L_TRACK] = { 0.013089439258670232, 1.0788590434098584, -1.6985821284268727 };
  neutral_stance_[R_TRACK] = { neutral_stance_[L_TRACK][0], -neutral_stance_[L_TRACK][1], neutral_stance_[L_TRACK][2] };

  return true;
}

Pose AsterixKinematicsPlugin::calcFeetCenter( const FootholdArray &footholds ) const
{
  Pose pose = l3::calcFeetCenter( footholds );

  FootholdMap map = l3::footholdArrayToMap<FootholdMap>( footholds );

  ROS_ASSERT( map.find( L_TRACK ) != map.end());
  ROS_ASSERT( map.find( R_TRACK ) != map.end());

  // estimate body yaw using computing the vectors between the diagonal opposing foot poses
  Vector3 lf = map[L_TRACK].pose().getPosition() + Vector3( 0.36, 0.0, 0.0 );
  Vector3 rf = map[R_TRACK].pose().getPosition() + Vector3( 0.36, 0.0, 0.0 );
  Vector3 lh = map[L_TRACK].pose().getPosition() - Vector3( 0.36, 0.0, 0.0 );
  Vector3 rh = map[R_TRACK].pose().getPosition() - Vector3( 0.36, 0.0, 0.0 );

  Vector3 vec1 = lf - rh;
  Vector3 vec2 = rf - lh;

  double yaw1 = atan2( vec1.y(), vec1.x());
  double yaw2 = atan2( vec2.y(), vec2.x());
  double dyaw = shortestAngularDistance( yaw1, yaw2 );

  pose.setYaw( yaw1 + 0.5 * dyaw );

  return pose;
}

Pose AsterixKinematicsPlugin::calcFeetCenter( const FootholdConstPtrArray &footholds ) const
{
  FootholdArray temp;
  for ( Foothold::ConstPtr f: footholds )
    temp.push_back( *f );
  return calcFeetCenter( temp );
}

bool AsterixKinematicsPlugin::calcLegIK( const Pose &base_pose, const Foothold &foothold, const std::vector<double> &cur_q,
                                      std::vector<double> &q ) const
{
  // provide better start point for KDL
  if ( cur_q.empty() && foothold.idx < neutral_stance_.size())
    return KdlKinematics::calcLegIK( base_pose, foothold, neutral_stance_[foothold.idx], q );
  else
    return KdlKinematics::calcLegIK( base_pose, foothold, cur_q, q );
}
}  // namespace asterix_l3

#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS( asterix_l3::AsterixKinematicsPlugin, l3::KinematicsPlugin )
