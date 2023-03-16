/*
 * Copyright (C) 2021 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

// We'll use a string and the gzmsg command below for a brief example.
// Remove these includes if your plugin doesn't need them.
#include <string>
#include <cstdlib>
#include <ignition/common/Console.hh>
#include <ignition/gazebo/Util.hh>
#include <ignition/gazebo/components/Name.hh>
#include <ignition/gazebo/components/Model.hh>

// This header is required to register plugins. It's good practice to place it
// in the cc file, like it's done here.
#include <ignition/plugin/Register.hh>
#include <maliput/plugin/create_road_network.h>

// Don't forget to include the plugin's header.
#include "PurePursuitMaliput.hh"

// This is required to register the plugin. Make sure the interfaces match
// what's in the header.
IGNITION_ADD_PLUGIN(
    simulation::PurePursuitMaliput,
    ignition::gazebo::System,
    simulation::PurePursuitMaliput::ISystemPostUpdate)

using namespace simulation;


PurePursuitMaliput::PurePursuitMaliput() : System(), ISystemPostUpdate() {
  cmd_vel_pub = node.Advertise<ignition::msgs::Twist>(cmd_vel_topic);
  node.Subscribe(pose_topic, &PurePursuitMaliput::OnPose, this);

  // TODO
  // Instantiates controller
  const std::string maliput_backend="maliput_osm";
  const std::string maliput_osm_resource_path=std::getenv("MALIPUT_OSM_RESOURCE_ROOT");
  const std::map<std::string,std::string> config{
    {"osm_file", maliput_osm_resource_path + "/resources/osm/circuit.osm"}
  };
  rn = maliput::plugin::CreateRoadNetwork(maliput_backend, config);
  controller = std::make_unique<PurePursuitController>(rn->road_geometry(), 5);
}

// Here we implement the PostUpdate function, which is called at every
// iteration.
void PurePursuitMaliput::PostUpdate(const ignition::gazebo::UpdateInfo &_info,
    const ignition::gazebo::EntityComponentManager &_ecm)
{
  // This is a simple example of how to get information from UpdateInfo.
  std::string msg = "Hello, world! Simulation is ";
  if (!_info.paused)
    msg += "not ";
  msg += "paused.";

  // Messages printed with gzmsg only show when running with verbosity 3 or
  // higher (i.e. gz sim -v 3)
  // ignmsg << msg << std::endl;



  maliput::math::Vector3 current_position{pose_msg.pose().position().x(), pose_msg.pose().position().y(), pose_msg.pose().position().z()};
  maliput::math::RollPitchYaw current_orientation;
  current_orientation.SetFromQuaternion({pose_msg.pose().orientation().w(),
                                pose_msg.pose().orientation().x(),
                                pose_msg.pose().orientation().y(),
                                pose_msg.pose().orientation().z()});

  position.x() = pose_msg.pose().position().x();
  position.y() = pose_msg.pose().position().y();
  position.z() = pose_msg.pose().position().z();


  // ignmsg << position.to_str() << std::endl;
  // ignmsg << current_orientation.yaw_angle() << std::endl;


  // Obtains ground thruth
  auto model_entity = _ecm.EntityByComponents(
  ignition::gazebo::components::Model(),
  ignition::gazebo::components::Name("prius_hybrid"));
  auto pose = worldPose(model_entity, _ecm);
  double current_heading = pose.Rot().Euler().Z() - 1.57;
  ignmsg << current_heading << std::endl;
  current_position.x() = pose.Pos().X();
  current_position.y() = pose.Pos().Y();
  current_position.z() = pose.Pos().Z();
  // double current_heading = current_orientation.yaw_angle()
  const double curvature = controller->ComputeCurvature(current_position, current_heading);
  const double k_gain = 6.0;




  // Publish to cmd_vel topic
  ignition::msgs::Twist cmd_vel_msg;
  cmd_vel_msg.mutable_linear()->set_x(5.0);
  cmd_vel_msg.mutable_angular()->set_z(curvature*k_gain);
  cmd_vel_pub.Publish(cmd_vel_msg);


}

void PurePursuitMaliput::OnPose(const ignition::msgs::Odometry &_msg) {
  // Print the pose
  // ignmsg << "Received pose: " << _msg.DebugString() << std::endl;
  // print pose 
  pose_msg = _msg;
}

double PurePursuitController::ComputeCurvature(const maliput::math::Vector3& inertial_position, const double heading){
  const auto goal_position = ComputeGoalPoint(inertial_position);
  const double x = inertial_position.x();
  const double y = inertial_position.y();
  const double delta_r = -(goal_position.x() - x) * std::sin(heading) + (goal_position.y() - y) * std::cos(heading);
  const double curvature = 2. * delta_r / std::pow((goal_position-inertial_position).norm(), 2.);
  return curvature;
}


maliput::math::Vector3 PurePursuitController::ComputeGoalPoint(const maliput::math::Vector3& inertial_position){
  const auto road_position_result = rg_->ToRoadPosition(maliput::api::InertialPosition::FromXyz(inertial_position));
  auto lane = road_position_result.road_position.lane;
  double s_new = road_position_result.road_position.pos.s() + lookahead_distance_;
  if (s_new > lane->length()) {
    const double s_delta = s_new - lane->length();
    const auto lane_end = lane->GetDefaultBranch(maliput::api::LaneEnd::Which::kFinish);
    if (lane_end != std::nullopt) {
      lane = lane_end->lane;
      s_new = s_delta;
    }
  }
  return lane->ToInertialPosition(maliput::api::LanePosition(s_new, 0., 0.)).xyz();
}
