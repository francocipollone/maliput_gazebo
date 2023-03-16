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

#ifndef SYSTEM_PLUGIN_HELLOWORLD_HH_
#define SYSTEM_PLUGIN_HELLOWORLD_HH_

// The only required include in the header is this one.
// All others will depend on what your plugin does.
#include <memory>
#include <ignition/gazebo/System.hh>
#include <ignition/transport/Node.hh>
#include <ignition/transport/TransportTypes.hh>
#include <maliput/math/vector.h>
#include <maliput/math/roll_pitch_yaw.h>
#include <maliput/api/road_geometry.h>

// It's good practice to use a custom namespace for your project.
namespace simulation
{

  class PurePursuitController {
   public:
    PurePursuitController(const maliput::api::RoadGeometry* rg, double lookahead_distance) {
      rg_=rg;
      lookahead_distance_ = lookahead_distance;
    }

   double ComputeCurvature(const maliput::math::Vector3& inertial_position, const double heading);

   private:

    maliput::math::Vector3 ComputeGoalPoint(const maliput::math::Vector3& inertial_position);

    const maliput::api::RoadGeometry* rg_;
    double lookahead_distance_;
  };

  // This is the main plugin's class. It must inherit from System and at least
  // one other interface.
  // Here we use `ISystemPostUpdate`, which is used to get results after
  // physics runs. The opposite of that, `ISystemPreUpdate`, would be used by
  // plugins that want to send commands.
  class PurePursuitMaliput:
    public ignition::gazebo::System,
    public ignition::gazebo::ISystemPostUpdate
  {
    public:
      PurePursuitMaliput();

      // Plugins inheriting ISystemPostUpdate must implement the PostUpdate
      // callback. This is called at every simulation iteration after the physics
      // updates the world. The _info variable provides information such as time,
      // while the _ecm provides an interface to all entities and components in
      // simulation.
      void PostUpdate(const ignition::gazebo::UpdateInfo &_info,
                  const ignition::gazebo::EntityComponentManager &_ecm) override;


      // onpose callback
      void OnPose(const ignition::msgs::Odometry &_msg);
    private:

      const std::string cmd_vel_topic = "/cmd_vel";
      const std::string pose_topic = "/model/prius_hybrid/odometry";
      ignition::transport::Node node;
      ignition::transport::Node::Publisher cmd_vel_pub;
      maliput::math::Vector3 position;
      maliput::math::RollPitchYaw orientation;
      ignition::msgs::Odometry pose_msg;
      std::unique_ptr<maliput::api::RoadNetwork> rn;
      std::unique_ptr<PurePursuitController> controller;
  };





}
#endif