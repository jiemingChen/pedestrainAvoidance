/*
 * Copyright (C) 2016 Open Source Robotics Foundation
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

#include <functional>
#include <iostream>
#include <ignition/math.hh>
#include "gazebo/physics/physics.hh"
#include "my_actor.h"
#include <ros/console.h>
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/subscribe_options.h>
#include <std_msgs/Float32MultiArray.h>
#include <tf/transform_broadcaster.h>

using namespace gazebo;
GZ_REGISTER_MODEL_PLUGIN(ActorPlugin)

#define WALKING_ANIMATION "walking"

/////////////////////////////////////////////////
ActorPlugin::ActorPlugin()
{
}

/////////////////////////////////////////////////
void ActorPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  std::cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!shittttt!!!!!!!!!!!!!!!!!!!!!!!!"<<std::endl;
  this->sdf = _sdf;
  this->actor = boost::dynamic_pointer_cast<physics::Actor>(_model);
  this->world = this->actor->GetWorld();

  this->connections.push_back(event::Events::ConnectWorldUpdateBegin(
          std::bind(&ActorPlugin::OnUpdate, this, std::placeholders::_1)));

  this->Reset();

  // Read in the target weight
  if (_sdf->HasElement("target_weight"))
    this->targetWeight = _sdf->Get<double>("target_weight");
  else
    this->targetWeight = 1.15;
  // Read in the initial pose
  if (_sdf->HasElement("initial_pose"))
    this->initial_pose = _sdf->Get<ignition::math::Vector3d>("initial_pose");
  // Read in the animation factor (applied in the OnUpdate function).
  if (_sdf->HasElement("animation_factor"))
    this->animationFactor = _sdf->Get<double>("animation_factor");
  else
    this->animationFactor = 4.5;

  // Add our own name to models we should ignore when avoiding obstacles.
  this->ignoreModels.push_back(this->actor->GetName());

  // Read in the other obstacles to ignore
  if (_sdf->HasElement("ignore_obstacles"))
  {
    sdf::ElementPtr modelElem =
      _sdf->GetElement("ignore_obstacles")->GetElement("model");
    while (modelElem)
    {
      this->ignoreModels.push_back(modelElem->Get<std::string>());
      modelElem = modelElem->GetNextElement("model");
    }
  }

    if (!ros::isInitialized())
    {
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, "gazebo_client",
                ros::init_options::NoSigintHandler);
    }
    this->rosNode.reset(new ros::NodeHandle("gazebo_client"));

    this->rosNode->setCallbackQueue(&this->rosQueue);
    this->GetVelService = this->rosNode->advertiseService("/"+this->actor->GetName()+"/GetActorVelocity",
                                                          &ActorPlugin::GetVelCallback, this);

    this->rosQueueThread = std::thread(std::bind(&ActorPlugin::QueueThread, this));
}

/////////////////////////////////////////////////
void ActorPlugin::Reset()
{
  this->velocity = 0.5;
  this->lastUpdate = 0;

  if (this->sdf && this->sdf->HasElement("target"))
    this->target = this->sdf->Get<ignition::math::Vector3d>("target");
  else
    this->target = ignition::math::Vector3d(0, -5, 1.2138);

  this->initial_target = this->target ;

  auto skelAnims = this->actor->SkeletonAnimations();
  if (skelAnims.find(WALKING_ANIMATION) == skelAnims.end())
  {
    gzerr << "Skeleton animation " << WALKING_ANIMATION << " not found.\n";
  }
  else
  {
    // Create custom trajectory
    this->trajectoryInfo.reset(new physics::TrajectoryInfo());
    this->trajectoryInfo->type = WALKING_ANIMATION;
    this->trajectoryInfo->duration = 1.0;

    this->actor->SetCustomTrajectory(this->trajectoryInfo);
  }

}

/////////////////////////////////////////////////
void ActorPlugin::ChooseNewTarget()
{
  if(flag==0){
    ignition::math::Vector3d newTarget(this->initial_pose);
    flag = 1;
    this->target = newTarget;

  }
  else{
    ignition::math::Vector3d newTarget(this->initial_target);
    flag = 0;
    this->target = newTarget;
  }

  #if 0
  while ((newTarget - this->target).Length() < 2.0)
  {
    newTarget.X(ignition::math::Rand::DblUniform(-3, 3.5));
    newTarget.Y(ignition::math::Rand::DblUniform(-10, 2));

    for (unsigned int i = 0; i < this->world->ModelCount(); ++i)
    {
      double dist = (this->world->ModelByIndex(i)->WorldPose().Pos()
          - newTarget).Length();
      if (dist < 2.0)
      {
        newTarget = this->target;
        break;
      }
    }
  }
  #endif

}

/////////////////////////////////////////////////
#if 0
void ActorPlugin::HandleObstacles(ignition::math::Vector3d &_pos)
{
  for (unsigned int i = 0; i < this->world->ModelCount(); ++i)
  {
    physics::ModelPtr model = this->world->ModelByIndex(i);
    if (std::find(this->ignoreModels.begin(), this->ignoreModels.end(),
          model->GetName()) == this->ignoreModels.end())
    {
      ignition::math::Vector3d offset = model->WorldPose().Pos() -
        this->actor->WorldPose().Pos();
      double modelDist = offset.Length();
      if (modelDist < 4.0)
      {
        double invModelDist = this->obstacleWeight / modelDist;
        offset.Normalize();
        offset *= invModelDist;
        _pos -= offset;
      }
    }
  }
}
#endif
/////////////////////////////////////////////////

void ActorPlugin::OnUpdate(const common::UpdateInfo &_info)
{
  // Time delta
  double dt = (_info.simTime - this->lastUpdate).Double();
  // Current pose
  ignition::math::Pose3d pose = this->actor->WorldPose();
  // Current - Target pose
  ignition::math::Vector3d pos = this->target - pose.Pos();
  ignition::math::Vector3d rpy = pose.Rot().Euler();

  double distance = pow(pos.X()*pos.X() + pos.Y()*pos.Y(), 0.5) ;
  // Choose a new target position if the actor has reached its current
  // target.
  if (distance <= 0.3)
  {
    this->ChooseNewTarget();
    pos = this->target - pose.Pos();
    // pos =  pose.Pos() -  pose.Pos();
  }
  // Normalize the direction vector, and apply the target weight
  pos = pos.Normalize() * this->targetWeight;

  // Adjust the direction vector by avoiding obstacles
//  this->HandleObstacles(pos);

  // Compute the yaw orientation
  ignition::math::Angle yaw = atan2(pos.Y(), pos.X()) + 1.5707 - rpy.Z();
  yaw.Normalize();

  // Rotate in place, instead of jumping.
  if (std::abs(yaw.Radian()) > IGN_DTOR(10))
  {
    pose.Rot() = ignition::math::Quaterniond(1.5707, 0, rpy.Z()+
        yaw.Radian()*0.001);
      this->velocity_real = this->velocity;
  }
  else
  {
    pose.Pos() += pos.Normalize()* this->velocity * dt;
    pose.Rot() = ignition::math::Quaterniond(1.5707, 0, rpy.Z()+yaw.Radian());
      this->velocity_real = pos.Normalize()* this->velocity;

  }
  // Make sure the actor stays within bounds
  //pose.Pos().X(std::max(-3.0, std::min(3.5, pose.Pos().X())));
  //pose.Pos().Y(std::max(-10.0, std::min(2.0, pose.Pos().Y())));
  pose.Pos().Z(1);

  // Distance traveled is used to coordinate motion with the walking
  // animation
  double distanceTraveled = (pose.Pos() -
      this->actor->WorldPose().Pos()).Length();

  this->actor->SetWorldPose(pose, false, false);
  this->actor->SetScriptTime(this->actor->ScriptTime() +
    (distanceTraveled * this->animationFactor));
  this->lastUpdate = _info.simTime;
//    ros::spinOnce();
}

bool ActorPlugin::GetVelCallback(actor_plugin::GetVel::Request& req, actor_plugin::GetVel::Response& res){
    res.x = this->velocity_real.X();
    res.y = this->velocity_real.Y();

//    res.yaw = this->yaw_vel;
    if (req.set_flag == true)
    {
        this->velocity.X(req.new_x);
        this->velocity.Y(req.new_y);
//        this->yaw_vel = req.new_yaw;
    }
    return true;
}

/// \brief ROS helper function that processes messages
void ActorPlugin::QueueThread()
{
    // It gonna be really slow if you change it to 0
    static const double timeout = 0.01;
    while (this->rosNode->ok())
    {
        this->rosQueue.callAvailable(ros::WallDuration(timeout));
    }
}
