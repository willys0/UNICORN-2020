/*
 * Copyright (C) 2012 Open Source Robotics Foundation
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
#include <gazebo/gazebo.hh>
#include <ignition/math.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>

namespace gazebo
{
  class AnimatedBox : public ModelPlugin
  {
    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
    {
      // Store the pointer to the model
      this->model = _parent;

        // create the animation
        gazebo::common::PoseAnimationPtr anim(
              // name the animation "test",
              // make it last 10 seconds,
              // and set it on a repeat loop
              new gazebo::common::PoseAnimation("test", 90.0, true));

        gazebo::common::PoseKeyFrame *key;

        // set starting location of the box
        key = anim->CreateKeyFrame(0);
        key->Translation(ignition::math::Vector3d(-1, 6, 0));
        key->Rotation(ignition::math::Quaterniond(0, 0, 0));


        key = anim->CreateKeyFrame(50.0);
        key->Translation(ignition::math::Vector3d(7, 3, 0));
        key->Rotation(ignition::math::Quaterniond(0, 0, 0));


        /*key = anim->CreateKeyFrame(45.0);
        key->Translation(ignition::math::Vector3d(5, 5, 0));
        key->Rotation(ignition::math::Quaterniond(0, 0, 0));


        key = anim->CreateKeyFrame(60.0);
        key->Translation(ignition::math::Vector3d(-5, 5, 0));
        key->Rotation(ignition::math::Quaterniond(0, 0, 0));*/

        // set final location equal to starting location
        key = anim->CreateKeyFrame(90.0);
        key->Translation(ignition::math::Vector3d(-1, 6, 0));
        key->Rotation(ignition::math::Quaterniond(0, 0, 0));
        // set the animation
        _parent->SetAnimation(anim);
    }
    // Pointer to the model
    private: physics::ModelPtr model;

    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(AnimatedBox)
  // export GAZEBO_PLUGIN_PATH=`pwd`:$GAZEBO_PLUGIN_PATH
}
