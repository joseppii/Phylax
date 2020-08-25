/**
*  \file       phylax_hardware.cpp 
*  \brief      Implementation of class abstracting the Phylax hardware
*  \author     Joseph Piperakis <i.piperakis@gmail.com>
*  \copyright  Copyright (c) 2020, Joseph Piperakis.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*     * Redistributions of source code must retain the above copyright
*       notice, this list of conditions and the following disclaimer.
*     * Redistributions in binary form must reproduce the above copyright
*       notice, this list of conditions and the following disclaimer in the
*       documentation and/or other materials provided with the distribution.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
* ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL CLEARPATH ROBOTICS, INC. BE LIABLE FOR ANY
* DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
* ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
* SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
* Please send comments, questions, or patches to the author i.piperakis@gmail.com
*
*/

#include <boost/assign.hpp>
#include "phylax_base/phylax_hardware.h"

namespace phylax_base
{

PhylaxHardware::PhylaxHardware():nh_("~"){
  ros::V_string joint_names = {"front_left_wheel_joint", "front_right_wheel_joint", "center_left_wheel_joint", "center_right_wheel_joint", "rear_left_wheel_joint", "rear_right_wheel_joint"};

  for (unsigned int i = 0; i < joint_names.size(); i++)
  {
    hardware_interface::JointStateHandle joint_state_handle(joint_names[i],
        &joints_[i].position, &joints_[i].velocity, &joints_[i].effort);
    joint_state_interface_.registerHandle(joint_state_handle);

    hardware_interface::JointHandle joint_handle(
        joint_state_handle, &joints_[i].velocity_command);
    velocity_joint_interface_.registerHandle(joint_handle);
  }

  registerInterface(&joint_state_interface_);
  registerInterface(&velocity_joint_interface_);

  feedback_sub_ = nh_.subscribe("feedback", 1, &PhylaxHardware::feedbackCallback, this);
  cmd_pub_.init(nh_, "cmd_drive", 1);    

}

void PhylaxHardware::feedbackCallback(const phylax_msgs::Feedback::ConstPtr& msg)
{
  boost::mutex::scoped_lock lock(feedback_mutex_);
  feedback_msg_ = msg;
//  ROS_INFO("Mpeeeeeeeeeeeeee");
}

void PhylaxHardware::read() 
{
  boost::mutex::scoped_lock feedback_lock(feedback_mutex_, boost::try_to_lock);
  if (feedback_msg_ && feedback_lock)
  {
    for (int i = 0; i < 6; i++)
    {
      joints_[i].position = feedback_msg_->drivers[i % 2].measured_travel;
      joints_[i].velocity = feedback_msg_->drivers[i % 2].measured_velocity;

//	ROS_INFO("Velocity: %f",feedback_msg_->drivers[i % 2].measured_velocity);            
//	ROS_INFO("Travel: %f",feedback_msg_->drivers[i % 2].measured_travel);
    }
  }
}

void PhylaxHardware::write() {
  if (cmd_pub_.trylock())
  {
    cmd_pub_.msg_.mode = phylax_msgs::Drive::MODE_VELOCITY;
    cmd_pub_.msg_.drivers[phylax_msgs::Drive::LEFT] = joints_[0].velocity_command;
    cmd_pub_.msg_.drivers[phylax_msgs::Drive::RIGHT] = joints_[1].velocity_command;
    cmd_pub_.unlockAndPublish();
  }
}

} //namespace phylax_base 
