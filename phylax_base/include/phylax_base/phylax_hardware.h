/**
*  \file       phylax_base.h 
*  \brief      Declaration of class abstracting the Phylax hardware
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

#ifndef PHYLAX_BASE_PHYLAX_HARDWARE_H
#define PHYLAX_BASE_PHYLAX_HARDWARE_H
#include "boost/thread.hpp"
#include <controller_manager/controller_manager.h>
#include "realtime_tools/realtime_publisher.h"
#include "hardware_interface/joint_state_interface.h"
#include "hardware_interface/joint_command_interface.h"
#include "hardware_interface/robot_hw.h"
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include "phylax_msgs/Drive.h"
#include "phylax_msgs/Feedback.h"
#include "ros/ros.h"

namespace phylax_base
{
class PhylaxHardware : public hardware_interface::RobotHW
{
public:
  PhylaxHardware();

  void read();
  void write();

private:
  void feedbackCallback(const phylax_msgs::Feedback::ConstPtr& msg);
  
  ros::NodeHandle nh_;

  ros::Subscriber feedback_sub_;
  realtime_tools::RealtimePublisher<phylax_msgs::Drive> cmd_pub_;

  hardware_interface::JointStateInterface joint_state_interface_;
  hardware_interface::VelocityJointInterface velocity_joint_interface_;

  struct Joint
  {
    double position;
    double velocity;
    double effort;
    double velocity_command;

    Joint() : position(0), velocity(0), effort(0), velocity_command(0)
    {
    }
  }
  joints_[6];  
  
  phylax_msgs::Feedback::ConstPtr feedback_msg_;
  boost::mutex feedback_mutex_;
};

}  // namespace phylax_base

#endif  // PHYLAX_BASE_PHYLAX_HARDWARE_H
