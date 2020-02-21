/**
*
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
#include <controller_manager/controller_manager.h>
#include "phylax_base/phylax_hardware.h"

typedef std::chrono::system_clock time_source;

void controlLoopThread(phylax_base::PhylaxHardware *pb, controller_manager::ControllerManager* cm, ros::Rate rate)
{
  time_source::time_point last_time = time_source::now();

  while (1)
  {
    std::chrono::system_clock::time_point current_time = time_source::now();
    std::chrono::duration<double> elapsed_time = current_time - last_time;
    
    ros::Duration elapsed(elapsed_time.count());
    last_time = current_time;

    pb->read();
    cm->update(ros::Time::now(), elapsed);
    pb->write();
    rate.sleep();
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "phylax_node");
  ros::NodeHandle controller_nh("");
  
  ROS_INFO("Phylax is active!"); 

  phylax_base::PhylaxHardware phylax;
  controller_manager::ControllerManager cm(&phylax, controller_nh);

  std::bind(controlLoopThread, std::ref(phylax), std::ref(cm), ros::Rate(50));
  
  ros::spin();
  
  ROS_INFO("Phylax is going for a nap!");
  
  return 0;
}

