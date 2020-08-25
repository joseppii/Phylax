#include "ros/ros.h"
#include "phylax_msgs/Drive.h"
#include "sensor_msgs/Joy.h"

#include "boost/algorithm/clamp.hpp"

//namespace ackal_teleop
//{

class SimpleJoy
{
public:
  explicit SimpleJoy(ros::NodeHandle* nh);

private:
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);

  ros::NodeHandle* nh_;
  ros::Subscriber joy_sub_;
  ros::Publisher drive_pub_;

  int deadman_button_;
  int axis_linear_;
  int axis_angular_;
  float scale_linear_;
  float scale_angular_;

  bool sent_deadman_msg_;
};

SimpleJoy::SimpleJoy(ros::NodeHandle* nh) : nh_(nh)
{
  joy_sub_ = nh_->subscribe<sensor_msgs::Joy>("/joystick_teleop/joy", 1, &SimpleJoy::joyCallback, this);
  drive_pub_ = nh_->advertise<phylax_msgs::Drive>("cmd_drive", 1, true);

  ros::param::param("~deadman_button", deadman_button_, 4);
  ros::param::param("~axis_linear", axis_linear_, 1);
  ros::param::param("~axis_angular", axis_angular_, 5);
  ros::param::param("~scale_linear", scale_linear_, 1.0f);
  ros::param::param("~scale_angular", scale_angular_, 1.0f);

  sent_deadman_msg_ = false;
}

void SimpleJoy::joyCallback(const sensor_msgs::Joy::ConstPtr& joy_msg)
{
  phylax_msgs::Drive drive_msg;

  if (joy_msg->buttons[deadman_button_])
  {
    drive_msg.mode = phylax_msgs::Drive::MODE_PWM;
    float linear = joy_msg->axes[axis_linear_] * scale_linear_;
    float angular = joy_msg->axes[axis_angular_] * scale_angular_;
    drive_msg.drivers[phylax_msgs::Drive::LEFT] = boost::algorithm::clamp(linear - angular, -1.0, 1.0);
    drive_msg.drivers[phylax_msgs::Drive::RIGHT] = boost::algorithm::clamp(linear + angular, -1.0, 1.0);
    drive_pub_.publish(drive_msg);
    sent_deadman_msg_ = false;
  }
  else
  {
    // When deadman button is released, immediately send a single no-motion command
    // in order to stop the robot.
    if (!sent_deadman_msg_)
    {
      drive_msg.mode = phylax_msgs::Drive::MODE_NONE;
      drive_pub_.publish(drive_msg);
      sent_deadman_msg_ = true;
    }
  }
}

//}  // namespace phylax_teleop

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "phylax_teleop_joy_pwm");

  ros::NodeHandle nh;
  SimpleJoy simple_joy(&nh);

  ros::spin();
}

