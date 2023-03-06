//=================================================================================================
// Copyright (c) 2012-2016, Institute of Flight Systems and Automatic Control,
// Technische Universität Darmstadt.
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of hector_quadrotor nor the names of its contributors
//       may be used to endorse or promote products derived from this software
//       without specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//=================================================================================================

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <mavros_msgs/OverrideRCIn.h>


namespace usv_sim
{

class Teleop
{
private:

  ros::NodeHandle node_handle_;
  ros::Subscriber joy_subscriber_;
  ros::Publisher joint_publisher_;

  struct Axis
  {
    Axis()
      : axis(0), factor(0.0), offset(0.0)
    {}

    int axis;
    double factor;
    double offset;
  };

  struct Button
  {
    Button()
      : button(0)
    {}

    int button;
  };

  struct
  {
    Axis x;
    Axis y;
    Axis z;
    Axis thrust;
    Axis yaw;
  } axes_;

  struct
  {
    Button slow;
    Button go;
    Button stop;
    Button interrupt;
  } buttons_;

  double slow_factor_;

public:
  Teleop()
  {
    ros::NodeHandle private_nh("~");

    private_nh.param<int>("x_axis", axes_.x.axis, 5);
    private_nh.param<int>("y_axis", axes_.y.axis, -2);
    private_nh.param<int>("z_axis", axes_.z.axis, -1);
    private_nh.param<int>("thrust_axis", axes_.thrust.axis, -3);
    private_nh.param<int>("yaw_axis", axes_.yaw.axis, 1);

    private_nh.param<double>("yaw_velocity_max", axes_.yaw.factor, 30.0);

    private_nh.param<int>("slow_button", buttons_.slow.button, 4);
    private_nh.param<int>("go_button", buttons_.go.button, 1);
    private_nh.param<int>("stop_button", buttons_.stop.button, 2);
    private_nh.param<int>("interrupt_button", buttons_.interrupt.button, 3);
    private_nh.param<double>("slow_factor", slow_factor_, 0.2);

    private_nh.param<double>("pitch_max", axes_.x.factor, 30.0);
    private_nh.param<double>("y_max", axes_.y.factor, 500.0);
    private_nh.param<double>("y_offset", axes_.y.offset, 1500.0);
    private_nh.param<double>("z_max", axes_.z.factor, -500.0);
    private_nh.param<double>("z_offset", axes_.z.offset, 1500.0);
    private_nh.param<double>("thrust_max", axes_.thrust.factor, 30.0);
    private_nh.param<double>("thrust_offset", axes_.thrust.offset, 30.0);
    private_nh.param<double>("yaw_max", axes_.yaw.factor, 30.0);
    private_nh.param<double>("yaw_offset", axes_.yaw.offset, 30.0);

    joy_subscriber_ = node_handle_.subscribe<sensor_msgs::Joy>("joy", 1,
                                                               boost::bind(&Teleop::joyAttitudeCallback, this, _1));

    joint_publisher_ = node_handle_.advertise<mavros_msgs::OverrideRCIn>(
        "/mavros/rc/override", 10);
  }

  ~Teleop()
  {
    stop();
  }

  void joyAttitudeCallback(const sensor_msgs::JoyConstPtr &joy)
  {

    mavros_msgs::OverrideRCIn joint_cmd;
    double rudder, sail;

    joint_cmd.channels[0] = (getAxis(joy, axes_.yaw));
    // joint_cmd.channels[1] = (getAxis(joy, axes_.z));
    joint_cmd.channels[2] = (getAxis(joy, axes_.thrust));
    // joint_cmd.channels[3] = (getAxis(joy, axes_.z));
    // joint_cmd.channels[4] = (getAxis(joy, axes_.z));
    // joint_cmd.channels[5] = (getAxis(joy, axes_.z));
    // joint_cmd.channels[6] = (getAxis(joy, axes_.z));
    // joint_cmd.channels[7] = (getAxis(joy, axes_.z));

    joint_publisher_.publish(joint_cmd);
  }

  double getAxis(const sensor_msgs::JoyConstPtr &joy, const Axis &axis)
  {
    if (axis.axis == 0 || std::abs(axis.axis) > joy->axes.size())
    {
      ROS_ERROR_STREAM("Axis " << axis.axis << " out of range, joy has " << joy->axes.size() << " axes");
      return 0;
    }

    double output = std::abs(axis.axis) / axis.axis * joy->axes[std::abs(axis.axis) - 1] * axis.factor + axis.offset;

    return output;
  }

  bool getButton(const sensor_msgs::JoyConstPtr &joy, const Button &button)
  {
    if (button.button <= 0 || button.button > joy->buttons.size())
    {
      ROS_ERROR_STREAM("Button " << button.button << " out of range, joy has " << joy->buttons.size() << " buttons");
      return false;
    }

    return joy->buttons[button.button - 1] > 0;
  }

  void stop()
  {
    if (joint_publisher_.getNumSubscribers() > 0)
    {
      joint_publisher_.publish(mavros_msgs::OverrideRCIn());
    }
  }
};

} // namespace hector_quadrotor

int main(int argc, char **argv)
{
  ros::init(argc, argv, "usv_teleop");

  usv_sim::Teleop teleop;
  ros::spin();

  return 0;
}
