// Copyright 2019 Zhihao Zhang License MIT

#ifndef JOYSTICK_LISTENER_HPP_
#define JOYSTICK_LISTENER_HPP_

#include "geometry_msgs/msg/pose_stamped.hpp" // http://docs.ros.org/melodic/api/geometry_msgs/html/msg/PoseStamped.html
#include "rclcpp/rclcpp.hpp"       // http://docs.ros2.org/dashing/api/rclcpp/
#include "sensor_msgs/msg/joy.hpp" // http://wiki.ros.org/joy

#include <string>

namespace assignment3
{
class JoystickListener final : public rclcpp::Node
{
public:
    explicit JoystickListener(std::string const & zid);
    double storedX = 0;
    double storedY = 0;
    double storedZ = 0;
    double xSignal = 0;
    double clearFlag = 0;
    
    double get_x();
    double get_y();
    double get_z();
    double get_x_signal();
    double get_clear_flag();

private:
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joystick_input_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_output_;
    rclcpp::TimerBase::SharedPtr timer_;
    geometry_msgs::msg::PoseStamped::UniquePtr pose_;


    std::string const zid_ = "z0000000";
    auto joy_message_callback(sensor_msgs::msg::Joy::UniquePtr joy_message)
        -> void;
};
} // namespace assignment3

#endif // JOYSTICK_LISTENER_HPP_