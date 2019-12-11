// Copyright 2019 Zhihao Zhang License MIT

#include "joystick_listener.hpp"
#include "student_helper.hpp"

#include <cstdlib>
#include <memory>
#include <string>
#include <utility>
#include <cmath>

namespace assignment3
{
JoystickListener::JoystickListener(std::string const & zid)
    // setting the values to be used for the publisher/subscriber and config
    // values
    : rclcpp::Node{"z0000000_input_node"}
    , zid_{zid}
{
    // setting up publisher and subscriber
    auto callback = std::bind(
        &JoystickListener::joy_message_callback, this, std::placeholders::_1);
    joystick_input_ = create_subscription<sensor_msgs::msg::Joy>
        (std::string("/" + zid_ + "/joy"), 10, callback);
    pose_output_ = create_publisher<geometry_msgs::msg::PoseStamped>
        (std::string("/" + zid_ + "/pose"), 10);
}
// Function for mapping
double linear_map(double accel, double in_min, double in_max)
{
    if ((in_min <= accel) && (accel <= in_max))
    {
        accel = 0;
    }
    else if (accel > in_max)
    {
        accel = (accel - in_max) * (1 / (1 - in_max));
    }
    else if (accel < in_min)
    {
        accel = (accel - in_min) * (1 / (1 - in_max));
        ;
    }
    return accel;
}
double JoystickListener::get_x(){
    return storedX;
}
double JoystickListener::get_y(){
    return storedY;
}
double JoystickListener::get_z(){
    return storedZ;
}
double elevation (double raise, double lower, double in_min, double in_max, double zMove){
    if ((raise < 0.8) && (lower < 0.8)) {
        zMove = 0;
        return zMove;
    } else if (raise < 0.8 ){
        zMove = abs(raise);
        zMove = linear_map(zMove, in_min, in_max);
        return zMove;
    } else if (lower < 0.8) {
        zMove = -abs(lower);
        zMove = linear_map(zMove, in_min, in_max);
        return zMove;
    }

}
// callback function
auto JoystickListener::joy_message_callback(
    sensor_msgs::msg::Joy::UniquePtr joy_message) -> void
{
    // putting axes values into a vector
    std::vector<float> axesValues{joy_message->axes.at(0),
        joy_message->axes.at(1), joy_message->axes.at(2),
        joy_message->axes.at(5), joy_message->axes.at(3),
        joy_message->axes.at(4)};

    // putting button presses into a vector
    std::vector<double> buttonValues{joy_message->buttons.at(0),
        joy_message->buttons.at(5)};

    // accounting for deadzone in joystick in x direction and mapping
    double in_min = -0.05;
    double in_max = 0.05;
    double xMove = axesValues.at(0);
    storedX = linear_map(xMove, in_min, in_max);

    // account for deadzone in joystick y direction and then mapping
    double yMove = axesValues.at(1);
    storedY = linear_map(yMove, in_min, in_max);

    // account for deadzone in joystick z direction and then mapping
    double zMove = 0;
    zMove = elevation(axesValues.at(2), axesValues.at(3), in_min, in_max, zMove);

    //publishing to topic
    pose_ = std::make_unique<geometry_msgs::msg::PoseStamped>();;
    pose_->header.frame_id = zid_;
    pose_->header.stamp = joy_message->header.stamp;
    pose_->pose.position.x = storedX + xMove/2;
    pose_->pose.position.y = storedY + yMove/2;
    pose_->pose.position.z = storedZ + zMove/5;
    // storing current position to use as previous position
    storedX = pose_->pose.position.x;
    storedY = pose_->pose.position.y;
    storedZ = pose_->pose.position.z;

    pose_output_->publish(std::move(pose_));

};
} // namespace assignment3

