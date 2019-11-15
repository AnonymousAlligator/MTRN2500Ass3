// Copyright 2019 Zhihao Zhang License MIT

#ifndef SINGLE_SHAPE_DISPLAY_HPP_
#define SINGLE_SHAPE_DISPLAY_HPP_

#include "interfaces.hpp"
#include "rclcpp/rclcpp.hpp" // http://docs.ros2.org/dashing/api/rclcpp/
#include "visualization_msgs/msg/marker.hpp"

#include <chrono>
#include <memory>
#include <string>

namespace display
{
class SingleShapeDisplay : public rclcpp::Node, public DisplayOutputInterface
{
public:
    explicit SingleShapeDisplay(
        std::string const & zid, std::chrono::milliseconds refresh_period);

private:
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr
        marker_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::shared_ptr<shapes::DisplayableInterface> object_to_be_displayed_;
    std::string zid_;

    auto marker_publisher_callback() -> void;
    auto display_object_imple(
        std::shared_ptr<shapes::DisplayableInterface> display_object)
        -> void override;
};
} // namespace display
#endif // SINGLE_SHAPE_DISPLAY_HPP_
