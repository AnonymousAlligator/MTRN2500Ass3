// Copyright 2019 Zhihao Zhang License MIT

#include "single_shape_display.hpp"

#include "student_helper.hpp"

#include <memory>
#include <string>
#include <vector>

namespace
{
auto constexpr marker_topic = [](std::string const & zid) {
    return "/" + zid + "/marker";
};
} // namespace

namespace display
{
SingleShapeDisplay::SingleShapeDisplay(
    std::string const & zid, std::chrono::milliseconds const refresh_period)
    : rclcpp::Node{helper::marker_node_name(zid)}
    , marker_publisher_{create_publisher<visualization_msgs::msg::Marker>(
          marker_topic(zid), 10)}
    , timer_{create_wall_timer(
          refresh_period, [this]() -> void { marker_publisher_callback(); })}
    , zid_{zid}
{
}

auto SingleShapeDisplay::display_object_imple(
    std::shared_ptr<shapes::DisplayableInterface> const display_object) -> void
{
    object_to_be_displayed_ = display_object;
}

// ReSharper disable once CppMemberFunctionMayBeConst
auto SingleShapeDisplay::marker_publisher_callback() -> void
{
    auto const shapes_list = object_to_be_displayed_->get_display_markers();
    if (shapes_list)
    {
        for (auto & shape : *shapes_list)
        {
            shape.header.stamp = rclcpp::Time{0};
            marker_publisher_->publish(shape);
        }
    }
}
} // namespace display
