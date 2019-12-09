// Copyright 2019 Zhihao Zhang License MIT
// Edited by Curtis Ly (z5209698)

#include "tri_pyr.hpp"

#include "rclcpp/rclcpp.hpp" // http://docs.ros2.org/dashing/api/rclcpp/
#include "student_helper.hpp"

#include <chrono>
#include <memory>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

namespace shapes
{
// Implementation of the triangle class
TriPyr::TriPyr(int id)
    : length_{2.0}
    , parent_frame_name_{"local_frame"}
    , shapes_list_ptr_{
          std::make_shared<std::vector<visualization_msgs::msg::Marker>>()}
{
    // Get a ref to the vector of marker for ease of use
    auto & shapes_list = *shapes_list_ptr_;
    // Create a new marker
    shapes_list.emplace_back();

    // Get a ref to the new marker
    auto & shape = shapes_list[0];
    // Parent frame name
    shape.header.frame_id = helper::world_frame_name("z0000000");
    // body.header.stamp

    // Namespace the marker will be in
    shape.ns = "";
    // Used to identify which marker we are adding/modifying/deleting
    shape.id = id;
    // Type of marker we want to display
    shape.type = visualization_msgs::msg::Marker::TRIANGLE_LIST;
    // Add, modify or delete.
    shape.action = visualization_msgs::msg::Marker::ADD;

    // Position
    shape.pose.position.x = -3;
    shape.pose.position.y = -3;
    shape.pose.position.z = 1.5;

    // Orientation in quaternion
    shape.pose.orientation.x = 0;
    shape.pose.orientation.y = 0;
    shape.pose.orientation.z = 0;
    shape.pose.orientation.w = 1;

    // Scale change the dimension of the sides
    shape.scale.x = 3.0;
    shape.scale.y = 3.0;
    shape.scale.z = 3.0;

    // Make the triangle brick-red coloured
    shape.color.r = 1.0;
    shape.color.g = 0.6;
    shape.color.b = 0.6;
    shape.color.a = 1.0;

    // Base of pyramid
    geometry_msgs::msg::Point p;
    p.x = -length_.get_value();
    p.y = -length_.get_value();
    p.z = length_.get_value();

    geometry_msgs::msg::Point p2 = p;
    p2.x = p.x + 2;

    geometry_msgs::msg::Point p3 = p;
    p2.x = p.x + sqrt(5);
    p3.y = p.y + 2;

    shape.points.push_back(p);
    shape.points.push_back(p2);
    shape.points.push_back(p3);

    // body.colors.emplace_back();
    using namespace std::chrono_literals;
    shape.lifetime =
        rclcpp::Duration{1s}; // How long our marker message is valid for
}

// Shape manipulation functions
auto TriPyr::resize_imple(AllAxis const new_size) -> void {length_ = new_size;}

auto TriPyr::rescale_imple(AnyAxis const factor) -> void
{
    length_ = AllAxis{length_.get_value() * factor.get_value()};
}

auto TriPyr::get_colour_imple() const -> Colour {return Colour::black;}

auto TriPyr::set_parent_frame_name_imple(std::string frame_name) -> void
{
    parent_frame_name_ = std::move(frame_name);
}

auto TriPyr::get_location_imple() const -> std::tuple<XAxis, YAxis, ZAxis>
{
    return std::tuple{XAxis{1.0}, YAxis{2.0}, ZAxis{3.0}};
}

auto TriPyr::move_to_imple(XAxis const) -> void {}

auto TriPyr::move_to_imple(YAxis const) -> void {}

auto TriPyr::move_to_imple(ZAxis const) -> void {}

// Move the shape to a new location
auto TriPyr::move_to_imple(XAxis const x, YAxis const y, ZAxis const z) -> void
{
    shapes_list_ptr_->at(0).pose.position.x = x.get_value();
    shapes_list_ptr_->at(0).pose.position.y = y.get_value();
    shapes_list_ptr_->at(0).pose.position.z = z.get_value();
}

auto TriPyr::move_by_imple(YAxis const) -> void {}

auto TriPyr::move_by_imple(ZAxis const) -> void {}

auto TriPyr::move_by_imple(XAxis const, YAxis const, ZAxis const) -> void {}

// Return marker message for displaying the shape
auto TriPyr::get_display_markers_imple()
    -> std::shared_ptr<std::vector<visualization_msgs::msg::Marker>>
{
    return shapes_list_ptr_;
}

auto TriPyr::rotate_about_axis_to_imple(ZAxis radians) -> void {}
auto TriPyr::get_orientation_imple() const -> ZAxis {return ZAxis{0.0};}
auto TriPyr::move_by_imple(XAxis const) -> void {}
} // namespace shapes