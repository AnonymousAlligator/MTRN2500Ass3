// Copyright 2019 Zhihao Zhang License MIT
// Edited by Kevin Hu

#include "sqr_pyr.hpp"

#include "rclcpp/rclcpp.hpp" // http://docs.ros2.org/dashing/api/rclcpp/
#include "student_helper.hpp"

#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

namespace shapes
{
// Implementation of the square pyramid class
SqrPyr::SqrPyr(int id, double posx, double posy, double posz, double a)
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
    shape.pose.position.x = 0;
    shape.pose.position.y = 0;
    shape.pose.position.z = 0;

    // Orientation in quaternion
    shape.pose.orientation.x = 0;
    shape.pose.orientation.y = 0;
    shape.pose.orientation.z = 0;
    shape.pose.orientation.w = 1;

    // Scale change the dimension of the sides
    shape.scale.x = length_.get_value();
    shape.scale.y = length_.get_value();
    shape.scale.z = length_.get_value();

    // Set colour
    shape.color.r = 1.0;
    shape.color.g = 0.0;
    shape.color.b = 0.0;
    shape.color.a = a;

    // Triangle 1 base upper
    geometry_msgs::msg::Point p;
    p.x = -6;
    p.y = 9;
    p.z = 0;

    geometry_msgs::msg::Point p2 = p;
    p2.x = -2;
    p2.y = 9;
    p2.z = 0;

    geometry_msgs::msg::Point p3 = p;
    p3.x = -2;
    p3.y = 5;
    p3.z = 0;

    shape.points.push_back(p);
    shape.points.push_back(p2);
    shape.points.push_back(p3);

    // Triangle 2 base lower
    p.x = -6;
    p.y = 9;
    p.z = 0;

    p2.x = -6;
    p2.y = 5;
    p2.z = 0;

    p3.x = -2;
    p3.y = 5;
    p3.z = 0;

    shape.points.push_back(p);
    shape.points.push_back(p2);
    shape.points.push_back(p3);

    // Triangle 3 front
    p.x = -6;
    p.y = 5;
    p.z = 0;

    p2.x = -2;
    p2.y = 5;
    p2.z = 0;

    p3.x = -4;
    p3.y = 7;
    p3.z = sqrt(8);

    shape.points.push_back(p);
    shape.points.push_back(p2);
    shape.points.push_back(p3);

    // Triangle 4 back
    p.x = -6;
    p.y = 9;
    p.z = 0;

    p2.x = -2;
    p2.y = 9;
    p2.z = 0;

    p3.x = -4;
    p3.y = 7;
    p3.z = sqrt(8);

    shape.points.push_back(p);
    shape.points.push_back(p2);
    shape.points.push_back(p3);

    // Triangle 5 left
    p.x = -6;
    p.y = 9;
    p.z = 0;

    p2.x = -6;
    p2.y = 5;
    p2.z = 0;

    p3.x = -4;
    p3.y = 7;
    p3.z = sqrt(8);

    shape.points.push_back(p);
    shape.points.push_back(p2);
    shape.points.push_back(p3);

    // Triangle 6 right
    p.x = -2;
    p.y = 9;
    p.z = 0;

    p2.x = -2;
    p2.y = 5;
    p2.z = 0;

    p3.x = -4;
    p3.y = 7;
    p3.z = sqrt(8);

    shape.points.push_back(p);
    shape.points.push_back(p2);
    shape.points.push_back(p3);

    // body.colors.emplace_back();
    using namespace std::chrono_literals;
    shape.lifetime =
        rclcpp::Duration{1s}; // How long our marker message is valid for
}

// Shape manipulation functions
auto SqrPyr::resize_imple(AllAxis const new_size) -> void {length_ = new_size;}

auto SqrPyr::rescale_imple(AnyAxis const factor) -> void
{
    length_ = AllAxis{length_.get_value() * factor.get_value()};
}

auto SqrPyr::get_colour_imple() const -> Colour {return Colour::black;}

auto SqrPyr::set_parent_frame_name_imple(std::string frame_name) -> void
{
    parent_frame_name_ = std::move(frame_name);
}

auto SqrPyr::get_location_imple() const -> std::tuple<XAxis, YAxis, ZAxis>
{
    return std::tuple{XAxis{1.0}, YAxis{2.0}, ZAxis{3.0}};
}

auto SqrPyr::move_to_imple(XAxis const) -> void {}

auto SqrPyr::move_to_imple(YAxis const) -> void {}

auto SqrPyr::move_to_imple(ZAxis const) -> void {}

// Move the shape to a new location
auto SqrPyr::move_to_imple(XAxis const x, YAxis const y, ZAxis const z) -> void
{
    shapes_list_ptr_->at(0).pose.position.x = x.get_value();
    shapes_list_ptr_->at(0).pose.position.y = y.get_value();
    shapes_list_ptr_->at(0).pose.position.z = z.get_value();
}

auto SqrPyr::move_by_imple(YAxis const) -> void {}

auto SqrPyr::move_by_imple(ZAxis const) -> void {}

auto SqrPyr::move_by_imple(XAxis const, YAxis const, ZAxis const) -> void {}

// Return marker message for displaying the shape
auto SqrPyr::get_display_markers_imple()
    -> std::shared_ptr<std::vector<visualization_msgs::msg::Marker>>
{
    return shapes_list_ptr_;
}

auto SqrPyr::rotate_about_axis_to_imple(ZAxis radians) -> void {}
auto SqrPyr::get_orientation_imple() const -> ZAxis {return ZAxis{0.0};}
auto SqrPyr::move_by_imple(XAxis const) -> void {}
} // namespace shapes