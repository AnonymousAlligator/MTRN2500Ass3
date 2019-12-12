// Copyright 2019 Zhihao Zhang License MIT
// Edited by Curtis Ly (z5209698)

#include "cube.hpp"

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
// Implementation of the cube class
Cube::Cube(int id, double posx, double posy, double posz)
    : length_{3.0}
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
    shape.type = visualization_msgs::msg::Marker::CUBE;
    // Add, modify or delete.
    shape.action = visualization_msgs::msg::Marker::ADD;

    // Position
    shape.pose.position.x = posx;
    shape.pose.position.y = posy;
    shape.pose.position.z = posz;

    // Orientation in quaternion
    shape.pose.orientation.x = 0;
    shape.pose.orientation.y = 0;
    shape.pose.orientation.z = 0;
    shape.pose.orientation.w = 1;

    // Scale change the dimension of the sides
    shape.scale.x = 1;
    shape.scale.y = 1;
    shape.scale.z = 1;

    // Make the cube brick-red coloured
    shape.color.r = 1.0;
    shape.color.g = 0.6;
    shape.color.b = 0.6;
    shape.color.a = 1.0;

    // body.colors.emplace_back();
    using namespace std::chrono_literals;
    shape.lifetime =
        rclcpp::Duration{1s}; // How long our marker message is valid for
}

// Shape manipulation functions
auto Cube::resize_imple(AllAxis const new_size) -> void {length_ = new_size;}

auto Cube::rescale_imple(AnyAxis const factor) -> void
{
    length_ = AllAxis{length_.get_value() * factor.get_value()};
}

auto Cube::get_colour_imple() const -> Colour {return Colour::black;}

auto Cube::set_parent_frame_name_imple(std::string frame_name) -> void
{
    parent_frame_name_ = std::move(frame_name);
}

auto Cube::get_location_imple() const -> std::tuple<XAxis, YAxis, ZAxis>
{
    return std::tuple{XAxis{1.0}, YAxis{2.0}, ZAxis{3.0}};
}

auto Cube::move_to_imple(XAxis const) -> void {}

auto Cube::move_to_imple(YAxis const) -> void {}

auto Cube::move_to_imple(ZAxis const) -> void {}

// Move the shape to a new location
auto Cube::move_to_imple(XAxis const x, YAxis const y, ZAxis const z) -> void
{
    shapes_list_ptr_->at(0).pose.position.x = x.get_value();
    shapes_list_ptr_->at(0).pose.position.y = y.get_value();
    shapes_list_ptr_->at(0).pose.position.z = z.get_value();
}

auto Cube::move_by_imple(YAxis const) -> void {}

auto Cube::move_by_imple(ZAxis const) -> void {}

auto Cube::move_by_imple(XAxis const, YAxis const, ZAxis const) -> void {}

// Return marker message for displaying the shape
auto Cube::get_display_markers_imple()
    -> std::shared_ptr<std::vector<visualization_msgs::msg::Marker>>
{
    return shapes_list_ptr_;
}

auto Cube::rotate_about_axis_to_imple(ZAxis radians) -> void {}
auto Cube::get_orientation_imple() const -> ZAxis {return ZAxis{0.0};}
auto Cube::move_by_imple(XAxis const) -> void {}
} // namespace shapes