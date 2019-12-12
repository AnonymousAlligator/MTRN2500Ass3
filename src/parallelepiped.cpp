// Copyright 2019 Zhihao Zhang License MIT
// Edited by Curtis Ly (z5209698)

#include "parallelepiped.hpp"

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
// Parallelepiped implementation

Parallelepiped::Parallelepiped(int id, double posx, double posy, double posz)
    : length_{2.0}
    , parent_frame_name_{"local_frame"}
    , shapes_list_ptr_{
          std::make_shared<std::vector<visualization_msgs::msg::Marker>>()}
{
    // Get a ref to the vector of marker for ease of use
    auto & shapes_list = *shapes_list_ptr_;
    // create a new marker
    shapes_list.emplace_back();

    // get a ref to the new marker.
    auto & shape = shapes_list[0];
    // Parent frame name
    shape.header.frame_id = helper::world_frame_name("z0000000");
    // body.header.stamp

    // namespace the marker will be in
    shape.ns = "";
    // Used to identify which marker we are adding/modifying/deleting
    // Must be unique between shape objects.
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

    // Make the parallelepiped yellow
    shape.color.r = 1.0;
    shape.color.g = 1.0;
    shape.color.b = 0.0;
    shape.color.a = 1.0;

    // Parallepiped from Geogebra by harry624
    // Bottom Parallelogram (ABEF)
    geometry_msgs::msg::Point p;
    p.x = 0;
    p.y = 0;
    p.z = 0;

    geometry_msgs::msg::Point p2 = p;
    p2.x = 1;
    p2.y = 5;
    p2.z = 3;

    geometry_msgs::msg::Point p3 = p;
    p3.x = 6;
    p3.y = 6;
    p3.z = 6;

    shape.points.push_back(p);
    shape.points.push_back(p2);
    shape.points.push_back(p3);

    p2.x = 5;
    p2.y = 1;
    p2.z = 3;

    shape.points.push_back(p);
    shape.points.push_back(p2);
    shape.points.push_back(p3);

    // Front Parallelogram (ABCD)
    p.x = 0;
    p.y = 0;
    p.z = 0;

    p2.x = 1;
    p2.y = 5;
    p2.z = 3;

    p3.x = 0;
    p3.y = 6;
    p3.z = 7;

    shape.points.push_back(p);
    shape.points.push_back(p2);
    shape.points.push_back(p3);

    p2.x = -1;
    p2.y = 1;
    p2.z = 4;

    shape.points.push_back(p);
    shape.points.push_back(p2);
    shape.points.push_back(p3);

    // Right Side Parallelogram (ADGF)
    p.x = 0;
    p.y = 0;
    p.z = 0;

    p2.x = 5;
    p2.y = 1;
    p2.z = 3;

    p3.x = 4;
    p3.y = 2;
    p3.z = 7;

    shape.points.push_back(p);
    shape.points.push_back(p2);
    shape.points.push_back(p3);

    p2.x = -1;
    p2.y = 1;
    p2.z = 4;

    shape.points.push_back(p);
    shape.points.push_back(p2);
    shape.points.push_back(p3);

    // Left Side Parallelogram (BEHC)
    p.x = 1;
    p.y = 5;
    p.z = 3;

    p2.x = 6;
    p2.y = 6;
    p2.z = 6;

    p3.x = 5;
    p3.y = 7;
    p3.z = 10;

    shape.points.push_back(p);
    shape.points.push_back(p2);
    shape.points.push_back(p3);

    p2.x = 0;
    p2.y = 6;
    p2.z = 7;

    shape.points.push_back(p);
    shape.points.push_back(p2);
    shape.points.push_back(p3);

    // Top Parallelogram (DGHC)
    p.x = -1;
    p.y = 1;
    p.z = 4;

    p2.x = 4;
    p2.y = 2;
    p2.z = 7;

    p3.x = 5;
    p3.y = 7;
    p3.z = 10;

    shape.points.push_back(p);
    shape.points.push_back(p2);
    shape.points.push_back(p3);

    p2.x = 0;
    p2.y = 6;
    p2.z = 7;

    shape.points.push_back(p);
    shape.points.push_back(p2);
    shape.points.push_back(p3);

    // Rear Parallelogram (FEHG)
    p.x = 5;
    p.y = 1;
    p.z = 3;

    p2.x = 6;
    p2.y = 6;
    p2.z = 6;

    p3.x = 5;
    p3.y = 7;
    p3.z = 10;

    shape.points.push_back(p);
    shape.points.push_back(p2);
    shape.points.push_back(p3);

    p2.x = 4;
    p2.y = 2;
    p2.z = 7;

    shape.points.push_back(p);
    shape.points.push_back(p2);
    shape.points.push_back(p3);

    // body.colors.emplace_back();
    using namespace std::chrono_literals;
    shape.lifetime =
        rclcpp::Duration{1s}; // How long our marker message is valid for
}

// Basic shape manipulation functions

auto Parallelepiped::resize_imple(AllAxis const new_size) -> void
{
    length_ = new_size;
}

auto Parallelepiped::rescale_imple(AnyAxis const factor) -> void
{
    length_ = AllAxis{length_.get_value() * factor.get_value()};
}

auto Parallelepiped::get_colour_imple() const -> Colour { return Colour::black; }

auto Parallelepiped::set_parent_frame_name_imple(std::string frame_name) -> void
{
    parent_frame_name_ = std::move(frame_name);
}

auto Parallelepiped::get_location_imple() const -> std::tuple<XAxis, YAxis, ZAxis>
{
    return std::tuple{XAxis{1.0}, YAxis{2.0}, ZAxis{3.0}};
}

auto Parallelepiped::move_to_imple(XAxis const) -> void {}

auto Parallelepiped::move_to_imple(YAxis const) -> void {}

auto Parallelepiped::move_to_imple(ZAxis const) -> void {}

// Shape movement
auto Parallelepiped::move_to_imple(XAxis const x, YAxis const y, ZAxis const z)
    -> void
{
    shapes_list_ptr_->at(0).pose.position.x = x.get_value();
    shapes_list_ptr_->at(0).pose.position.y = y.get_value();
    shapes_list_ptr_->at(0).pose.position.z = z.get_value();
}

auto Parallelepiped::move_by_imple(YAxis const) -> void {}

auto Parallelepiped::move_by_imple(ZAxis const) -> void {}

auto Parallelepiped::move_by_imple(XAxis const, YAxis const, ZAxis const) -> void {}

// Return marker message
auto Parallelepiped::get_display_markers_imple()
    -> std::shared_ptr<std::vector<visualization_msgs::msg::Marker>>
{
    return shapes_list_ptr_;
}
auto Parallelepiped::rotate_about_axis_to_imple(ZAxis radians) -> void {}
auto Parallelepiped::get_orientation_imple() const -> ZAxis { return ZAxis{0.0}; }
auto Parallelepiped::move_by_imple(XAxis const) -> void {}
} // namespace shapes
