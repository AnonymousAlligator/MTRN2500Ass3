// Copyright 2019 Zhihao Zhang License MIT
// Edited by Kevin Hu

#include "oct_pyr.hpp"

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
// Implementation of the octagonal pyramid class
OctPyr::OctPyr(int id, double posx, double posy, double posz)
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
    shape.scale.x = 3.0;
    shape.scale.y = 3.0;
    shape.scale.z = 3.0;

    // Set colour
    shape.color.r = 1.0;
    shape.color.g = 0.0;
    shape.color.b = 0.0;
    shape.color.a = 1.0;

// Triangle 1 top 12 oclock
    geometry_msgs::msg::Point p;
    p.x = 6.5;
    p.y = 1.2;
    p.z = 2.5;

    geometry_msgs::msg::Point p2 = p;
    p2.x = 7.5;
    p2.y = 1.2;
    p2.z = 2.5;

    geometry_msgs::msg::Point p3 = p;
    p3.x = 7.0;
    p3.y = 0.0;
    p3.z = 2.5;

    shape.points.push_back(p);
    shape.points.push_back(p2);
    shape.points.push_back(p3);

    // Triangle 2 top 1-2 oclock
    p.x = 7.5;
    p.y = 1.2;
    p.z = 2.5;

    p2.x = 8.2;
    p2.y = 0.5;
    p2.z = 2.5;

    p3.x = 7.0;
    p3.y = 0.0;
    p3.z = 2.5;

    shape.points.push_back(p);
    shape.points.push_back(p2);
    shape.points.push_back(p3);

    // Triangle 3 top 3 oclock
    p.x = 8.2;
    p.y = 0.5;
    p.z = 2.5;

    p2.x = 8.2;
    p2.y = -0.5;
    p2.z = 2.5;

    p3.x = 7.0;
    p3.y = 0.0;
    p3.z = 2.5;

    shape.points.push_back(p);
    shape.points.push_back(p2);
    shape.points.push_back(p3);

    // Triangle 4 top 4-5 oclock
    p.x = 8.2;
    p.y = -0.5;
    p.z = 2.5;

    p2.x = 7.5;
    p2.y = -1.2;
    p2.z = 2.5;

    p3.x = 7.0;
    p3.y = 0.0;
    p3.z = 2.5;

    shape.points.push_back(p);
    shape.points.push_back(p2);
    shape.points.push_back(p3);

    // Triangle 5 top 6 oclock
    p.x = 7.5;
    p.y = -1.2;
    p.z = 2.5;

    p2.x = 6.5;
    p2.y = -1.2;
    p2.z = 2.5;

    p3.x = 7.0;
    p3.y = 0.0;
    p3.z = 2.5;

    shape.points.push_back(p);
    shape.points.push_back(p2);
    shape.points.push_back(p3);

    // Triangle 6 top 7-8 oclock
    p.x = 6.5;
    p.y = -1.2;
    p.z = 2.5;

    p2.x = 5.8;
    p2.y = -0.5;
    p2.z = 2.5;

    p3.x = 7.0;
    p3.y = 0.0;
    p3.z = 2.5;

    shape.points.push_back(p);
    shape.points.push_back(p2);
    shape.points.push_back(p3);

    // Triangle 7 top 9 oclock 
    p.x = 5.8;
    p.y = -0.5;
    p.z = 2.5;

    p2.x = 5.8;
    p2.y = 0.5;
    p2.z = 2.5;

    p3.x = 7.0;
    p3.y = 0.0;
    p3.z = 2.5;

    shape.points.push_back(p);
    shape.points.push_back(p2);
    shape.points.push_back(p3);

    // Triangle 8 top 10-11 oclock
    p.x = 5.8;
    p.y = 0.5;
    p.z = 2.5;

    p2.x = 6.5;
    p2.y = 1.2;
    p2.z = 2.5;

    p3.x = 7.0;
    p3.y = 0.0;
    p3.z = 2.5;

    shape.points.push_back(p);
    shape.points.push_back(p2);
    shape.points.push_back(p3);

    // Triangle 9 side 12 oclock
    p.x = 6.5;
    p.y = 1.2;
    p.z = 2.5;

    p2.x = 7.5;
    p2.y = 1.2;
    p2.z = 2.5;

    p3.x = 7.0;
    p3.y = 0.0;
    p3.z = 6.5;

    shape.points.push_back(p);
    shape.points.push_back(p2);
    shape.points.push_back(p3);

    // Triangle 10 side 1-2 oclock
    p.x = 7.5;
    p.y = 1.2;
    p.z = 2.5;

    p2.x = 8.2;
    p2.y = 0.5;
    p2.z = 2.5;

    p3.x = 7.0;
    p3.y = 0.0;
    p3.z = 6.5;

    shape.points.push_back(p);
    shape.points.push_back(p2);
    shape.points.push_back(p3);

    // Triangle 11 side 3 oclock
    p.x = 8.2;
    p.y = 0.5;
    p.z = 2.5;

    p2.x = 8.2;
    p2.y = -0.5;
    p2.z = 2.5;

    p3.x = 7.0;
    p3.y = 0.0;
    p3.z = 6.5;

    shape.points.push_back(p);
    shape.points.push_back(p2);
    shape.points.push_back(p3);

    // Triangle 12 side 4-5 oclock
    p.x = 8.2;
    p.y = -0.5;
    p.z = 2.5;

    p2.x = 7.5;
    p2.y = -1.2;
    p2.z = 2.5;

    p3.x = 7.0;
    p3.y = 0.0;
    p3.z = 6.5;

    shape.points.push_back(p);
    shape.points.push_back(p2);
    shape.points.push_back(p3);

    // Triangle 13 side 6 oclock
    p.x = 7.5;
    p.y = -1.2;
    p.z = 2.5;

    p2.x = 6.5;
    p2.y = -1.2;
    p2.z = 2.5;

    p3.x = 7.0;
    p3.y = 0.0;
    p3.z = 6.5;

    shape.points.push_back(p);
    shape.points.push_back(p2);
    shape.points.push_back(p3);

    // Triangle 14 side 7-8 oclock
    p.x = 6.5;
    p.y = -1.2;
    p.z = 2.5;

    p2.x = 5.8;
    p2.y = -0.5;
    p2.z = 2.5;

    p3.x = 7.0;
    p3.y = 0.0;
    p3.z = 6.5;

    shape.points.push_back(p);
    shape.points.push_back(p2);
    shape.points.push_back(p3);

    // Triangle 15 side 9 oclock 
    p.x = 5.8;
    p.y = -0.5;
    p.z = 2.5;

    p2.x = 5.8;
    p2.y = 0.5;
    p2.z = 2.5;

    p3.x = 7.0;
    p3.y = 0.0;
    p3.z = 6.5;

    shape.points.push_back(p);
    shape.points.push_back(p2);
    shape.points.push_back(p3);

    // Triangle 16 side 10-11 oclock
    p.x = 5.8;
    p.y = 0.5;
    p.z = 2.5;

    p2.x = 6.5;
    p2.y = 1.2;
    p2.z = 2.5;

    p3.x = 7.0;
    p3.y = 0.0;
    p3.z = 6.5;

    shape.points.push_back(p);
    shape.points.push_back(p2);
    shape.points.push_back(p3);

    // body.colors.emplace_back();
    using namespace std::chrono_literals;
    shape.lifetime =
        rclcpp::Duration{1s}; // How long our marker message is valid for
}

// Shape manipulation functions
auto OctPyr::resize_imple(AllAxis const new_size) -> void {length_ = new_size;}

auto OctPyr::rescale_imple(AnyAxis const factor) -> void
{
    length_ = AllAxis{length_.get_value() * factor.get_value()};
}

auto OctPyr::get_colour_imple() const -> Colour {return Colour::black;}

auto OctPyr::set_parent_frame_name_imple(std::string frame_name) -> void
{
    parent_frame_name_ = std::move(frame_name);
}

auto OctPyr::get_location_imple() const -> std::tuple<XAxis, YAxis, ZAxis>
{
    return std::tuple{XAxis{1.0}, YAxis{2.0}, ZAxis{3.0}};
}

auto OctPyr::move_to_imple(XAxis const) -> void {}

auto OctPyr::move_to_imple(YAxis const) -> void {}

auto OctPyr::move_to_imple(ZAxis const) -> void {}

// Move the shape to a new location
auto OctPyr::move_to_imple(XAxis const x, YAxis const y, ZAxis const z) -> void
{
    shapes_list_ptr_->at(0).pose.position.x = x.get_value();
    shapes_list_ptr_->at(0).pose.position.y = y.get_value();
    shapes_list_ptr_->at(0).pose.position.z = z.get_value();
}

auto OctPyr::move_by_imple(YAxis const) -> void {}

auto OctPyr::move_by_imple(ZAxis const) -> void {}

auto OctPyr::move_by_imple(XAxis const, YAxis const, ZAxis const) -> void {}

// Return marker message for displaying the shape
auto OctPyr::get_display_markers_imple()
    -> std::shared_ptr<std::vector<visualization_msgs::msg::Marker>>
{
    return shapes_list_ptr_;
}

auto OctPyr::rotate_about_axis_to_imple(ZAxis radians) -> void {}
auto OctPyr::get_orientation_imple() const -> ZAxis {return ZAxis{0.0};}
auto OctPyr::move_by_imple(XAxis const) -> void {}
} // namespace shapes