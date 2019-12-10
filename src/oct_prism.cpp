// Copyright 2019 Zhihao Zhang License MIT
// Edited by Kevin Hu

#include "oct_prism.hpp"

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
// Implementation of the octangular prism class
OctPrism::OctPrism(int id, double posx, double posy, double posz)
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
    shape.pose.position.x = posx;
    shape.pose.position.y = posy;
    shape.pose.position.z = posz;

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

    // Triangle 1 base 12 oclock
    geometry_msgs::msg::Point p;
    p.x = 6.5;
    p.y = 1.2;
    p.z = 0.5;

    geometry_msgs::msg::Point p2 = p;
    p2.x = 7.5;
    p2.y = 1.2;
    p2.z = 0.5;

    geometry_msgs::msg::Point p3 = p;
    p3.x = 7.0;
    p3.y = 0.0;
    p3.z = 0.5;

    shape.points.push_back(p);
    shape.points.push_back(p2);
    shape.points.push_back(p3);

    // Triangle 2 base 1-2 oclock
    p.x = 7.5;
    p.y = 1.2;
    p.z = 0.5;

    p2.x = 8.2;
    p2.y = 0.5;
    p2.z = 0.5;

    p3.x = 7.0;
    p3.y = 0.0;
    p3.z = 0.5;

    shape.points.push_back(p);
    shape.points.push_back(p2);
    shape.points.push_back(p3);

    // Triangle 3 base 3 oclock
    p.x = 8.2;
    p.y = 0.5;
    p.z = 0.5;

    p2.x = 8.2;
    p2.y = -0.5;
    p2.z = 0.5;

    p3.x = 7.0;
    p3.y = 0.0;
    p3.z = 0.5;

    shape.points.push_back(p);
    shape.points.push_back(p2);
    shape.points.push_back(p3);

    // Triangle 4 base 4-5 oclock
    p.x = 8.2;
    p.y = -0.5;
    p.z = 0.5;

    p2.x = 7.5;
    p2.y = -1.2;
    p2.z = 0.5;

    p3.x = 7.0;
    p3.y = 0.0;
    p3.z = 0.5;

    shape.points.push_back(p);
    shape.points.push_back(p2);
    shape.points.push_back(p3);

    // Triangle 5 base 6 oclock
    p.x = 7.5;
    p.y = -1.2;
    p.z = 0.5;

    p2.x = 6.5;
    p2.y = -1.2;
    p2.z = 0.5;

    p3.x = 7.0;
    p3.y = 0.0;
    p3.z = 0.5;

    shape.points.push_back(p);
    shape.points.push_back(p2);
    shape.points.push_back(p3);

    // Triangle 6 base 7-8 oclock
    p.x = 6.5;
    p.y = -1.2;
    p.z = 0.5;

    p2.x = 5.8;
    p2.y = -0.5;
    p2.z = 0.5;

    p3.x = 7.0;
    p3.y = 0.0;
    p3.z = 0.5;

    shape.points.push_back(p);
    shape.points.push_back(p2);
    shape.points.push_back(p3);

    // Triangle 7 base 9 oclock 
    p.x = 5.8;
    p.y = -0.5;
    p.z = 0.5;

    p2.x = 5.8;
    p2.y = 0.5;
    p2.z = 0.5;

    p3.x = 7.0;
    p3.y = 0.0;
    p3.z = 0.5;

    shape.points.push_back(p);
    shape.points.push_back(p2);
    shape.points.push_back(p3);

    // Triangle 8 base 10-11 oclock
    p.x = 5.8;
    p.y = 0.5;
    p.z = 0.5;

    p2.x = 6.5;
    p2.y = 1.2;
    p2.z = 0.5;

    p3.x = 7.0;
    p3.y = 0.0;
    p3.z = 0.5;

    shape.points.push_back(p);
    shape.points.push_back(p2);
    shape.points.push_back(p3);

    // Triangle 9 top 12 oclock
    p.x = 6.5;
    p.y = 1.2;
    p.z = 2.5;

    p2.x = 7.5;
    p2.y = 1.2;
    p2.z = 2.5;

    p3.x = 7.0;
    p3.y = 0.0;
    p3.z = 2.5;

    shape.points.push_back(p);
    shape.points.push_back(p2);
    shape.points.push_back(p3);

    // Triangle 10 top 1-2 oclock
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

    // Triangle 11 top 3 oclock
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

    // Triangle 12 top 4-5 oclock
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

    // Triangle 13 top 6 oclock
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

    // Triangle 14 top 7-8 oclock
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

    // Triangle 15 top 9 oclock 
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

    // Triangle 16 top 10-11 oclock
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

    // Triangle 17 side 12 oclock lower
    p.x = 6.5;
    p.y = 1.2;
    p.z = 0.5;

    p2.x = 7.5;
    p2.y = 1.2;
    p2.z = 0.5;

    p3.x = 6.5;
    p3.y = 1.2;
    p3.z = 2.5;

    shape.points.push_back(p);
    shape.points.push_back(p2);
    shape.points.push_back(p3);

    // Triangle 18 side 12 oclock upper
    p.x = 6.5;
    p.y = 1.2;
    p.z = 2.5;

    p2.x = 7.5;
    p2.y = 1.2;
    p2.z = 2.5;

    p3.x = 7.5;
    p3.y = 1.2;
    p3.z = 0.5;

    shape.points.push_back(p);
    shape.points.push_back(p2);
    shape.points.push_back(p3);

    // Triangle 19 side 1-2 oclock lower
    p.x = 7.5;
    p.y = 1.2;
    p.z = 0.5;

    p2.x = 8.2;
    p2.y = 0.5;
    p2.z = 0.5;

    p3.x = 7.5;
    p3.y = 1.2;
    p3.z = 2.5;

    shape.points.push_back(p);
    shape.points.push_back(p2);
    shape.points.push_back(p3);

    // Triangle 20 side 1-2 oclock upper
    p.x = 7.5;
    p.y = 1.2;
    p.z = 2.5;

    p2.x = 8.2;
    p2.y = 0.5;
    p2.z = 2.5;

    p3.x = 8.2;
    p3.y = 0.5;
    p3.z = 0.5;

    shape.points.push_back(p);
    shape.points.push_back(p2);
    shape.points.push_back(p3);

    // Triangle 21 side 3 oclock lower
    p.x = 8.2;
    p.y = 0.5;
    p.z = 0.5;

    p2.x = 8.2;
    p2.y = -0.5;
    p2.z = 0.5;

    p3.x = 8.2;
    p3.y = 0.5;
    p3.z = 2.5;

    shape.points.push_back(p);
    shape.points.push_back(p2);
    shape.points.push_back(p3);

    // Triangle 22 side 3 oclock upper
    p.x = 8.2;
    p.y = 0.5;
    p.z = 2.5;

    p2.x = 8.2;
    p2.y = -0.5;
    p2.z = 2.5;

    p3.x = 8.2;
    p3.y = -0.5;
    p3.z = 0.5;

    shape.points.push_back(p);
    shape.points.push_back(p2);
    shape.points.push_back(p3);

    // Triangle 23 side 4-5 oclock lower
    p.x = 8.2;
    p.y = -0.5;
    p.z = 0.5;

    p2.x = 7.5;
    p2.y = -1.2;
    p2.z = 0.5;

    p3.x = 8.2;
    p3.y = -0.5;
    p3.z = 2.5;

    shape.points.push_back(p);
    shape.points.push_back(p2);
    shape.points.push_back(p3);

    // Triangle 24 side 4-5 oclock upper
    p.x = 8.2;
    p.y = -0.5;
    p.z = 2.5;

    p2.x = 7.5;
    p2.y = -1.2;
    p2.z = 2.5;

    p3.x = 7.5;
    p3.y = -1.2;
    p3.z = 0.5;

    shape.points.push_back(p);
    shape.points.push_back(p2);
    shape.points.push_back(p3);

    // Triangle 25 side 6 oclock lower
    p.x = 7.5;
    p.y = -1.2;
    p.z = 0.5;

    p2.x = 6.5;
    p2.y = -1.2;
    p2.z = 0.5;

    p3.x = 7.5;
    p3.y = -1.2;
    p3.z = 2.5;

    shape.points.push_back(p);
    shape.points.push_back(p2);
    shape.points.push_back(p3);

    // Triangle 26 side 6 oclock upper
    p.x = 7.5;
    p.y = -1.2;
    p.z = 2.5;

    p2.x = 6.5;
    p2.y = -1.2;
    p2.z = 2.5;

    p3.x = 6.5;
    p3.y = -1.2;
    p3.z = 0.5;

    shape.points.push_back(p);
    shape.points.push_back(p2);
    shape.points.push_back(p3);

    // Triangle 27 side 7-8 oclock lower
    p.x = 6.5;
    p.y = -1.2;
    p.z = 0.5;

    p2.x = 5.8;
    p2.y = -0.5;
    p2.z = 0.5;

    p3.x = 6.5;
    p3.y = -1.2;
    p3.z = 2.5;

    shape.points.push_back(p);
    shape.points.push_back(p2);
    shape.points.push_back(p3);

    // Triangle 28 side 7-8 oclock upper
    p.x = 6.5;
    p.y = -1.2;
    p.z = 2.5;

    p2.x = 5.8;
    p2.y = -0.5;
    p2.z = 2.5;

    p3.x = 5.8;
    p3.y = -0.5;
    p3.z = 0.5;

    shape.points.push_back(p);
    shape.points.push_back(p2);
    shape.points.push_back(p3);

    // Triangle 29 side 9 oclock lower
    p.x = 5.8;
    p.y = -0.5;
    p.z = 0.5;

    p2.x = 5.8;
    p2.y = 0.5;
    p2.z = 0.5;

    p3.x = 5.8;
    p3.y = -0.5;
    p3.z = 2.5;

    shape.points.push_back(p);
    shape.points.push_back(p2);
    shape.points.push_back(p3);

    // Triangle 30 9 oclock upper
    p.x = 5.8;
    p.y = -0.5;
    p.z = 2.5;

    p2.x = 5.8;
    p2.y = 0.5;
    p2.z = 2.5;

    p3.x = 5.8;
    p3.y = 0.5;
    p3.z = 0.5;


    shape.points.push_back(p);
    shape.points.push_back(p2);
    shape.points.push_back(p3);

    // Triangle 31 10-11 oclock lower 
    p.x = 5.8;
    p.y = 0.5;
    p.z = 0.5;

    p2.x = 6.5;
    p2.y = 1.2;
    p2.z = 0.5;

    p3.x = 5.8;
    p3.y = 0.5;
    p3.z = 2.5;

    shape.points.push_back(p);
    shape.points.push_back(p2);
    shape.points.push_back(p3);

    // Triangle 32 10-11 oclock upper
    p.x = 5.8;
    p.y = 0.5;
    p.z = 2.5;

    p2.x = 6.5;
    p2.y = 1.2;
    p2.z = 2.5;

    p3.x = 6.5;
    p3.y = 1.2;
    p3.z = 0.5;


    shape.points.push_back(p);
    shape.points.push_back(p2);
    shape.points.push_back(p3);

    // body.colors.emplace_back();
    using namespace std::chrono_literals;
    shape.lifetime =
        rclcpp::Duration{1s}; // How long our marker message is valid for
}

// Shape manipulation functions
auto OctPrism::resize_imple(AllAxis const new_size) -> void {length_ = new_size;}

auto OctPrism::rescale_imple(AnyAxis const factor) -> void
{
    length_ = AllAxis{length_.get_value() * factor.get_value()};
}

auto OctPrism::get_colour_imple() const -> Colour {return Colour::black;}

auto OctPrism::set_parent_frame_name_imple(std::string frame_name) -> void
{
    parent_frame_name_ = std::move(frame_name);
}

auto OctPrism::get_location_imple() const -> std::tuple<XAxis, YAxis, ZAxis>
{
    return std::tuple{XAxis{1.0}, YAxis{2.0}, ZAxis{3.0}};
}

auto OctPrism::move_to_imple(XAxis const) -> void {}

auto OctPrism::move_to_imple(YAxis const) -> void {}

auto OctPrism::move_to_imple(ZAxis const) -> void {}

// Move the shape to a new location
auto OctPrism::move_to_imple(XAxis const x, YAxis const y, ZAxis const z) -> void
{
    shapes_list_ptr_->at(0).pose.position.x = x.get_value();
    shapes_list_ptr_->at(0).pose.position.y = y.get_value();
    shapes_list_ptr_->at(0).pose.position.z = z.get_value();
}

auto OctPrism::move_by_imple(YAxis const) -> void {}

auto OctPrism::move_by_imple(ZAxis const) -> void {}

auto OctPrism::move_by_imple(XAxis const, YAxis const, ZAxis const) -> void {}

// Return marker message for displaying the shape
auto OctPrism::get_display_markers_imple()
    -> std::shared_ptr<std::vector<visualization_msgs::msg::Marker>>
{
    return shapes_list_ptr_;
}

auto OctPrism::rotate_about_axis_to_imple(ZAxis radians) -> void {}
auto OctPrism::get_orientation_imple() const -> ZAxis {return ZAxis{0.0};}
auto OctPrism::move_by_imple(XAxis const) -> void {}
} // namespace shapes