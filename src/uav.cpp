// Copyright 2019 Zhihao Zhang License MIT
// Editted by Curtis Ly (z5209698) and Kevin Hu (z5207293)

#include "uav.hpp"
#include "cube.hpp"

#include "rclcpp/rclcpp.hpp" // http://docs.ros2.org/dashing/api/rclcpp/
#include "student_helper.hpp"
#include "single_shape_display.hpp"

#include <chrono>
#include <cstdlib>
#include <cstdio>
#include <cstring>
#include <iostream>
#include <map>
#include <memory>
#include <string>
#include <sstream>
#include <tuple>
#include <utility>
#include <vector>
// make a class for each shape and class of uav which incorporates all the shapes
namespace shapes
{
// UAV implementation

UAV::UAV(int id, double posx, double posy, double posz)
    : length_{2.0}
    , breadth_{1.0}
    , height_{1.0}
    , parent_frame_name_{"local_frame"}
    , shapes_list_ptr_{
          std::make_shared<std::vector<visualization_msgs::msg::Marker>>()}
{
    using namespace std::chrono_literals;

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
    shape.type = visualization_msgs::msg::Marker::CUBE;
    // Add, modify or delete.
    shape.action = visualization_msgs::msg::Marker::ADD;

    // Position
    shape.pose.position.x = 0;
    shape.pose.position.y = 0;
    shape.pose.position.z = 20;

    // Orientation in quaternion
    shape.pose.orientation.x = 0;
    shape.pose.orientation.y = 0;
    shape.pose.orientation.z = 0;
    shape.pose.orientation.w = 1;

    // Scale change the dimension of the sides
    shape.scale.x = length_.get_value();
    shape.scale.y = breadth_.get_value();
    shape.scale.z = height_.get_value();

    // Colour red, green, blue, alpha (transparency)
    shape.color.r = 1.0;
    shape.color.g = 0.0;
    shape.color.b = 0.0;
    shape.color.a = 1.0;

    shapes::Cube(16, posx, posy, posz);

    /*
    // Create and display a cube
    auto my_cube = std::make_shared<shapes::Cube>(30, posx, posy, posz);
    auto my_cube_display =
        std::make_shared<display::SingleShapeDisplay>("cube", 100ms);
    my_cube_display->display_object(my_cube);
    ros_worker.add_node(my_cube_display);
    */

    // body.colors.emplace_back();
    using namespace std::chrono_literals;
    shape.lifetime =
        rclcpp::Duration{1s}; // How long our marker message is valid for
}

// Basic shape manipulation functions

auto UAV::resize_imple(AllAxis const new_size) -> void
{
    length_ = new_size;
}

auto UAV::rescale_imple(AnyAxis const factor) -> void
{
    length_ = AllAxis{length_.get_value() * factor.get_value()};
}

auto UAV::get_colour_imple() const -> Colour { return Colour::black; }

auto UAV::set_parent_frame_name_imple(std::string frame_name) -> void
{
    parent_frame_name_ = std::move(frame_name);
}

auto UAV::get_location_imple() const -> std::tuple<XAxis, YAxis, ZAxis>
{
    return std::tuple{XAxis{1.0}, YAxis{2.0}, ZAxis{3.0}};
}

auto UAV::move_to_imple(XAxis const) -> void {}

auto UAV::move_to_imple(YAxis const) -> void {}

auto UAV::move_to_imple(ZAxis const) -> void {}

// Shape movement
auto UAV::move_to_imple(XAxis const x, YAxis const y, ZAxis const z)
    -> void
{
    shapes_list_ptr_->at(0).pose.position.x = x.get_value();
    shapes_list_ptr_->at(0).pose.position.y = y.get_value();
    shapes_list_ptr_->at(0).pose.position.z = z.get_value();
}

auto UAV::move_by_imple(YAxis const) -> void {}

auto UAV::move_by_imple(ZAxis const) -> void {}

auto UAV::move_by_imple(XAxis const, YAxis const, ZAxis const) -> void {}

// Return marker message
auto UAV::get_display_markers_imple()
    -> std::shared_ptr<std::vector<visualization_msgs::msg::Marker>>
{
    return shapes_list_ptr_;
}
auto UAV::rotate_about_axis_to_imple(ZAxis radians) -> void {}
auto UAV::get_orientation_imple() const -> ZAxis { return ZAxis{0.0}; }
auto UAV::move_by_imple(XAxis const) -> void {}
} // namespace shapes
