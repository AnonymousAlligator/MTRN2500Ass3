// Copyright 2019 Zhihao Zhang License MIT
// Editted by Kevin Hu

#include "rect_prism.hpp"

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
// Rectangular prism implementation

RectPrism::RectPrism(int id, double posx, double posy, double posz)
    : length_{2.0}
    , breadth_{1.0}
    , height_{5.0}
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
    shape.type = visualization_msgs::msg::Marker::CUBE;
    // Add, modify or delete.
    shape.action = visualization_msgs::msg::Marker::ADD;

    // Position
    shape.pose.position.x = -3;
    shape.pose.position.y = 3;
    shape.pose.position.z = (height_.get_value())/2;

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

    // body.colors.emplace_back();
    using namespace std::chrono_literals;
    shape.lifetime =
        rclcpp::Duration{1s}; // HOw long our marker message is valid for
}

// Basic shape manipulation functions

auto RectPrism::resize_imple(AllAxis const new_size) -> void
{
    length_ = new_size;
}

auto RectPrism::rescale_imple(AnyAxis const factor) -> void
{
    length_ = AllAxis{length_.get_value() * factor.get_value()};
}

auto RectPrism::get_colour_imple() const -> Colour { return Colour::black; }

auto RectPrism::set_parent_frame_name_imple(std::string frame_name) -> void
{
    parent_frame_name_ = std::move(frame_name);
}

auto RectPrism::get_location_imple() const -> std::tuple<XAxis, YAxis, ZAxis>
{
    return std::tuple{XAxis{1.0}, YAxis{2.0}, ZAxis{3.0}};
}

auto RectPrism::move_to_imple(XAxis const) -> void {}

auto RectPrism::move_to_imple(YAxis const) -> void {}

auto RectPrism::move_to_imple(ZAxis const) -> void {}

// Shape movement
auto RectPrism::move_to_imple(XAxis const x, YAxis const y, ZAxis const z)
    -> void
{
    shapes_list_ptr_->at(0).pose.position.x = x.get_value();
    shapes_list_ptr_->at(0).pose.position.y = y.get_value();
    shapes_list_ptr_->at(0).pose.position.z = z.get_value();
}

auto RectPrism::move_by_imple(YAxis const) -> void {}

auto RectPrism::move_by_imple(ZAxis const) -> void {}

auto RectPrism::move_by_imple(XAxis const, YAxis const, ZAxis const) -> void {}

// Return marker message
auto RectPrism::get_display_markers_imple()
    -> std::shared_ptr<std::vector<visualization_msgs::msg::Marker>>
{
    return shapes_list_ptr_;
}
auto RectPrism::rotate_about_axis_to_imple(ZAxis radians) -> void {}
auto RectPrism::get_orientation_imple() const -> ZAxis { return ZAxis{0.0}; }
auto RectPrism::move_by_imple(XAxis const) -> void {}
} // namespace shapes
