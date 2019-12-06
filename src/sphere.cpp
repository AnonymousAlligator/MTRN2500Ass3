// Copyright 2019 Zhihao Zhang License MIT
#include "sphere.hpp"

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
/**
 * \brief Example of how sphere class may be implemented, the design may not be
 *the most suitable. Trivial functionalities are not implemented.
 **/

Sphere::Sphere(int id)
    : radius_{5.0}
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
    shape.type = visualization_msgs::msg::Marker::SPHERE;
    // Add, modify or delete.
    shape.action = visualization_msgs::msg::Marker::ADD;

    // Position
    shape.pose.position.x = 1;
    shape.pose.position.y = 1;
    shape.pose.position.z = 2.5;

    // Orientation in quaternion. Check transform marker in assignment 2
    // for how to manipulate it.
    shape.pose.orientation.x = 0;
    shape.pose.orientation.y = 0;
    shape.pose.orientation.z = 0;
    shape.pose.orientation.w = 1;

    // Scale change the dimension of the sides.
    shape.scale.x = 3;
    shape.scale.y = 3;
    shape.scale.z = 3;

    // colour red, green, blue, alpha (transparency)
    shape.color.r = 0.0;
    shape.color.g = 1.0;
    shape.color.b = 0.0;
    shape.color.a = 1.0;

    // body.colors.emplace_back();
    using namespace std::chrono_literals;
    shape.lifetime =
        rclcpp::Duration{1s}; // HOw long our marker message is valid for
}

auto Sphere::resize_imple(AllAxis const new_size) -> void
{
    radius_ = new_size;
}

auto Sphere::rescale_imple(AnyAxis const factor) -> void
{
    radius_ = AllAxis{radius_.get_value() * factor.get_value()};
}

auto Sphere::get_colour_imple() const -> Colour { return Colour::black; }

auto Sphere::set_parent_frame_name_imple(std::string frame_name) -> void
{
    parent_frame_name_ = std::move(frame_name);
}

auto Sphere::get_location_imple() const -> std::tuple<XAxis, YAxis, ZAxis>
{
    return std::tuple{XAxis{1.0}, YAxis{2.0}, ZAxis{3.0}};
}

auto Sphere::move_to_imple(XAxis const) -> void {}

auto Sphere::move_to_imple(YAxis const) -> void {}

auto Sphere::move_to_imple(ZAxis const) -> void {}

/**
 * \brief Move the shape to a new location.
 * \param x new x location
 * \param y new y location
 * \param z new z location
 */
auto Sphere::move_to_imple(XAxis const x, YAxis const y, ZAxis const z) -> void
{
    shapes_list_ptr_->at(0).pose.position.x = x.get_value();
    shapes_list_ptr_->at(0).pose.position.y = y.get_value();
    shapes_list_ptr_->at(0).pose.position.z = z.get_value();
}

auto Sphere::move_by_imple(YAxis const) -> void {}

auto Sphere::move_by_imple(ZAxis const) -> void {}

auto Sphere::move_by_imple(XAxis const, YAxis const, ZAxis const) -> void {}

/**
 * \brief Return marker message for displaying the shape
 * \return shape marker message
 */
auto Sphere::get_display_markers_imple()
    -> std::shared_ptr<std::vector<visualization_msgs::msg::Marker>>
{
    return shapes_list_ptr_;
}

auto Sphere::rotate_about_axis_to_imple(ZAxis radians) -> void {}

auto Sphere::get_orientation_imple() const -> ZAxis { return ZAxis{0.0}; }

auto Sphere::move_by_imple(XAxis const) -> void {}

} // namespace shapes
