// Created by Curtis Ly (z5209698)

#ifndef UAV_HPP_
#define UAV_HPP_

#include "interfaces.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "single_shape_display.hpp"
#include "cube.hpp"

#include <cstdlib>
#include <memory>
#include <string>
#include <tuple>
#include <vector>

namespace shapes
{
// ReSharper disable once CppClassCanBeFinal
class UAV : public ShapeCommonInterface
{
public:
    explicit UAV(int id, double posx, double posy, double posz);
    // std::shared_ptr<display::SingleShapeDisplay> get_cube_display();

protected:
    AllAxis length_, breadth_, height_;
    std::string parent_frame_name_;
    std::shared_ptr<std::vector<visualization_msgs::msg::Marker>>
        shapes_list_ptr_;

    auto resize_imple(AllAxis new_size) -> void override;

    auto rescale_imple(AnyAxis factor) -> void override;

    auto set_colour_imple(Colour)
        -> void override{}[[nodiscard]] auto get_colour_imple() const
        -> Colour override;

    auto set_parent_frame_name_imple(std::string frame_name) -> void override;

    [[nodiscard]] auto get_location_imple() const
        -> std::tuple<XAxis, YAxis, ZAxis> override;

    auto move_to_imple(XAxis) -> void override;
    auto move_to_imple(YAxis) -> void override;
    auto move_to_imple(ZAxis) -> void override;
    auto move_to_imple(XAxis, YAxis, ZAxis) -> void override;

    auto move_by_imple(XAxis) -> void override;
    auto move_by_imple(YAxis) -> void override;
    auto move_by_imple(ZAxis) -> void override;

    auto move_by_imple(XAxis, YAxis, ZAxis) -> void override;

    auto get_display_markers_imple() -> std::shared_ptr<
        std::vector<visualization_msgs::msg::Marker>> override;

    auto rotate_about_axis_to_imple(ZAxis radians) -> void override;

    [[nodiscard]] auto get_orientation_imple() const -> ZAxis override;

    // std::shared_ptr <shapes::Cube> uav_cube;
    // std::shared_ptr <display::SingleShapeDisplay> uav_cube_display;
};
} // namespace shapes
#endif // UAV_HPP_