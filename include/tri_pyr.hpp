// Copyright 2019 Zhihao Zhang License MIT
// Edited by Curtis Ly (z5209698)

#ifndef TRIPYR_HPP_
#define TRIPYR_HPP_

#include "interfaces.hpp"
#include "visualization_msgs/msg/marker.hpp"

#include <memory>
#include <string>
#include <tuple>
#include <vector>

namespace shapes
{
// ReSharper disable once CppClassCanBeFinal
class TriPyr : public ShapeCommonInterface
{
public:
    explicit TriPyr(int id, double posx, double posy, double posz, double a);

protected:
    AllAxis length_;
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
};
} // namespace shapes
#endif // TRIPYR_HPP_