// Copyright 2019 Zhihao Zhang License MIT

#ifndef STUDENT_HELPER_HPP_
#define STUDENT_HELPER_HPP_

#include <string>

namespace helper
{

auto constexpr pi = 3.14159265358979323846;

auto constexpr local_frame_name = [](std::string const & zid) {
    return "/" + zid + "/local_frame";
};
auto constexpr world_frame_name = [](std::string const & zid) {
    return "/" + zid + "/world_frame";
};

auto constexpr marker_node_name = [](std::string const & zid) {
    return zid + "_marker_node";
};

auto constexpr joy_node_name = [](std::string const & zid) {
    return zid + "_input_node";
};

} // namespace helper
#endif // STUDENT_HELPER_HPP_
