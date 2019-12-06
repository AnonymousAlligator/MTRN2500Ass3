// SPDX-License-Identifier: MIT
/**
 *  \brief     Assignment 3 starter program
 *  \details   Program will show a sphere in RVIZ2
 *  \author    Zhihao Zhang
 *  \version   0.11.15
 *  \date      Nov 2019
 *  \copyright MIT
 **/

#include "cube.hpp"
#include "flat_plane.hpp"
#include "rclcpp/rclcpp.hpp" // http://docs.ros2.org/dashing/api/rclcpp/
#include "single_shape_display.hpp"
#include "sphere.hpp"

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <map>
#include <memory>
#include <sstream>

// ReSharper disable once CppParameterMayBeConst
auto main(int argc, char * argv[]) -> int
{
    using namespace std::chrono_literals;

    try
    {
        rclcpp::init(argc, argv); // Initialise ROS2
        std::cout << "Demo program starting, press enter to display shape.";
        std::cin.ignore();

        auto ros_worker = rclcpp::executors::SingleThreadedExecutor{};

        // Create a sphere and display node for the sphere.
        auto const my_sphere = std::make_shared<shapes::Sphere>(0);
        auto my_shape_display =
            std::make_shared<display::SingleShapeDisplay>("shape_1", 100ms);

        // Set link sphere to the display output
        my_shape_display->display_object(my_sphere);
        // Add display node to list of node ros automatically manage.
        ros_worker.add_node(my_shape_display);

        // Create and display another sphere
        auto const my_sphere_2 = std::make_shared<shapes::Sphere>(1);
        auto my_shape_display_2 =
            std::make_shared<display::SingleShapeDisplay>("shape_2", 100ms);
        my_shape_display_2->display_object(my_sphere_2);
        ros_worker.add_node(my_shape_display_2);

        auto previous_time = std::chrono::steady_clock::now();
        auto x = shapes::XAxis{0.0};
        auto const yz = shapes::AnyAxis{0.0};

        // Create and display the flat plane
        auto const my_flat_plane = std::make_shared<shapes::FlatPlane>(13);
        auto my_FlatPlane_display =
            std::make_shared<display::SingleShapeDisplay>("plane", 100ms);
        my_FlatPlane_display->display_object(my_flat_plane);
        ros_worker.add_node(my_FlatPlane_display);

        // Create and display a cube
        auto const my_cube = std::make_shared<shapes::Cube>(2);
        auto my_cube_display =
            std::make_shared<display::SingleShapeDisplay>("cube", 100ms);
        my_cube_display->display_object(my_cube);
        ros_worker.add_node(my_cube_display);

        // Periodically do some work
        while (rclcpp::ok())
        {
            auto current_time = std::chrono::steady_clock::now();
            if (current_time - previous_time > 1s)
            {
                // Meowing at rate of 1hz.
                std::cout << "meow\n";

                // Also move the sphere a bit
                x.set_value(x.get_value() + 0.25);
                my_sphere_2->move_to(x, yz, yz);

                previous_time = current_time;
            }
            ros_worker.spin_some(50ms);
        }
    }
    catch (std::exception & e)
    {
        // Something wrong occured, printing error message.
        std::cerr << "Error message:" << e.what() << "\n";
    }

    rclcpp::shutdown(); // Cleaning up before exiting.

    return 0;
}
