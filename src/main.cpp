// SPDX-License-Identifier: MIT
/**
 *  \brief     Assignment 3 starter program
 *  \details   Program will show a sphere in RVIZ2
 *  \author    Zhihao Zhang
 *  \version   0.11.15
 *  \date      Nov 2019
 *  \copyright MIT
 **/

#include "rclcpp/rclcpp.hpp" // http://docs.ros2.org/dashing/api/rclcpp/
#include "single_shape_display.hpp"
#include "sphere.hpp"

#include <chrono>
#include <iostream>
#include <memory>

// ReSharper disable once CppParameterMayBeConst
auto main(int argc, char * argv[]) -> int
{
    using namespace std::chrono_literals;

    try
    {
        rclcpp::init(argc, argv); // Initialise ROS2
        std::cout << "Demo program starting, press enter to display shape.\n";
        std::cin.ignore();

        auto const my_sphere = std::make_shared<shapes::Sphere>();
        auto my_shape_display =
            std::make_shared<display::SingleShapeDisplay>("z0000000", 100ms);
        my_shape_display->display_object(my_sphere);

        auto ros_worker = rclcpp::executors::SingleThreadedExecutor{};
        ros_worker.add_node(my_shape_display);
        auto previous_time = std::chrono::steady_clock::now();

        // Periodically do some work
        while (rclcpp::ok())
        {
            auto current_time = std::chrono::steady_clock::now();
            if (current_time - previous_time > 1s)
            {
                // Meowing at rate of 1hz.
                std::cout << "meow\n";
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
