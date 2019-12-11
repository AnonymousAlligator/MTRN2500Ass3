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
#include "cylinder.hpp"
#include "rect_prism.hpp"
#include "cube.hpp"
#include "flat_plane.hpp"
#include "rect_pyr.hpp"
#include "sqr_pyr.hpp"
#include "triangle.hpp"
#include "tri_pyr.hpp"
#include "tri_prism.hpp"
#include "cone.hpp"
#include "oct_prism.hpp"
#include "oct_pyr.hpp"
#include "joystick_listener.hpp"

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

        auto input_node = std::make_shared<assignment3::JoystickListener>(
            "z0000000");
        ros_worker.add_node(input_node);

        // Create a sphere and display node for the sphere
        auto const my_sphere = std::make_shared<shapes::Sphere>(0,0,0,0);
        auto my_shape_display =
            std::make_shared<display::SingleShapeDisplay>("shape_1", 100ms);

        // Set link sphere to the display output
        my_shape_display->display_object(my_sphere);
        // Add display node to list of node ros automatically manage.
        ros_worker.add_node(my_shape_display);

        // Create and display another sphere
        auto const my_sphere_2 = std::make_shared<shapes::Sphere>(1,0,0,0);
        auto my_shape_display_2 =
            std::make_shared<display::SingleShapeDisplay>("shape_2", 100ms);
        my_shape_display_2->display_object(my_sphere_2);
        ros_worker.add_node(my_shape_display_2);
        /*
         // Create and display a rectangular prism
        auto const my_rect_prism = std::make_shared<shapes::RectPrism>(2,0,0,0);
        auto my_rect_prism_display =
            std::make_shared<display::SingleShapeDisplay>("rectangular_prism", 100ms);
        my_rect_prism_display->display_object(my_rect_prism);
        ros_worker.add_node(my_rect_prism_display);

        // Create and display a octagonal prism
        auto const my_oct_prism = std::make_shared<shapes::OctPrism>(3,0,0,0);
        auto my_oct_prism_display =
            std::make_shared<display::SingleShapeDisplay>("octagonal_prism", 100ms);
        my_oct_prism_display->display_object(my_oct_prism);
        ros_worker.add_node(my_oct_prism_display);

        // Create and display a octagonal pyramid
        auto const my_oct_pyr = std::make_shared<shapes::OctPyr>(4,0,0,0);
        auto my_oct_pyr_display =
            std::make_shared<display::SingleShapeDisplay>("octagonal_pyramid", 100ms);
        my_oct_pyr_display->display_object(my_oct_pyr);
        ros_worker.add_node(my_oct_pyr_display);        

        // Create and display the rectangular pyramid
        auto const my_rect_pyr = std::make_shared<shapes::RectPyr>(5,0,0,0);
        auto my_rect_pyr_display =
            std::make_shared<display::SingleShapeDisplay>("rect_pyr", 100ms);
        my_rect_pyr_display->display_object(my_rect_pyr);
        ros_worker.add_node(my_rect_pyr_display);
        
        // Create and display the triangular pyramid
        auto const my_tri_pyr = std::make_shared<shapes::TriPyr>(6,0,0,0);
        auto my_tri_pyr_display =
            std::make_shared<display::SingleShapeDisplay>("tri_pyr", 100ms);
        my_tri_pyr_display->display_object(my_tri_pyr);
        ros_worker.add_node(my_tri_pyr_display);
        
        // Create and display the square pyramid
        auto const my_sqr_pyr = std::make_shared<shapes::SqrPyr>(7,0,0,0);
        auto my_sqr_pyr_display =
            std::make_shared<display::SingleShapeDisplay>("sqr_pyr", 100ms);
        my_sqr_pyr_display->display_object(my_sqr_pyr);
        ros_worker.add_node(my_sqr_pyr_display);

        // Create and display a triangluar prism
        auto const my_tri_prism = std::make_shared<shapes::TriPrism>(8,0,0,0);
        auto my_tri_prism_display =
            std::make_shared<display::SingleShapeDisplay>("triangular_prism", 100ms);
        my_tri_prism_display->display_object(my_tri_prism);
        ros_worker.add_node(my_tri_prism_display);

        // Create and display a cube
        auto const my_cube = std::make_shared<shapes::Cube>(10,0,0,0);
        auto my_cube_display =
            std::make_shared<display::SingleShapeDisplay>("cube", 100ms);
        my_cube_display->display_object(my_cube);
        ros_worker.add_node(my_cube_display);

        // Create and display a cone
        auto const my_cone = std::make_shared<shapes::Cone>(11,0,0,0);
        auto my_cone_display =
            std::make_shared<display::SingleShapeDisplay>("cone", 100ms);
        my_cone_display->display_object(my_cone);
        ros_worker.add_node(my_cone_display);
        
        // Create and display a cylinder
        auto const my_cylinder = std::make_shared<shapes::Cylinder>(12,0,0,0);
        auto my_cylinder_display =
            std::make_shared<display::SingleShapeDisplay>("cylinder", 100ms);
        my_cylinder_display->display_object(my_cylinder);
        ros_worker.add_node(my_cylinder_display);
        */
        // Create and display the flat plane
        auto const my_flat_plane = std::make_shared<shapes::FlatPlane>(13,0,0,0);
        auto my_FlatPlane_display =
            std::make_shared<display::SingleShapeDisplay>("plane", 100ms);
        my_FlatPlane_display->display_object(my_flat_plane);
        ros_worker.add_node(my_FlatPlane_display);

        auto previous_time = std::chrono::steady_clock::now();
        auto x = shapes::XAxis{0.0};
        auto y = shapes::YAxis{0.0};
        auto z = shapes::ZAxis{0.0};

        // Create UAV here
        
        // Periodically do some work
        while (rclcpp::ok())
        {
            auto current_time = std::chrono::steady_clock::now();
            if (current_time - previous_time > 0.2s)
            {
                // Also move the sphere a bit
                x.set_value(x.get_value() + input_node->get_x());
                y.set_value(y.get_value() + input_node->get_y());
                z.set_value(z.get_value() + input_node->get_z());
                my_sphere_2->move_to(x, y, z);

                // While joystick input is whatever it moved by that much
                // If button is pressed, then get position of UAV and drop the block
                // Cycle to next block colour
                // Flag for if the RShoulder is pressed
                // Remove all placed blocks

                // Iterator
                previous_time = current_time;
            }
            ros_worker.spin_some(50ms);
        }
    }
    catch (std::exception & e)
    {
        // Something wrong occured, printing error message
        std::cerr << "Error message:" << e.what() << "\n";
    }
    rclcpp::shutdown(); // Cleaning up before exiting
    return 0;
}
