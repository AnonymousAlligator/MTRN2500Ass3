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
#include "parallelepiped.hpp"
#include "uav.hpp"
#include "joystick_listener.hpp"

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <map>
#include <memory>
#include <sstream>

int counter = 0;
int previousPress = 0;
int currentPress = 0;
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

        /*

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

        // Create and display a parallelepiped
        auto const my_parallelepiped = std::make_shared<shapes::Parallelepiped>(9,0,0,0);
        auto my_parallelepiped_display =
            std::make_shared<display::SingleShapeDisplay>("my_parallelepiped", 100ms);
        my_parallelepiped_display->display_object(my_parallelepiped);
        ros_worker.add_node(my_parallelepiped_display);

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

       // Creating cubes

       // Create and display a red cube 
        auto my_cube_r1 = std::make_shared<shapes::Cube>(100, 0, 0, 0, 1, 0, 0, 0, 1);
        auto my_cube_r1_display =
            std::make_shared<display::SingleShapeDisplay>("cube_r1", 100ms);
        my_cube_r1_display->display_object(my_cube_r1);
        ros_worker.add_node(my_cube_r1_display);

        // Create and display a yellow cube
        auto my_cube_y1 = std::make_shared<shapes::Cube>(101, 0, 0, 0, 1, 1, 0, 0, 1);
        auto my_cube_y1_display =
            std::make_shared<display::SingleShapeDisplay>("cube_y1", 100ms);
        my_cube_y1_display->display_object(my_cube_y1);
        ros_worker.add_node(my_cube_y1_display);

        // Create and display a green cube
        auto my_cube_g1 = std::make_shared<shapes::Cube>(102, 0, 0, 0, 0, 1, 0, 0, 1);
        auto my_cube_g1_display =
            std::make_shared<display::SingleShapeDisplay>("cube_g1", 100ms);
        my_cube_g1_display->display_object(my_cube_g1);
        ros_worker.add_node(my_cube_g1_display);

        // Create and display a blue cube
        auto my_cube_blue1 = std::make_shared<shapes::Cube>(103, 0, 0, 0, 0, 0, 1, 0, 1);
        auto my_cube_blue1_display =
            std::make_shared<display::SingleShapeDisplay>("cube_blue1", 100ms);
        my_cube_blue1_display->display_object(my_cube_blue1);
        ros_worker.add_node(my_cube_blue1_display);

        // Create and display a black cube
        auto my_cube_black1 = std::make_shared<shapes::Cube>(104, 0, 0, 0, 0, 0, 0, 0, 1);
        auto my_cube_black1_display =
            std::make_shared<display::SingleShapeDisplay>("cube_black1", 100ms);
        my_cube_black1_display->display_object(my_cube_black1);
        ros_worker.add_node(my_cube_black1_display);

        // Create and display a white cube
        auto my_cube_w1 = std::make_shared<shapes::Cube>(105, 0, 0, 0, 1, 1, 1, 0, 1);
        auto my_cube_w1_display =
            std::make_shared<display::SingleShapeDisplay>("cube_w1", 100ms);
        my_cube_w1_display->display_object(my_cube_w1);
        ros_worker.add_node(my_cube_w1_display);

        // Create and display a red cube 
        auto my_cube_r2 = std::make_shared<shapes::Cube>(106, 0, 0, 0, 1, 0, 0, 0, 1);
        auto my_cube_r2_display =
            std::make_shared<display::SingleShapeDisplay>("cube_r2", 100ms);
        my_cube_r2_display->display_object(my_cube_r2);
        ros_worker.add_node(my_cube_r2_display);

        // Create and display a yellow cube
        auto my_cube_y2 = std::make_shared<shapes::Cube>(107, 0, 0, 0, 1, 1, 0, 0, 1);
        auto my_cube_y2_display =
            std::make_shared<display::SingleShapeDisplay>("cube_y2", 100ms);
        my_cube_y2_display->display_object(my_cube_y2);
        ros_worker.add_node(my_cube_y2_display);

        // Create and display a green cube
        auto my_cube_g2 = std::make_shared<shapes::Cube>(108, 0, 0, 0, 0, 1, 0, 0, 1);
        auto my_cube_g2_display =
            std::make_shared<display::SingleShapeDisplay>("cube_g2", 100ms);
        my_cube_g2_display->display_object(my_cube_g2);
        ros_worker.add_node(my_cube_g2_display);

        // Create and display a blue cube
        auto my_cube_blue2 = std::make_shared<shapes::Cube>(109, 0, 0, 0, 0, 0, 1, 0, 1);
        auto my_cube_blue2_display =
            std::make_shared<display::SingleShapeDisplay>("cube_blue2", 100ms);
        my_cube_blue2_display->display_object(my_cube_blue2);
        ros_worker.add_node(my_cube_blue2_display);

        // Create and display a black cube
        auto my_cube_black2 = std::make_shared<shapes::Cube>(110, 0, 0, 0, 0, 0, 0, 0, 1);
        auto my_cube_black2_display =
            std::make_shared<display::SingleShapeDisplay>("cube_black2", 100ms);
        my_cube_black2_display->display_object(my_cube_black2);
        ros_worker.add_node(my_cube_black2_display);

        // Create and display a white cube
        auto my_cube_w2 = std::make_shared<shapes::Cube>(111, 0, 0, 0, 1, 1, 1, 0, 1);
        auto my_cube_w2_display =
            std::make_shared<display::SingleShapeDisplay>("cube_w2", 100ms);
        my_cube_w2_display->display_object(my_cube_w2);
        ros_worker.add_node(my_cube_w2_display);

        // Create and display the flat plane
        auto const my_flat_plane = std::make_shared<shapes::FlatPlane>(13,0,0,0);
        auto my_FlatPlane_display =
            std::make_shared<display::SingleShapeDisplay>("plane", 100ms);
        my_FlatPlane_display->display_object(my_flat_plane);
        ros_worker.add_node(my_FlatPlane_display);

        auto previous_time = std::chrono::steady_clock::now();

        auto x = shapes::XAxis{0};
        auto y = shapes::YAxis{0};
        auto z = shapes::ZAxis{3};
        auto z_cube = shapes::ZAxis{9.0};
        auto z_ground = shapes::ZAxis{0.0};

        // Create and display the UAV
        auto const my_uav = std::make_shared<shapes::UAV>(14,0,0,0);
        auto my_uav_display =
            std::make_shared<display::SingleShapeDisplay>("uav", 100ms);
        my_uav_display->display_object(my_uav);
        ros_worker.add_node(my_uav_display);
        // ros_worker.add_node(my_uav.get_cube_display());
        int m_count = 1;
        int cube_count = 16;
        
        
        

        // Periodically do some work
        while (rclcpp::ok())
        {
            // std::cout << "Program start!" << std::endl;
            auto current_time = std::chrono::steady_clock::now();

            if (current_time - previous_time > 1s)
            {
                // Meowing at rate of 1hz
                std::cout << "meow_" << m_count << std::endl;

                // Moving the UAV
                x.set_value(x.get_value() + input_node->get_x());
                y.set_value(y.get_value() + input_node->get_y());
                z.set_value(z.get_value() + input_node->get_z());

                // Checking if UAV is still in the allowed boundaries
                if (x.get_value() > 40){
                    std::cout << "leaving x boundaries, please go back" << std::endl;
                    x.set_value(39);
                } else if (x.get_value() < -40){
                    std::cout << "leaving x boundaries, please go back" << std::endl;
                    x.set_value(-39);
                }
                if (y.get_value() > 40){
                    std::cout << "leaving y boundaries, please go back" << std::endl;
                    y.set_value(39);
                } else if (y.get_value() < -40){
                    std::cout << "leaving y boundaries, please go back" << std::endl;
                    y.set_value(-39);
                }
                if (z.get_value() > 40){
                    std::cout << "leaving z boundaries, please go back" << std::endl;
                    z.set_value(39);
                } else if (z.get_value() < 1){
                    std::cout << "About to hit the ground!" << std::endl;
                    z.set_value(1);
                }
             
                my_uav->move_to(x, y, z);
                //my_cube_r1->move_to(x, y, z);
                // Logging state of the button
                currentPress = input_node->get_x_signal();
                // Place blocks
                if(input_node->get_x_signal() == 1){
                    currentPress = input_node->get_x_signal();
                    // Checking to see if button is being held down
                    if(previousPress != currentPress)
                    {
                        counter++;
                        std::cout << counter << std::endl;
                        std::cout << "Block dropped!" << std::endl;
                        if (counter == 1)
                        {
                            my_cube_r1->move_to(x, y, z);
                            my_cube_r1->set_a(1.0);
                            // set previous state as current state
                            previousPress = currentPress;
                        }
                        if (counter == 2)
                        {
                            my_cube_y1->move_to(x, y, z);
                            my_cube_y1->set_a(1.0);
                            previousPress = currentPress;
                        }
                        if (counter == 3)
                        {
                            my_cube_g1->move_to(x, y, z);
                            my_cube_g1->set_a(1.0);
                            previousPress = currentPress;
                        }
                        if (counter == 4)
                        {
                            my_cube_blue1->move_to(x, y, z);
                            my_cube_blue1->set_a(1.0);
                            previousPress = currentPress;
                        }
                        if (counter == 5)
                        {
                            my_cube_black1->move_to(x, y, z);
                            my_cube_black1->set_a(1.0);
                            previousPress = currentPress;
                        }
                        if (counter == 6)
                        {
                            my_cube_w1->move_to(x, y, z);
                            my_cube_w1->set_a(1.0);
                            previousPress = currentPress;
                        }
                        if (counter == 7)
                        {
                            my_cube_r2->move_to(x, y, z);
                            my_cube_r2->set_a(1.0);
                            previousPress = currentPress;
                        }
                        if (counter == 8)
                        {
                            my_cube_y2->move_to(x, y, z);
                            my_cube_y2->set_a(1.0);
                            previousPress = currentPress;
                        }
                        if (counter == 9)
                        {
                            my_cube_g2->move_to(x, y, z);
                            my_cube_g2->set_a(1.0);
                            previousPress = currentPress;
                        }
                        if (counter == 10)
                        {
                            my_cube_blue2->move_to(x, y, z);
                            my_cube_blue2->set_a(1.0);
                            previousPress = currentPress;
                        }
                        if (counter == 11)
                        {
                            my_cube_black2->move_to(x, y, z);
                            my_cube_black2->set_a(1.0);
                            previousPress = currentPress;
                        }
                        if (counter == 12)
                        {
                            my_cube_w2->move_to(x, y, z);
                            my_cube_w2->set_a(1.0);
                            previousPress = currentPress;
                        }
                    }
                }
                // removing blocks where RS is pressed
                } else if (input_node->get_clear_flag() == 1){
                    std::cout << "Removing blocks!" << std::endl;
                    counter = 0;
                    my_cube_g1->set_a(0.0);
                }
                previousPress = currentPress;
                // Iterator
                previous_time = current_time;
                m_count++;
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
