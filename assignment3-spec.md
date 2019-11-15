# Assignment 3: Vehicle Game

## Relevant course learning objectives
1.	Be well versed with structured and modular programming using C/C++ and to appreciate the use of software to communicate with external devices.  
2.	Understand how to interface to an external device through a computer program to effect control action.  
3.	Be able to develop prototype user interfaces to assist in the development of controlled Mechatronic systems.  

## Assignment learning objectives
* Structure a set of classes appropriately, handling inheritance, access and polymorphism.
* Make use of external libraries (including ROS) to parse user input from config files and an external device.
* Implement real-time visualisation and control of simulated objects in response to user input.
* Follow a prescribed C++ coding style.
* Collaborate within a small team effectively using code management tools and practices.

## Getting started
Choose a team member from within your tutorial as you will be working in pairs. You will be be collaborating with your team member using the private Github assignment3 repository created for your team.

After the repository was created, you must do the following:

1. Both members need to add an empty file to the root directory of the repository with their zid as the filename in the format `z0000000.txt`. 
2. One member only will create a `review` branch. The style task will be assessed on that branch. **Do not merge anything into this branch** This branch must be named `review`.
3. Each member must create their own development branches, each with a meaningful branch name (ie `process_input`). You should be doing work in a development branch and only merge tested working code into the `master` branch after you have completed a requirement. Failure to correctly branch off may lead to team members' code being lost.

**Do not modify `interfaces.hpp`, you may modify all other `hpp` and `cpp` files. If you want to add extra `hpp` or `cpp` files, you need to update `CMakeLists.txt` in the `target_sources` section.**


## Submission process
One member will create a **pull request** from the `master` to the `review` branch with all your commits.

## Deadlines
Code will be due by Friday 5 pm week 12 (6 December) for style checking. Your demonstration will be during your lab timeslot in week 13 (during the exam period).

There will be no lab class in week 10 (as it has been moved to week 13). However, consultations will be made available between weeks 10 and 13.

At least one member must be available for the final demonstration for presenting your work.
 
## Task Overview
Your team has been tasked with developing a Unmanned Aerial Vehicle (UAV) simulator that shows a single UAV dropping blocks in a Minecraft like game.
Players will use a game controller to fly the UAV and drop blocks.

Begin by looking at the provided interface diagram, and designing the additional classes that are needed, including the methods that will be required in each class (before you even start thinking about coding them!). These classes will include not only shapes, but one or more vehicle related classes. You will probably want a class to represent your UAV, which you will draw using the shapes listed below.


### Simulation Scene
In RVIZ2, show the ground using a green `Flat Plane` shape object. 
Add some variety to the simulation scene by including some object such as trees and houses.

For the UAV, you will be designing your own UAV. You can be as creative as you want with your design. Pick any colour for the starting block, attached to the bottom of your UAV.

You need to use each shape at least once in the scene. 

## Tasks
This assignment is out of 40 marks scaled to to 20% of your course grade.

### Program Demonstration (20 marks):
1. Display each of the shapes from the `Shapes Specification` in RVIZ2. You should have done this as part of building your scene. 
1. Display your UAV in RVIZ2. You will not have to modify the shape of the UAV for demonstration. Your UAV will also be holding a coloured block below it.
1. Control your UAV using a joystick controller, further requirements are given in UAV controls section.
1. Release the held block from the UAV by the press of a joystick button. The block will remain at the position and orientation it was released at.
1. Immediately spawn a new block after the previous block has been released. The block should have the next colour from the list of colours given in the ColourInterface section. Cycle back to the beginning once all the colours have been shown.
1. Have a joystick button that will clear all the blocks.

Your tutor will also assess how well your program works overall.

#### UAV controls
1. One thumbstick (two axes) to control the x and y position of the UAV.
2. Left and right triggers to raise and lower the UAV respectively.
3. Other left/right joystick axis to control the heading (yaw) of the UAV.
5. One button to release a block. Only release one block per button press.
5. One button to clear all the blocks.

#### Error handling
Your program should handle all possible errors including but not limited to:
1. Lost connection to the controller.
2. UAV flying too heigh or too low, lost UAV. You may set these limits, but they must be sensible.
3. Incorrect axis configuration.

Error handling must not crash the program but may exit the program with an error message if it's the most suitable solution. 
The end-user experience will be taken into consideration in your demonstration.

### Program Design Presentation (10 marks):
Prepare a short presentation or talk (around 4 min) for your tutor on the design you have chosen and the reasoning behind your design. 
Discuss how you have applied polymorphism, inheritance, composition, functions, DRY principles, etc, and the overall structure of your program.

Your tutor will then ask some questions about your design choices.

### Teamwork and style (10 marks):
You need to conduct code reviews and provide feedback on your partner's code in a timely manner. Evidence of this will need to be in GitHub. You are also responsible for testing your partner's code, it is highly recommended you write test cases to make sure your code still works for the demonstration. 

Your tutor may balance the distribution of marks between team members based on objective evidence on GitHub. Please let your tutor or the lecturer know if there are any issues with your group.

#### Style
The most important thing with style is to make your code understandable to other people. 

* Follow the course style guide in `style_guide.md` in this repository which is based on ROS2 development guide. When contributing to an existing codebase, you should follow the style already being used. This ensures consistency between developers and helps to maintain the codebase quality.
* Neat and tidy code, style is consistent throughout. Ensure your code is formatted according to the clang-format file, this ensures your code does not inadvertently get changed when other people work in the same file, polluting the git commit history. Clang-format is commonly used to automatically format the code. Most programming IDE have clang-format intergration built in. 
* Good choice of names that are meaningful and descriptive.
* Good use of functions to split code into manageable segments and avoid code duplication (DRY principle).
* Appropriate use of C++ features.
* Good documentation and comments. Comments should explain the technical aspects of your code, not merely restating what you have written.
* No error or warning messages when compiling. Error and warning messages are generally sign something is wrong with your program.
* The master branch must be able to compile and work. This ensures you always have a working copy for demonstration.
* Consistent use of source management. Development should be done in feature based branches and merged into the `master` branch incrementally after milestone development is complete. 
* Keep your Github repository up to date with your work. 
* Write meaningful commit messages.

We use automated tool to check your code compiles and adhere to the style guide. The tool will compile your code when you merge code to the master branch or when you make a pull request to review branch. You can check the result by going to the check tab of your pull request and click the build link. We run the following tools:
1. Build 
1. Cpplint to check complience with the style guide.
1. Cppcheck normally dont cause any issue.
1. Clang-format to check for formatting

You can also run the tests locally by running colcon in test mode. Please consult colcon documentation. 

## Bonus Marks (2.5 marks each):
Must be fully functioning and impress the tutor to get the marks. No help will be provided for the bonus tasks.
1. Have the UAV rotors spin with their velocity corresponding to the height of the UAV. For this, you need to show at least two rotors.
2. Rather than have the block stay stationary, show the dropped block fall to the ground at a constant velocity of 0.1m/s and stop on the ground plane. You may ignore collision with other objects.

## Abstract Classes

Through an abstract class, a set of common functionalities shared by all the derived class may be defined. This creates an interface through which user can use any the derived classes. An [interface](https://en.wikipedia.org/wiki/Interface_(computing)) in computing describes the shared boundary between separate components of a program.
In this assignment abstract classes that specify an interface between different section of the program will be denoted with the word `Interface` in their name. For example if `Sphere` is derived from `ShapeCommonInterface`,then `Sphere` guaranteed to have all the functions specified in `ShapeCommonInterface`.

The interface classes will be implemented using [Non-Virtual Interface](https://en.wikibooks.org/wiki/More_C%2B%2B_Idioms/Non-Virtual_Interface) pattern. 
Each interface class will specify the public interface through a set of public non-virtual functions. 
The public interface functions will call private virtual implementation functions that a class implementing the interface need to implement. Your class needs to implement those virtual functions.

A brief overview of all the interfaces will be provided here, 
You need to select the appropriate interfaces for your classes.
You may need to add additional interfaces.

How the interfaces fits together is illustrated in inheritance_diagram.png.


### Coordinate System Interfaces
The following interfaces allow the manipulation of a coordinate position and orientation.

#### AxisInterface
`AxisInterface` represents some value with respect to some axis of a coordinate system. The value can be modified or rescaled.
Example of what the value would represent can be location, distance or angle. Axis value can be scaled by using the operator `*`. 

For simplicity in this assignment, some components that would be expected from a fully designed axis library are omitted. For example, additional types could be used to express what units the value represents. Other math operators are also omitted.

Classes implementing this interface have been provided for you. `XAxis`, `YAxis`, and `ZAxis` represents the x, y, and z-axis respectively. `AllAxis` represents a value that applies to all three axes. `AnyAxis` represents a value that is not associated with a particular axis, it could be applied to any axis.

#### LocationInterface
`LocationInterface` represents the location of an object in 3D space. Essentially a linear translation of the coordinate frame with respect to a parent frame. The point may be moved to a new position or moved by a certain distance. The location of a point can be obtained as a `std::tuple` of `Axis`, `YAxis`, `ZAxis`. 

#### YawInterface
`YawInterface` represents the rotation of an object about the z-axis, use units in radians.

#### DisplayableInterface
`DisplayableInterface` is able to return a `std::shared_ptr` of `visualization_msgs::msg::Marker` to be used to display the object in RVIZ.

#### DisplayOutputInterface
`DisplayOutputInterface` will get the marker message from `DisplayableInterface` and send the marker to RVIZ.

#### ResizeableInterfaceBase
`ResizeableInterfaceBase` represents the ability to resizing the object in one of the axes to a new size or rescaling by a factor,
with respect to certain `AxisInterface`. 
`BasicResizeableInterface` requires equal scaling in the x,y and z-axis.

### ColourInterface
`ColourInterface` get and set the colour of a displayable object.
Colour may be one of the following: 
* red
* yellow
* green
* blue
* black
* white

Exact RGB values for each colour is not specified, just make it obvious which colour it is.

### ShapeCommonInterface
All shapes need to implement this common interface:
```c++
	class ShapeCommonInterface : 
		public virtual BasicResizeableInterface,
		public virtual LocationInterface,
		public virtual YawInterface,
		public virtual DisplayableInterface
	{
	};
```


## Shapes Specification
You need to design a class hierarchy using polymorphism, class inheritance and composition of the following shapes. 

1. Octagonal Prism
1. Rectangular Prism
1. Triangular Prism
1. Octagonal Pyramid
1. Rectangular Pyramid
1. Triangular Pyramid
1. Square Pyramid
1. Sphere
1. Parallelepiped
1. Cube
1. Cone
1. Cylinder
1. Flat Plane

How to specify the dimension of each shape is implementation detail you can decide.
