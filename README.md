robot_no_crash package  
========================

general information
-------------------

This is a fully working package. It uses the 2D robot simulator, rqt_gui, and its own node to control a robot that will not crash.

launch directory
----------------

### files
* robot_supervisor.launch
* launcher.launch

### information

The main launch file to run this node is the robot_supervisor.launch file which gets included in the launcher.launch file that runs the robot simulator and the robot controls. The deliverables section of the instructions does not show a file that includes the robot_supervisor.launch file but based on the previous instructions in the lab I decided to keep the launcher.launch file just in case.

>Create a second launch file. This file must launch the STDR simulator by including the the launch file from the STDR simulator that launches the environment with a robot, and the robot_supervisor. launch file as well. Use the include element as specified on the ROS Wiki page for roslaunch to accomplish this.

### robot_supervisor.launch

#### code

```
<launch>
    <node name="robot_no_crash" type="movement_reader_node" pkg="robot_no_crash" output="screen" />
</launch>
```

#### information

This code simply runs the movement_reader_node from the robot_no_crash package (the node is the movement_reader.cpp file). Launch the file with `roslaunch robot_no_crash robot_supervisor.launch` and optionally `__ns:=<namespace>` to run the node in a desired namespace and to be able to run multiple instances of the node at the same time.

### launcher.launch

#### code

```
<launch>

    <arg name="robot_ns" default="robot0" />

    <param name="$(arg robot_ns)/wall_dist" value="0.5" type="double" />

    <include file="$(find stdr_launchers)/launch/server_with_map_and_gui_plus_robot.launch" />
    <include file="$(find robot_no_crash)/launch/robot_supervisor.launch" ns="$(arg robot_ns)" >
        <remap from="/laser_1" to="/laser_0"/>
    </include>
    <node name="gui" pkg="rqt_gui" type="rqt_gui" />

</launch>
```

#### information

This is the file that runs the node, simulator, and robot controller gui. 

```
    <arg name="robot_ns" default="robot0" />

    <param name="$(arg robot_ns)/wall_dist" value="0.5" type="double" />
```

It takes the argument `robot_ns` as the namespace from the node it is running and the parameter `$(arg robot_ns)/wall_dist` as the distance away from the wall the robot should be before it is stopped from crashing into the wall. The `robot_ns` argument defaults to the value `robot0`. The `$(arg robot_ns)/wall_dist` parameter is set to 0.5 as it briefly appeared in the instructions.

>#Add parameter to the Parameter Server  
>rosparam set /wall_dist 0.5

```
    <include file="$(find stdr_launchers)/launch/server_with_map_and_gui_plus_robot.launch" />
```

Then the server_with_map_and_gui_plus_robot.launch file from the stdr launchers package (a part of stdr mega package) is included which starts a server, opens a map creates a robot. 

```
    <include file="$(find robot_no_crash)/launch/robot_supervisor.launch" ns="$(arg robot_ns)" >
        <remap from="/laser_1" to="/laser_0"/>
    </include>
```

Afterwords the robot_supervisor.launch file from the robot_no_crash package is included so that the robot will not crash. It will open in the namespace given by the `robot_ns` argument. It also remaps the `/laser_1` topic to the `/laser_0` topic as in the instructions.

>For clarity, the effective name of the topic to which the node created in this lab is subscribing to for laser rangefinder data will be changed from laser_1 to laser_0.

src directory
-------------

### files
* movement_reader.cpp

### movement_reader.cpp

#### code

```
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"

ros::Publisher *p_pub; 

bool closeToWall = false;

double wall_dist = 0.0;

void isCloseToWall(const sensor_msgs::LaserScan::ConstPtr& laser)
{


 std::vector<float> ranges = laser->ranges;


 for(size_t c = 0; c < ranges.size(); c++)
 {

  if (c > 264 && c < 401)
  {

    float range = ranges[c];


    if (range <= wall_dist)
    {
    
      closeToWall = true;


      break;


    }
  
    closeToWall = false;
  } else {

    float range = ranges[c];


    if (range <= wall_dist / 2)
    {
    
      closeToWall = true;


      break;


    }

  }



 }
 


}


void chatterCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
  
  geometry_msgs::Twist twist_msg;

  if (closeToWall == true)
  {

    twist_msg.linear.x = 0.0;  
    twist_msg.angular.z = msg->angular.z;

  } else {

    twist_msg.linear.x = msg->linear.x;  
    twist_msg.angular.z = msg->angular.z;

  }

  p_pub->publish(twist_msg);

}

int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "movement_reader");

  
  ros::NodeHandle n;

  p_pub = new ros::Publisher(n.advertise<geometry_msgs::Twist>("cmd_vel", 1000));

  ros::Subscriber laserSub = n.subscribe<sensor_msgs::LaserScan>("laser_0", 1000, isCloseToWall);
  
  ros::Subscriber sub = n.subscribe<geometry_msgs::Twist>("des_vel", 1000, chatterCallback);

  n.getParamCached("wall_dist", wall_dist);

  


 
  
  ros::spin();

  delete p_pub;

  return 0;
}
```

#### information

This file is the code for the movement_reader_node node. 

```
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
```

The code begins with its statements that include ros/ros, std_msgs/String, geometry_msgs/Twist and sensor_msgs/LaserScan.

```
ros::Publisher *p_pub; 

bool closeToWall = false;

double wall_dist = 0.0;
```

Then it continues by creating global variables for running the node. `ros::Publisher *p_pub` is a publisher handle that gets set later. `bool closeToWall` is a boolean value that indicates whether the robot is too close to a wall or not. `double wall_dist` is a double that indicates at what distance the robot should start avoiding the wall and is based on a ros parameter that is called later.

```
  p_pub = new ros::Publisher(n.advertise<geometry_msgs::Twist>("cmd_vel", 1000));
```

The p_pub is set to a publisher handle so that the node will be publishing `geometry_msgs/Twist` messages to the `cmd_vel` topic.

```
  n.getParamCached("wall_dist", wall_dist);
```

The `wall_dist` global parameter is set based on the ros parameter `wall_dist`.

```
  ros::Subscriber laserSub = n.subscribe<sensor_msgs::LaserScan>("laser_0", 1000, isCloseToWall);
```

It subscribes to the `laser_0` topic and uses it to run the `isCloseToWall` function. 

```
void isCloseToWall(const sensor_msgs::LaserScan::ConstPtr& laser)
{


 std::vector<float> ranges = laser->ranges;


 for(size_t c = 0; c < ranges.size(); c++)
 {

  if (c > 264 && c < 401)
  {

    float range = ranges[c];


    if (range <= wall_dist)
    {
    
      closeToWall = true;


      break;


    }
  
    closeToWall = false;
  } else {

    float range = ranges[c];


    if (range <= wall_dist / 2)
    {
    
      closeToWall = true;


      break;


    }

  }



 }
 


}
```

The `isCloseToWall` function decides when the robot should stop moving forward based on the `wall_dist` and the sensor_msgs/LaserScan values. It goes through all this different distances given by the robot. For the lasers between 265 and 400 the robot is close to the wall if it is closer than what is given by `wall_dist` then the robot `isCloseToWall`. This is a cone in front and center of the robot. The amount of lasers in this cone was decided so that when the robot is running into straight into a wall and the robot turns while continuing to try and advance that it will have to turn enough to not crash into the wall. The rest of the lasers will stop the robot when it is at a distance of only half the `wall dist`. This allows the robot to move more freely while reducing the amount of crashes caused by passing next to a thin wall and turning in a manner so as for the center cone to not see the wall.  

Basically the robot is able to move forward as long as it is not `wall_dist` away from crashing straight into a wall and will not crash into a wall in any direction while turning.

```
  ros::Subscriber sub = n.subscribe<geometry_msgs::Twist>("des_vel", 1000, chatterCallback);
```

Here the node subscribes to the `des_vel` topic to recieve the robot movement instructions probably sent by the rqt_gui robot controller and opens them in the `chatterCallBack` function.

```
void chatterCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
  
  geometry_msgs::Twist twist_msg;

  if (closeToWall == true)
  {

    twist_msg.linear.x = 0.0;  
    twist_msg.angular.z = msg->angular.z;

  } else {

    twist_msg.linear.x = msg->linear.x;  
    twist_msg.angular.z = msg->angular.z;

  }

  p_pub->publish(twist_msg);

}
```

The `chatterCallBack` function basically just passes on the same geometry_msgs/Twist messages that it recieves if the robot is not `closeToWall`, but otherwise sends back the recieved message with the `linear.x` value set to 0 if the robot is `closeToWall` so as to stop the robot from advancing before it crashes.

package.xml
-----------

### code

```
<?xml version="1.0"?>
<package format="2">
  <name>robot_no_crash</name>
  <version>0.0.0</version>
  <description>The robot_no_crash package</description>

  <!-- One maintainer tag required, multiple allowed, one person per tag -->
  <!-- Example:  -->
  <!-- <maintainer email="jane.doe@example.com">Jane Doe</maintainer> -->
  <maintainer email="ubuntu@todo.todo">ubuntu</maintainer>


  <!-- One license tag required, multiple allowed, one license per tag -->
  <!-- Commonly used license strings: -->
  <!--   BSD, MIT, Boost Software License, GPLv2, GPLv3, LGPLv2.1, LGPLv3 -->
  <license>TODO</license>


  <!-- Url tags are optional, but multiple are allowed, one per tag -->
  <!-- Optional attribute type can be: website, bugtracker, or repository -->
  <!-- Example: -->
  <!-- <url type="website">http://wiki.ros.org/robot_no_crash</url> -->


  <!-- Author tags are optional, multiple are allowed, one per tag -->
  <!-- Authors do not have to be maintainers, but could be -->
  <!-- Example: -->
  <!-- <author email="jane.doe@example.com">Jane Doe</author> -->


  <!-- The *depend tags are used to specify dependencies -->
  <!-- Dependencies can be catkin packages or system dependencies -->
  <!-- Examples: -->
  <!-- Use depend as a shortcut for packages that are both build and exec dependencies -->
  <!--   <depend>roscpp</depend> -->
  <!--   Note that this is equivalent to the following: -->
  <!--   <build_depend>roscpp</build_depend> -->
  <!--   <exec_depend>roscpp</exec_depend> -->
  <!-- Use build_depend for packages you need at compile time: -->
  <!--   <build_depend>message_generation</build_depend> -->
  <!-- Use build_export_depend for packages you need in order to build against this package: -->
  <!--   <build_export_depend>message_generation</build_export_depend> -->
  <!-- Use buildtool_depend for build tool packages: -->
  <!--   <buildtool_depend>catkin</buildtool_depend> -->
  <!-- Use exec_depend for packages you need at runtime: -->
  <!--   <exec_depend>message_runtime</exec_depend> -->
  <!-- Use test_depend for packages you need only for testing: -->
  <!--   <test_depend>gtest</test_depend> -->
  <!-- Use doc_depend for packages you need only for building documentation: -->
  <!--   <doc_depend>doxygen</doc_depend> -->
  <buildtool_depend>catkin</buildtool_depend>
  <build_depend>gazebo_plugins</build_depend>
  <build_depend>geometry_msgs</build_depend>
  <build_depend>roscpp</build_depend>
  <build_depend>rviz</build_depend>
  <build_depend>sensor_msgs</build_depend>
  <build_depend>std_msgs</build_depend>
  <build_depend>urdf</build_depend>
  <build_depend>velodyne_description</build_depend>
  <build_depend>xacro</build_depend>
  <build_export_depend>gazebo_plugins</build_export_depend>
  <build_export_depend>geometry_msgs</build_export_depend>
  <build_export_depend>roscpp</build_export_depend>
  <build_export_depend>rviz</build_export_depend>
  <build_export_depend>sensor_msgs</build_export_depend>
  <build_export_depend>std_msgs</build_export_depend>
  <build_export_depend>urdf</build_export_depend>
  <build_export_depend>velodyne_description</build_export_depend>
  <build_export_depend>xacro</build_export_depend>
  <exec_depend>gazebo_plugins</exec_depend>
  <exec_depend>geometry_msgs</exec_depend>
  <exec_depend>roscpp</exec_depend>
  <exec_depend>rviz</exec_depend>
  <exec_depend>sensor_msgs</exec_depend>
  <exec_depend>std_msgs</exec_depend>
  <exec_depend>urdf</exec_depend>
  <exec_depend>velodyne_description</exec_depend>
  <exec_depend>xacro</exec_depend>


  <!-- The export tag contains other, unspecified, tags -->
  <export>
    <!-- Other tools can request additional information be placed here -->

  </export>
</package>
```

### information

It is a properly configured `package.xml` file with the proper `<build_depend>`, `<build_export_depend>` and `<exec_depend>` tags for the package's dependencies.

CMakeLists.txt
--------------

### code

```
cmake_minimum_required(VERSION 3.0.2)
project(robot_no_crash)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  gazebo_plugins
  geometry_msgs
  roscpp
  rviz
  sensor_msgs
  std_msgs
  urdf
  velodyne_description
  xacro
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   geometry_msgs#   sensor_msgs#   std_msgs
# )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES robot_no_crash
#  CATKIN_DEPENDS gazebo_plugins geometry_msgs roscpp rviz sensor_msgs std_msgs urdf velodyne_description xacro
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/robot_no_crash.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/robot_no_crash_node.cpp)
add_executable(movement_reader_node src/movement_reader.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )
target_link_libraries(movement_reader_node ${catkin_LIBRARIES})

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# catkin_install_python(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
# install(TARGETS ${PROJECT_NAME}_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_robot_no_crash.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
```

### information

It is a properly configured `CMakeLists.txt` file with the proper dependencies and that indicates that `movement_reader.cpp` is an executable.

README.md
---------

### information

This is just my very pretty `README.md` file. Hope you enjoyed.