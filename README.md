# ROS 1
First create your workspace, after creating workspace go to src and use 
```catkin_create_pkg package-name dependenices seperated by space```
and go back to root and do ```source devel/setup.bash```, after which run ```catkin_make```.
<br><b>NOTE:</b> The difference between CPP and PY files is that for CPP you have to include the excec in the CMake so that the compiler knows it is a CPP file to be converted to an executable. If it is a PY file then you don't have to do anything except for making the file executable by using the following command <br>```chmod 777 <b>$path/to/file</b>```<br>
Go to CMakeLists.txt -> <br>
```
find_package(catkin REQUIRED dependencies like roscpp rospy)
add_executable(executable_name relative-path to the source-file)
target_link_libraries(executable_name ${catkin_LIBRARIES})
```
Go to package.xml -><br>
```<build_depend>package-name</build_depend><run_depend>package-name</run_depend>```
<br>package-name means dependencies here - roscpp 

 Once this is done, then we can run roscore and rosrun for the respective publisher and subscriber.<br>```rosrun package-name(project name) executable_name```

## Example
```
cd ~/catkin-ws/src/
catkin_create_pkg agitr roscpp
cd ~/catkin-ws/
source devel/setup.bash
catkin_make
cd ~/catkin-ws/src/agitr/src/
touch hello.cpp
Write the required code.
#include <ros/ros.h>

int main(int argc, char **argv){
    ros::init(argc, argv, "hello_cpp");
    ros::NodeHandle nh;
    ROS_INFO_STREAM("HELLO ROS!");
}
```
In CMakeLists.txt ->
```
find_package(catkin REQUIRED dependencies roscpp)
add_executable(hello src/hello.cpp)
target_link_libraries(hello ${catkin_LIBRARIES})
```
In package.xml ->
```
<build_depend>roscpp</build_depend><run_depend>roscpp</run_depend>
```
Save all the files.
``` 
cd ~/catkin-ws/
source devel/setup.bash
catkin_make
```
```roscore``` in a seperate terminal
```
rosrun agitr hello
```

<hr>

# ROS 2
## How to send randomly-generated velocity commands to a turtlesim turtle?

First, create the required cpp file, write the required code->
```
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <stdlib.h> //for rand and RAND_MAX

int main(int argc, char **argv){
    ros::init(argc, argv, "publish_velocity");
    ros::NodeHandle nh;
    
    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 1000);
    srand(time(0));

    ros::Rate rate(2);
    int count = 0;
    while(ros::ok()){
        geometry_msgs::Twist msg;
        msg.linear.x =  double(rand())/double(RAND_MAX);
        msg.angular.z = 2*double(rand())/double(RAND_MAX) - 1;;
        
        
        pub.publish(msg);

        ROS_INFO_STREAM("Sending random velocity commad:"<<"linear="<<msg.linear.x<<"angular="<<msg.angular.z);

        rate.sleep();
    }

}
```
Go to the CMakeLists ->
```
find_package(catkin REQUIRED dependencies roscpp geometry_msgs)
add_executable(randomv src/random_vel.cpp)
target_link_libraries(randomv ${catkin_LIBRARIES})
```

In package.xml, add ->
```
  <build_depend>geometry_msgs</build_depend> 
  <exec_depend>geometry_msgs</exec_depend>
```
Save all the files.
``` 
cd ~/catkin-ws/
source devel/setup.bash
catkin_make
```
```roscore``` in a seperate terminal<br>
```rosrun turtlesim turtlesim_node``` in another terminal and finally 
```
rosrun agitr randomv
```
in another terminal obvi rofl

<b>Errors faced</b><br>
1. Valid Characters Error => Cannot use ```.``` in the name of the package in cpp file.

# ROS 3
## Subscribe to pose 
First, create the required cpp file, write the required code->
```
#include <ros/ros.h>
#include <turtlesim/Pose.h>
#include <iomanip> //for setprecision and setfixed

void poseMessageReceive(const turtlesim::Pose &msg){
    ROS_INFO_STREAM(std::setprecision(2)<<std::fixed<<"Position=("<<msg.x<<","<<msg.y<<")"<<" direction="<<msg.theta);
}

int main(int argc, char **argv){
    ros::init(argc, argv, "subscribe_to_pose");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("turtle1/pose", 1000, &poseMessageReceive);

    ros::spin();
}
```
While creating a ros::Subscriber object, we do not explicitly
mention the message type anywhere. The C++ compiler infers the correct message type based on the data type of the callback
function pointer we provide, i.e, poseMessageRececive
<br>
### Go to the CMakeLists ->
```
add_executable(subscribe_pose src/subscriber.cpp)
target_link_libraries(subscribe_pose ${catkin_LIBRARIES})
```
Save all the files.
``` 
cd ~/catkin-ws/
source devel/setup.bash
catkin_make
```
```roscore``` in a seperate terminal<br>
```rosrun turtlesim turtlesim_node``` in another terminal and finally 
```
rosrun agitr subscribe_pose
```
in another terminal obvi rofl, you can also run the publisher 
```
rosrun agitr randomv
```
in another terminal :)

<b>Errors faced</b><br>
1. Initially used poseMessageReceive() instead of a pointer: Basically we use pointer because we only want the address of the function and not actually want to call the function!

We can also use poseMessageReceive without the ```&``` -> compiler will interpret it as a pointer itself!

<b>The question of whether to use ros::spinOnce() or ros::spin() comes down to this: Does your program have any repetitive work to do, other than responding to callbacks? If the
answer is “No,” then use ros::spin(). If the answer is “Yes,” then a reasonable option is to
write a loop that does that other work and calls ros::spinOnce() periodically to process
callbacks!</b>

<hr>

# ROS 4: Logging

There are 5 levels of logging in order of increasing importance ->

1. DEBUG - reading header from buffer
2. INFO - Waiting for all connections to establish
3. WARN - Less than 5GB of space free on disk
4. ERROR - Publisher header did not have required element: type
5. FATAL - You must call ros::init() before creating the first NodeHandle

```
ROS_DEBUG_STREAM(message);
ROS_INFO_STREAM(message);
ROS_WARN_STREAM(message);
ROS_ERROR_STREAM(message);
ROS_FATAL_STREAM(message);
```
ROS also provides shorthand macros that generate precisely these sorts of one-time only log messages.
```
ROS_DEBUG_STREAM_ONCE(message);
ROS_INFO_STREAM_ONCE(message);
ROS_WARN_STREAM_ONCE(message);
ROS_ERROR_STREAM_ONCE(message);
ROS_FATAL_STREAM_ONCE(message);
```

The interval parameter is a double that specifies the minimum amount of time, measured in seconds, that must pass between successive instances of the given log message
```
ROS_DEBUG_STREAM_THROTTLE(interval, message);
ROS_INFO_STREAM_THROTTLE(interval, message);
ROS_WARN_STREAM_THROTTLE(interval, message);
ROS_ERROR_STREAM_THROTTLE(interval, message);
ROS_FATAL_STREAM_THROTTLE(interval, message);
```
### These log messages are actually in 3 different destinations, namely -> output on console, as a message on rosout topic and as an entry in a log file
1. We essentially publish logs to /rosout topic because all the log messages irrespective of its nodes, subscribers, locations all of them are logged into rosout! We can view this using 
```
rostopic echo /rosout (or)
rqt_console
```
>rqt_console actually subscribes to rosout_agg = rosout aggregated -> basically rosout aggregated subscribes to rosout to streamline the publisher scubscriber to one topic instead of many nodes to rosout.

2. The third and final destination for log messages is a log file generated by the rosout node.
As part of its callback function for the /rosout topic, this node writes a line to a file with a
name like this:
```∼/.ros/log/run_id/rosout.log```<br>
To learn the run_id(generated from MAC address and the current time):
```rorparam get /run_id```

<hr>

# ROS 5: Graph Resource Names

## These are essentially names of topics, nodes, services and pararmeters, and how ROS resolves this realtive naming system
> default_namespace + relative_name = globalname <br> 
/turtle1 + cmd_vel = /turtle1/cmd_vel

Private Names - begin with tilde symbol -> The difference is that, instead of using the current default namespace, private
names use the name of their node as a namespace

Anonymous Names - They are used specifically used to name nodes. The purpose
of an anonymous name is to make it easier to obey the rule that each node must have a
unique name.
```
ros::init(argc, argv, base_name, ros::init_options::AnonymousName);
```
>This is to ensure that the name of the node is unique

Output the name on console -
```
ROS_INFO_STREAM( "This message is from "
<<ros::this_node::getName());
```

# ROS 6: Launch Files:

Instead of the need to start so many nodes at once in seperate terminals, there is a solution for it using Launch Files. We can configure the file to start the rosmaster, nodes, etc.

## Remapping - 

To remap names within a launch fie, use a remap element in the node attribute - 
```
<node pkg="turtlesim" type="turtlesim_node" name="turtlesim" >
    <remap from="turtle1/pose" to="tim" />
</node>
```

### ERRORS
1. So basically in the [example.launch](launch/example.launch)
 for pkg = "agitr" I had initially put the name of the file -> subscriber.cpp, it doesn't work because it expects an executable-file for that cpp.
2. Use the executable-name used in the [CMake](CMakeLists.txt), i.e, subscribe_pose
3. The name can be anything as it will replace the base_name defined in the <i>ros::init();</i>

## Program that subscribes to turtle1/cmd_vel and republishes on turtle1/cmd_vel_reversed

1. Write the [program](src/reverse_cmd_vel.cpp)
2. To include the contents of another launch file, including all of its nodes and parameters, use an <i>include</i> element

```
<include file="$(find package-name)/launch-file-name" />
```

<hr>

# ROS 7: Pararmeters

The idea is that a centralized parameter
server keeps track of a collection of values—things like integers, floating point numbers,
strings, or other data—each identified by a short string name.<br>
The parameter is actually a part of the master, so it is started automatically when roscore and roslaunch.<br>
To get information about the parameters use ->
```
rosparam get /
```

## For example, changing the background of the turtlesim
```
rosrun turtlesim turtlesim_node
rosparam get /turtlesim/background_b
rosparam get /turtlesim/background_g
rosparam get /turtlesim/background_r
rosparam set /turtlesim/background_b 255
rosparam set /turtlesim/background_g 245
rosparam set /turtlesim/background_r 0
```
Even after setting these parameters, the background color remains the same
The explanation is that turtlesim_node only reads the values of these parameters
when its /clear service is called.
```
rosservice call /clear
```

## Accessing parameters from C++
The C++ interface to ROS parameters is quite straightforward:
void ros::param::set(parameter_name, input_value);
bool ros::param::get(parameter_name, output_value);

<hr>

# ROS 8: Services

Alternate method of communication apart from messages are <i>service calls</i>. It eliminates the limitations that <i>messages</i> have.