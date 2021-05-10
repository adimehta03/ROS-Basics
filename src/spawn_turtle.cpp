#include<ros/ros.h>
#include<turtlesim/Spawn.h>

int main(int argc, char **argv){
    ros::init(argc, argv, "spawn_turtle");
    ros::NodeHandle nh;

    ros::ServiceClient spawnClient = nh.serviceClient<turtlesim::Spawn>("spawn");
    //In this we dont use queue because only  if we get a request we send the resonse


    turtlesim::Spawn::Request req;
    turtlesim::Spawn::Response resp; 

    //turtlesim::Spawn(package_name::service_type) is actually a struct ->
    // Response and Request are data members. An object from this struct is called as a srv
    // typedef struct turtlesim::Spawn{
    //     Request <obj_name>;
    //     Response <obj_name>;
    // } soething like this


    req.x = 2; req.y = 3;
    req.theta = M_PI/2;
    req.name = "Leo";

    bool success = spawnClient.call(req,resp); //locating the server node, transmitting the request data, waiting for a response and storing the response data

    if(success){
        ROS_INFO_STREAM("Spawned a turtle named "<<resp.name);
    } else{
        ROS_ERROR_STREAM("Failed to spawn.");
    }
}
