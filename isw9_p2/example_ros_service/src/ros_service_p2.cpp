// ROS Service for P2
// ISAAC WITHROW


#include <ros/ros.h>
#include <example_ros_service/ClientP2.h>
#include <iostream>
#include <string>
using namespace std;

bool callback(example_ros_service::ClientP2Request& request, example_ros_service::ClientP2Response& response)
{
    ROS_INFO("callback activated");
    response.received = false;
    if (request.amplitude.data > 0 && request.frequency.data > 0)
 {
        ROS_INFO("Service has received request");
	ROS_INFO("Requested amplitude = %f", request.amplitude.data);
	ROS_INFO("Requested frequency = %f", request.frequency.data);
        response.received = true;
    }
    else {
      ROS_INFO("The requested amplitude or frequency was less than zero");
    }
  
    
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ros_service_p2");
  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService("frequency_and_amplitude", callback);
  ros::spin();

  return 0;
}
