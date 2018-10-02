// ISAAC WITHROW
// P2 ROS SERVICE CLIENT

#include <ros/ros.h>
#include <example_ros_service/ClientP2.h> // this message type is defined in the current package
#include <iostream>
#include <std_msgs/Float64.h>
#include <string>
using namespace std;

int main(int argc, char **argv) {
    ros::init(argc, argv, "ros_service_client_p2");
    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<example_ros_service::ClientP2>("frequency_and_amplitude");
    example_ros_service::ClientP2 srv;
    double in_amplitude;
    double in_frequency;
    while (ros::ok()) {
    // prompt the user for input
    cout<<endl;
    cout << "Enter an amplitude: ";
    cin >> in_amplitude;
    srv.request.amplitude.data = in_amplitude;

    cout<<endl;
    cout << "Enter a frequency: ";
    cin >> in_frequency;
    srv.request.frequency.data = in_frequency;
        if (client.call(srv)) {
            if (srv.response.received == true) {
	        ROS_INFO("The service successfully received the amplitude and frequency");
            } else {
	      
            }

        } else {
            ROS_ERROR("Failed to call service");
            return 1;
        }
    }
    return 0;
}
