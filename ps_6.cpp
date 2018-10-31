//ps6 Isaac Withrow
#include <ros/ros.h>
#include <std_srvs/Trigger.h>
#include <osrf_gear/ConveyorBeltControl.h>
#include <osrf_gear/DroneControl.h>
#include <osrf_gear/LogicalCameraImage.h>
#include <iostream>
#include <string>
using namespace std;
bool g_take_new_snapshot = false;
osrf_gear::LogicalCameraImage g_cam2_data;
//rosservice call /ariac/conveyor/control
// rosservice call /ariac/drone
void camera_twoCB(const osrf_gear::LogicalCameraImage& message_holder)
{
  if (g_take_new_snapshot) {
    ROS_INFO_STREAM("image from camera2: " << message_holder<<endl);
    g_cam2_data = message_holder;
    g_take_new_snapshot = false;
  }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "ps_6");
    ros::NodeHandle n;

    // setup srvs and clients for service communications
    ros::ServiceClient startup_client = n.serviceClient<std_srvs::Trigger>("/ariac/start_competition");
    std_srvs::Trigger startup_srv;
    ros::ServiceClient conveyor_client = n.serviceClient<osrf_gear::ConveyorBeltControl>("/ariac/conveyor/control");
    osrf_gear::ConveyorBeltControl conveyor_srv;
    ros::ServiceClient drone_client = n.serviceClient<osrf_gear::DrontControl>("/ariac/drone");
    osrf_gear::DrontControl drone_srv;

    ros::Subscriber cam2_subscriber = n.subscribe("/ariac/logical_camera_two",1,camera_twoCB);

    ros::spin();

    drone_srv.request.shipment_type = "dummy";

    // We must start the competition before we start the conveyor belt
    startup_srv.response.success = false;
    startup_client.call(startup_srv);

    while (!startup_srv.response.success) {
      ROS_WARN("NOT SUCCESSFUL STARTING UP COMPETITION YET");
      startup_client.call(startup_srv);
      ros::Duration(0.5).sleep();
    }
    ROS_INFO("Successfully started competition");

    // Now we must start the conveyor belt
    conveyor_srv.request.power = 100.0;
    conveyor_srv.response.success = false;
    conveyor_client.call(conveyor_srv);

    while (!conveyor_srv.response.success) {
      ROS_WARN("NOT SUCCESSFUL STARTING CONVEYOR BELT");
      conveyor_client.call(conveyor_srv);
      ros::Duration(0.5).sleep();
    }
    ROS_INFO("Successfully started conveyor belt");

    g_take_new_snapshot = true;
    while (g_cam2_data.models.size() < 1) {
      ros::spinOnce();
      ros::Duration(0.5).sleep();
      g_take_new_snapshot = true;
    }
    ROS_INFO("I SEE A BOX");

    // now check coordinates of this box

    // MONITOR LOGICAL CAMERA 2
    // STOP CONVEYOR WHEN BOX'S Z POSITION = 0
    // WAIT FOR 5 SECONDS
    // RESTART  conveyor belt
    //


    // Now we must call the drone
    drone_srv.request.shipment_type = "dummy";
    drone_srv.response.success = false;
    drone_client.call(drone_srv);

    while (!drone_srv.response.success) {
      ROS_WARN("NOT SUCCESSFUL STARTING DRONE");
      drone_client.call(drone_srv);
      ros::Duration(0.5).sleep();
    }
    ROS_INFO("Successfully started drone");

    return 0;
}
