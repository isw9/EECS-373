//ps6 Isaac Withrow
#include <ros/ros.h>
#include <std_srvs/Trigger.h>
#include <osrf_gear/ConveyorBeltControl.h>
#include <osrf_gear/DroneControl.h>
#include <osrf_gear/LogicalCameraImage.h>
#include <iostream>
#include <string>
#include <math.h>
using namespace std;
bool g_take_new_snapshot = false;
osrf_gear::LogicalCameraImage g_cam2_data;

void camera_twoCB(const osrf_gear::LogicalCameraImage& message_holder)
{
  if (g_take_new_snapshot && message_holder.models.size() > 0) {
    g_cam2_data = message_holder;
    g_take_new_snapshot = false;
    }
  else {
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
    ros::ServiceClient drone_client = n.serviceClient<osrf_gear::DroneControl>("/ariac/drone");
    osrf_gear::DroneControl drone_srv;

    ros::Subscriber cam2_subscriber = n.subscribe("/ariac/logical_camera_2",1,camera_twoCB);

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

    g_take_new_snapshot = true;
    while (g_cam2_data.models[0].pose.position.z < abs(0.01) || g_cam2_data.models[0].pose.position.z < 0) {
      ros::spinOnce();
      ros::Duration(0.3).sleep();
      g_take_new_snapshot = true;
    }
    ROS_INFO ("z-value is %f", g_cam2_data.models[0].pose.position.z); 


    // Now we must stop the conveyor belt
    conveyor_srv.request.power = 0.0;
    conveyor_srv.response.success = false;
    conveyor_client.call(conveyor_srv);

    while (!conveyor_srv.response.success) {
      ROS_WARN("NOT SUCCESSFUL STOPPING CONVEYOR BELT");
      conveyor_client.call(conveyor_srv);
      ros::Duration(0.5).sleep();
    }
    ROS_INFO("Successfully stopped conveyor belt, sleeping for 5 seconds");

    ros::Duration(5.0).sleep();

    // Now we must restart  the conveyor belt
    conveyor_srv.request.power = 100.0;
    conveyor_srv.response.success = false;
    conveyor_client.call(conveyor_srv);

    while (!conveyor_srv.response.success) {
      ROS_WARN("NOT SUCCESSFUL RESTARTING CONVEYOR BELT");
      conveyor_client.call(conveyor_srv);
      ros::Duration(0.5).sleep();
    }
    ROS_INFO("Successfully restarted conveyor belt");
    
    //restart conveyor belt
    //


    // Now we must call the drone
    drone_srv.request.shipment_type = "dummy";
    drone_srv.response.success = false;
    drone_client.call(drone_srv);

    while (!drone_srv.response.success) {
      ROS_WARN("NOT SUCCESSFUL STARTING DRONE");
      drone_client.call(drone_srv);
      ros::Duration(1.5).sleep();
    }
    ROS_INFO("Successfully started drone");

    ros::spin();
    return 0;
}
