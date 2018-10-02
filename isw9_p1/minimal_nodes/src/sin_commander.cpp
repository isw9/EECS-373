// sin_commander node:
// wsn example node that both subscribes and publishes
// does somewhat more complex calculations that minimal_controller to update sinusoidal velocity
// given F specified on topic "force_cmd"
// publishes velocity on topic "velocity"
#include<ros/ros.h>
#include<std_msgs/Float64.h>
#include<iostream>
#include<cmath>
#include<math.h>
using namespace std;
std_msgs::Float64 g_velocity;
std_msgs::Float64 g_force;
double mass = 1.0;
double dt = 0.001;

void myCallback(const std_msgs::Float64& message_holder) {
   ROS_INFO("received force velocity value is: %f", message_holder.data);
   g_force.data = message_holder.data + (g_velocity.data / mass) * dt;
   g_force.data = (g_force.data * -1);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "sin_commander"); //name this node
    // when this compiled code is run, ROS will recognize it as a node called "sin_commander"
    ros::NodeHandle nh; // node handle
    //create a Subscriber object and have it subscribe to the topic "force_cmd"
    ros::Subscriber my_subscriber_object = nh.subscribe("force_cmd", 1, myCallback);
    //simulate accelerations and publish the resulting velocity;
    //need two publisher objects because we will be publishing to two different topics
    ros::Publisher my_publisher_object = nh.advertise<std_msgs::Float64>("velocity", 1);
    ros::Publisher my_publisher_object_two =  nh.advertise<std_msgs::Float64>("vel_cmd", 1);
    double amplitude;
    double frequency;

    // prompt the user for input
    cout << "Enter an amplitude";
    cin >> amplitude;
    cout << "Enter a frequency";
    cin >> frequency;

    double time_elapsed = 0.0;
    double sample_rate = 1.0 / dt; // compute the corresponding update frequency
    ros::Rate naptime(sample_rate);
    g_velocity.data = 0.0; //initialize velocity to zero
    g_force.data = 0.0; // initialize force to 0; will get updated by callback
    while (ros::ok()) {
      	//sinusoidal velocity calculation
        double x = amplitude * cos(2 * 3.14 * frequency * time_elapsed);
        g_velocity.data = 2 * 3.14 * frequency * sqrt((amplitude * amplitude) - (x * x));
        my_publisher_object.publish(g_velocity); // publish the system state
	      my_publisher_object_two.publish(g_force);
        ROS_INFO("commanded velocity = %f", g_velocity.data);
        ros::spinOnce(); //allow data update from callback
        naptime.sleep();
        time_elapsed = time_elapsed + sample_rate;

    }
    return 0; // should never get here, unless roscore dies
}
