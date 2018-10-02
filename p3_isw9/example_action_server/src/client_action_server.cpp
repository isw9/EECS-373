// ISAAC WITHROW
// SIN COMMANDER ACTION CLIENT
#include<ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <iostream>
#include <string>
#include <ctime>

#include<example_action_server/P3Action.h>
using namespace std;

// This function will be called once when the goal completes
// this is optional, but it is a convenient way to get access to the "result" message sent by the server
void doneCb(const actionlib::SimpleClientGoalState& state,
        const example_action_server::P3ResultConstPtr& result) {
        bool returnValue = result->success;
	double calculated_velocity = result->velocity.data;
	ROS_INFO("Calculated velocity is %f", calculated_velocity);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "client_action_server"); // name this node
    
    example_action_server::P3Goal goal;

    actionlib::SimpleActionClient<example_action_server::P3Action> action_client("demo_action_server_node", true);

    // attempt to connect to the server:
    ROS_INFO("waiting for server: ");
    bool server_exists = action_client.waitForServer(ros::Duration(5.0)); // wait for up to 5 seconds

    if (!server_exists) {
        ROS_WARN("could not connect to server; halting");
        return 0; // bail out; optionally, could print a warning message and retry
    }


    ROS_INFO("connected to action server"); // if here, then we connected to the server;
    double in_amplitude;
    double in_frequency;
    int in_cycle_count;
    while (true) {
      cout<<endl;
      cout << "Enter an amplitude: ";
      cin >> in_amplitude;

      cout<<endl;
      cout << "Enter a frequency: ";
      cin >> in_frequency;
      
      cout<<endl;
      cout << "Enter a cycle count: ";
      cin >> in_cycle_count;

      goal.amplitude.data = in_amplitude;
      goal.frequency.data = in_frequency;
      goal.num_cycles = in_cycle_count;

      time_t start = time(0);
      action_client.sendGoal(goal, &doneCb);
      
      bool finished_before_timeout = action_client.waitForResult(ros::Duration(1000.0));
       if (!finished_before_timeout) {
           ROS_WARN("giving up waiting on result");
           return 0;
       } else {
	 time_t end = time(0);
	 double time = difftime(end, start);
	 ROS_INFO("It took %f seconds for the server to return", time);
       }

    }

    return 0;
}

