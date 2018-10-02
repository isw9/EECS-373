// P3 Action Server
// ISAAC WITHROW

#include<ros/ros.h>
#include<math.h>
#include <actionlib/server/simple_action_server.h>
//the following #include refers to the "action" message defined for this package
// The action message can be found in: .../example_action_server/action/demo.action
// Automated header generation creates multiple headers for message I/O
// These are referred to by the root name (demo) and appended name (Action)
#include<example_action_server/P3Action.h>

int countdown_val_ = 0;

class ExampleActionServer {
private:

    ros::NodeHandle nh_;  // we'll need a node handle; get one upon instantiation

    // this class will own a "SimpleActionServer" called "as_".
    // it will communicate using messages defined in example_action_server/action/demo.action
    // the type "demoAction" is auto-generated from our name "demo" and generic name "Action"
    actionlib::SimpleActionServer<example_action_server::P3Action> as_;
    
    // here are some message types to communicate with our client(s)
    example_action_server::P3Goal goal_; // goal message, received from client
    example_action_server::P3Result result_; // put results here, to be sent back to the client when done w/ goal



public:
    ExampleActionServer(); //define the body of the constructor outside of class definition

    ~ExampleActionServer(void) {
    }
    // Action Interface
    void executeCB(const actionlib::SimpleActionServer<example_action_server::P3Action>::GoalConstPtr& goal);
};

//implementation of the constructor:
// member initialization list describes how to initialize member as_
// member as_ will get instantiated with specified node-handle, name by which this server will be known,
//  a pointer to the function to be executed upon receipt of a goal.
//  
// Syntax of naming the function to be invoked: get a pointer to the function, called executeCB, which is a member method
// of our class exampleActionServer.  Since this is a class method, we need to tell boost::bind that it is a class member,
// using the "this" keyword.  the _1 argument says that our executeCB takes one argument
// the final argument  "false" says don't start the server yet.  (We'll do this in the constructor)

ExampleActionServer::ExampleActionServer() :
   as_(nh_, "demo_action_server_node", boost::bind(&ExampleActionServer::executeCB, this, _1),false) 
// in the above initialization, we name the server "example_action"
//  clients will need to refer to this name to connect with this server
{
    ROS_INFO("in constructor of exampleActionServer...");
    // do any other desired initializations here...specific to your implementation

    as_.start(); //start the server running
}

void ExampleActionServer::executeCB(const actionlib::SimpleActionServer<example_action_server::P3Action>::GoalConstPtr& goal) {
    result_.success = false; // we'll use the member variable result_, defined in our class
    ros::Rate timer(1.0);
    // the class owns the action server, so we can use its member methods here
     countdown_val_ = goal->num_cycles;
     double amplitude_ = goal->amplitude.data;
     double frequency_ = goal->frequency.data;

    double time_elapsed = 0.0;
    //implement a simple timer, which counts down from provided countdown_val to 0, in seconds
     double velocity = 0;
    while (countdown_val_>0) {
      double x = amplitude_ * cos(2 * 3.14 * frequency_ * time_elapsed); 
      result_.velocity.data = 2 * 3.14 * frequency_ * sqrt((amplitude_ * amplitude_) - (x * x));
      ROS_INFO("velocity is: %f", result_.velocity.data);
      countdown_val_--;
      timer.sleep();
      time_elapsed = time_elapsed + 1.0;
    }
    result_.success = true; 
    as_.setSucceeded(result_);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "demo_action_server_node"); // name this node 

    ROS_INFO("instantiating the demo action server: ");

    ExampleActionServer as_object; // create an instance of the class "ExampleActionServer"
    
    ROS_INFO("going into spin");
    // from here, all the work is done in the action server, with the interesting stuff done within "executeCB()"
    // you will see 5 new topics under example_action: cancel, feedback, goal, result, status
    while (ros::ok()) {
        ros::spinOnce(); //normally, can simply do: ros::spin();  
    }

    return 0;
}
