//unload_box_v4.cpp:
// try to fill a single order, using both inspection  stations

//use a RobotBehaviorInterface object to communicate with the robot behavior action server
#include <robot_behavior_interface/RobotBehaviorInterface.h>

//we define a message type for a "part" that includes name, pose and associated location code (e.g. bin # or box location)
#include<inventory_msgs/Part.h>

//a "box inspector" object can compare a packing list to a logical camera image to see how we are doing
#include<box_inspector/box_inspector2.h>

//conveyor interface communicates with the conveyor action server
#include<conveyor_as/ConveyorInterface.h>

#include<bin_inventory/bin_inventory.h>


const double COMPETITION_TIMEOUT = 500.0; // need to  know what this is for the finals;
// want to ship out partial credit before time runs out!

osrf_gear::Order g_order;
bool g_got_order = false;

void model_to_part(osrf_gear::Model model, inventory_msgs::Part &part, unsigned short int location) {
    part.name = model.type;
    part.pose.pose = model.pose;
    part.location = location; //by default
}

// Listening for the Orders from ARIAC

void orderCallback(const osrf_gear::Order::ConstPtr& msg) {

    g_order = *msg;
    g_got_order = true;
    ROS_INFO("Received order %s with %i shipment%s", msg->order_id.c_str(), (int) msg->shipments.size(), msg->shipments.size() == 1 ? "" : "s");
    ROS_INFO_STREAM(g_order);
}

int main(int argc, char** argv) {
    // ROS set-ups:
    ros::init(argc, argv, "box_unloader"); //node name
    ros::NodeHandle nh; // create a node handle; need to pass this to the class constructor
    int ans;
    unsigned char box_location_code = inventory_msgs::Part::QUALITY_SENSOR_1;

    ros::ServiceClient startup_client = nh.serviceClient<std_srvs::Trigger>("/ariac/start_competition");
    std_srvs::Trigger startup_srv;

    ROS_INFO("instantiating a RobotBehaviorInterface");
    RobotBehaviorInterface robotBehaviorInterface(&nh); //shipmentFiller owns one as well

    ROS_INFO("instantiating a ConveyorInterface");
    ConveyorInterface conveyorInterface(&nh);

    ROS_INFO("instantiating a BoxInspector");
    BoxInspector2 boxInspector(&nh);

    ROS_INFO("instantiating a binInventory object");
    BinInventory binInventory(&nh);
    inventory_msgs::Inventory current_inventory;

    ros::ServiceClient drone_client = nh.serviceClient<osrf_gear::DroneControl>("/ariac/drone");
    osrf_gear::DroneControl droneControl;

    //instantiate an object of appropriate data type for our move-part commands
    inventory_msgs::Part current_part, desired_part;

    geometry_msgs::PoseStamped box_pose_wrt_world; //camera sees box, coordinates are converted to world coords

    bool status;
    int nparts;
    while (!startup_srv.response.success) {
      ROS_WARN("NOT SUCCESSFUL STARTING UP COMPETITION YET");
      startup_client.call(startup_srv);
      ros::Duration(0.5).sleep();
    }
    ROS_INFO("Successfully started competition");
    ROS_WARN("SHOULD START THE COMPETITION HERE...");

    ros::Subscriber sub = nh.subscribe("ariac/orders", 5, orderCallback);
    ROS_INFO("waiting for order...");
    while (!g_got_order) {
        ros::spinOnce();
        ros::Duration(0.5).sleep();
        if (!g_got_order) ROS_INFO("waiting");

    }

    ROS_WARN("Successfully got order");
    //for box inspector, need to define multiple vectors for args, 
    //box inspector will identify parts and convert their coords to world frame
    //in the present example, desired_models_wrt_world is left empty, so ALL observed parts will be considered "orphaned"
    vector<osrf_gear::Model> desired_models_wrt_world;
    vector<osrf_gear::Model> satisfied_models_wrt_world;
    vector<osrf_gear::Model> misplaced_models_actual_coords_wrt_world;
    vector<osrf_gear::Model> misplaced_models_desired_coords_wrt_world;
    vector<osrf_gear::Model> missing_models_wrt_world;
    vector<osrf_gear::Model> orphan_models_wrt_world;
    vector<int> part_indices_missing;
    vector<int> part_indices_misplaced;
    vector<int> part_indices_precisely_placed;


    //use conveyor action  server for multi-tasking
    ROS_INFO("getting a box into position: ");
    int nprint = 0;
    conveyorInterface.move_new_box_to_Q1(); //member function of conveyor interface to move a box to inspection station 1
    while (conveyorInterface.get_box_status() != conveyor_as::conveyorResult::BOX_SEEN_AT_Q1) {
        ros::spinOnce();
        ros::Duration(0.1).sleep();
        nprint++;
        if (nprint % 10 == 0) {
            ROS_INFO("waiting for conveyor to advance a box to Q1...");
        }
    }

    for (int i = 0; i < 2; i++) {
      if (i == 0) {
        box_location_code = inventory_msgs::Part::QUALITY_SENSOR_1;
      }
      else {
        box_location_code = inventory_msgs::Part::QUALITY_SENSOR_2;
      }

      int cam = i + 1;

      if (boxInspector.get_box_pose_wrt_world(box_pose_wrt_world, cam)) {
        ROS_INFO_STREAM("box seen at: " << box_pose_wrt_world << endl);
      } else {
        ROS_WARN("no box seen.  something is wrong! I quit!!");
        exit(1);
      }

      if (i == 1) {
	cout << "Enter 1 after you have manually moved a gear part ";
	cin>>ans;
      }

      boxInspector.compute_shipment_poses_wrt_world(g_order.shipments[0], box_pose_wrt_world, desired_models_wrt_world);

      boxInspector.update_inspection(desired_models_wrt_world,
            satisfied_models_wrt_world, misplaced_models_actual_coords_wrt_world,
            misplaced_models_desired_coords_wrt_world, missing_models_wrt_world,
            orphan_models_wrt_world, part_indices_missing, part_indices_misplaced,
            part_indices_precisely_placed, cam);

      nparts = orphan_models_wrt_world.size();
      ROS_INFO("num orphaned parts seen = %d", nparts);
      for (int i = 0; i < nparts; i++) {
        ROS_INFO_STREAM("orphaned  parts: " << orphan_models_wrt_world[i] << endl);
      }

      if (boxInspector.get_bad_part_Q(current_part, cam)) {
        ROS_INFO("found bad part: ");
        ROS_INFO_STREAM(current_part << endl);

	// need to find the coordinates of the current part by checking which of the
	// orphaned models has coordinates close to the current part's coordinates
	for (int i = 0; i < nparts; i++) {
	double x_difference = abs(orphan_models_wrt_world[i].pose.position.x - current_part.pose.pose.position.x);
	double y_difference = abs(orphan_models_wrt_world[i].pose.position.y - current_part.pose.pose.position.y);
	//  model_to_part(orphan_models_wrt_world[i], current_part, inventory_msgs::Part::QUALITY_SENSOR_1);
	  if (x_difference < 0.01 && y_difference < 0.01) {
	      current_part.name = orphan_models_wrt_world[i].type;
              robotBehaviorInterface.pick_part_from_box(current_part);
	      robotBehaviorInterface.discard_grasped_part(current_part);
	  }
	}
      }

      ROS_INFO("done removing orphans; attempt part relocations, as necessary");
      boxInspector.update_inspection(desired_models_wrt_world,
            satisfied_models_wrt_world, misplaced_models_actual_coords_wrt_world,
            misplaced_models_desired_coords_wrt_world, missing_models_wrt_world,
            orphan_models_wrt_world, part_indices_missing, part_indices_misplaced,
            part_indices_precisely_placed, cam);


      int nparts_misplaced = misplaced_models_actual_coords_wrt_world.size();
      ROS_INFO("found %d misplaced parts", nparts_misplaced);
      bool go_on = false;


      while (nparts_misplaced > 0) {
       // move all of the parts that are not in the correct place
	for (int i = 0; i < nparts_misplaced; i++) {
	  model_to_part(misplaced_models_actual_coords_wrt_world[i], current_part, box_location_code);
	  int index_des_part = part_indices_misplaced[i];
	  model_to_part(desired_models_wrt_world[index_des_part], desired_part, box_location_code);
	  ROS_INFO("move part from: ");
	  ROS_INFO_STREAM(current_part);
	  ROS_INFO("move part to: ");
	  ROS_INFO_STREAM(desired_part);
	  //use the robot action server to grasp part in the box:
	  status = robotBehaviorInterface.pick_part_from_box(current_part);

	  //following fnc works ONLY if part is already grasped:
	  status = robotBehaviorInterface.adjust_part_location_no_release(current_part, desired_part);
	  status = robotBehaviorInterface.release_and_retract();
	}

	boxInspector.update_inspection(desired_models_wrt_world,
            satisfied_models_wrt_world, misplaced_models_actual_coords_wrt_world,
            misplaced_models_desired_coords_wrt_world, missing_models_wrt_world,
            orphan_models_wrt_world, part_indices_missing, part_indices_misplaced,
            part_indices_precisely_placed, cam);
	nparts_misplaced = misplaced_models_actual_coords_wrt_world.size();
      }

      int number_of_missing_parts = part_indices_missing.size();
      ROS_WARN("found %d missing parts", number_of_missing_parts);
      for (int i = 0; i < number_of_missing_parts; i++) {
	model_to_part(desired_models_wrt_world[part_indices_missing[i]], current_part, inventory_msgs::Part::QUALITY_SENSOR_1);
	std::string part_name(desired_models_wrt_world[part_indices_missing[i]].type);
	ROS_INFO_STREAM("looking for part " << part_name << endl);
	int partnum_in_inventory;
	bool part_in_inventory = true;
	inventory_msgs::Part pick_part, place_part;
	binInventory.update();
	binInventory.get_inventory(current_inventory);
	part_in_inventory = binInventory.find_part(current_inventory, part_name, pick_part, partnum_in_inventory);

	if (!part_in_inventory) {
	  ROS_WARN("could not find desired  part in inventory; giving up on process_part()");
	  return false; 
	}
	ROS_INFO_STREAM("found part: " << pick_part << endl);
	//specify place part:
	model_to_part(desired_models_wrt_world[part_indices_missing[i]], place_part, inventory_msgs::Part::QUALITY_SENSOR_1);
	go_on = robotBehaviorInterface.evaluate_key_pick_and_place_poses(pick_part, place_part);
	if (!go_on) {
	  ROS_WARN("could not compute key pickup and place poses for this part source and destination");
	}
	ROS_INFO("attempting pick...");
	ROS_INFO("attempting to pick part");
	if (!robotBehaviorInterface.pick_part_from_bin(pick_part)) {
	  ROS_INFO("pick failed");
	  go_on = false;
	  return false;   
	}
	ROS_INFO("moving to approach pose");
	if (!robotBehaviorInterface.move_part_to_approach_pose(place_part)) {
	  ROS_WARN("could not move to approach pose");
	  go_on = false;
	  robotBehaviorInterface.discard_grasped_part(place_part);     
	}
	//place part:
	ROS_INFO("attempting to place part");

	if (!robotBehaviorInterface.place_part_in_box_no_release(place_part)) {
	  ROS_INFO("placement failed");
	  go_on = false;
	  return false;
	}

	status = robotBehaviorInterface.release_and_retract();
      }

      if (i == 0) {
	//advance the box further!
	conveyorInterface.move_box_Q1_to_Q2(); //member function of conveyor interface to move a box to inspection station 1
	while (conveyorInterface.get_box_status() != conveyor_as::conveyorResult::BOX_SEEN_AT_Q2) {
	  ros::spinOnce();
	  ros::Duration(0.1).sleep();
	  nprint++;
	  if (nprint % 10 == 0) {
            ROS_INFO("waiting for conveyor to advance a box to Q2...");
	  }
	}
      }
      else {
	ROS_INFO("advancing box to loading dock for shipment");
	conveyorInterface.move_box_Q2_to_drone_depot(); //member function of conveyor interface to move a box to shipping dock

	while (conveyorInterface.get_box_status() != conveyor_as::conveyorResult::BOX_SENSED_AT_DRONE_DEPOT) {
	  ros::spinOnce();
	  ros::Duration(0.1).sleep();
	  nprint++;
	  if (nprint % 10 == 0) {
            ROS_INFO("waiting for conveyor to advance a box to loading dock...");
	  }
	}
      }
    }

    
    ROS_INFO("calling drone");
    ROS_WARN("FIX THE LABEL");
    
    droneControl.request.shipment_type = g_order.shipments[0].shipment_type;
    ROS_INFO_STREAM("shipment name: " << g_order.shipments[0].shipment_type << endl);

    droneControl.response.success = false;
    while (!droneControl.response.success) {
        drone_client.call(droneControl);
        ros::Duration(0.5).sleep();
    }

    ROS_INFO("all done; goodbye, world");
    return 0;
    //here's an oddity: this node runs to completion.  But sometimes, Linux complains bitterly about
    // *** Error in `/home/wyatt/ros_ws/devel/lib/shipment_filler/unload_box': corrupted size vs. prev_size: 0x000000000227c7c0 ***
    // don't know why.  But does not seem to matter.  If anyone figures this  out, please let me know.
}
