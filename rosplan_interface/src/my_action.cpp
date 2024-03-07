#include "rosplan_interface/my_action.h" 
#include <unistd.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

#include <move_base_msgs/MoveBaseAction.h>

namespace KCL_rosplan { 	 	
		// Constructor for MyActionInterface class
	MyActionInterface::MyActionInterface(ros::NodeHandle &nh) {
	
	}
	// Callback function for the action dispatch
	bool MyActionInterface::concreteCallback(const
	rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg) {
		// here the implementation of the action
		std::cout << "Going from " << msg->parameters[0].value << " to " << msg->parameters[1].value << std::endl;
		actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base", true);

		// Waiting for the action client to be ready and creating a MoveBaseGoal message

		ac.waitForServer();
		move_base_msgs::MoveBaseGoal goal;
		          
            	goal.target_pose.header.frame_id = "map";
            	
            	// Determining the goal based on the parameters received
            	
		if(msg->parameters[2].value == "init"){
			std::cout << "Going to init" << std::endl;
			goal.target_pose.pose.position.x = 0.0;
			goal.target_pose.pose.position.y = 2.0;
			goal.target_pose.pose.orientation.w = 1.0;
		}
		else if (msg->parameters[2].value == "m11"){
			std::cout << "Going to m11" << std::endl;
			goal.target_pose.pose.position.x = 6.0;
			goal.target_pose.pose.position.y = 2.0;
			goal.target_pose.pose.orientation.w = 1.0;
		}
		else if (msg->parameters[2].value == "m12"){
			std::cout << "Going to m12" << std::endl;
			goal.target_pose.pose.position.x = 7.0;
			goal.target_pose.pose.position.y = -5.0;
			goal.target_pose.pose.orientation.w = 1.0;
		}
		else if (msg->parameters[2].value == "m13"){
			std::cout << "Going to m13" << std::endl;
			goal.target_pose.pose.position.x = -3.0;
			goal.target_pose.pose.position.y = -8.0;
			goal.target_pose.pose.orientation.w = 1.0;
		}
		else if (msg->parameters[2].value == "m15"){
			std::cout << "Going to m15" << std::endl;
			goal.target_pose.pose.position.x = -7.0;
			goal.target_pose.pose.position.y = -1.5;
			goal.target_pose.pose.orientation.w = 1.0;
		}
		std::cout << "Before send goal" << std::endl;
		// Sending the goal to the action server
		ac.sendGoal(goal);
		std::cout << "before wait" << std::endl;
		// Waiting for the action result
		ac.waitForResult();	
		
		ROS_INFO("Action (%s) performed: completed!", msg->name.c_str());
		return true;
	}
}

int main(int argc, char **argv) {
	// Initializing ROS node and creating an instance of MyActionInterface class
	ros::init(argc, argv, "my_rosplan_action", ros::init_options::AnonymousName);
	ros::NodeHandle nh("~");
	KCL_rosplan::MyActionInterface my_aci(nh);
	my_aci.runActionInterface();
	return 0;
}








