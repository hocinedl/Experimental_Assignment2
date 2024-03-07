#include "rosplan_interface/my_action.h" 
#include <unistd.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
// #include <motion_plan/PlanningAction.h>
// #include <motion_plan/PlanningGoal.h>
#include <move_base_msgs/MoveBaseAction.h>

namespace KCL_rosplan {
	MyActionInterface::MyActionInterface(ros::NodeHandle &nh) {
	// here the initialization
	}

	bool MyActionInterface::concreteCallback(const
	rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg) {
		// here the implementation of the action
		std::cout << "Going from " << msg->parameters[0].value << " to " << msg->parameters[1].value << std::endl;
		actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base", true);
		
		 
		 // motion_plan::PlanningGoal goal;
		 
		
		ac.waitForServer();
		move_base_msgs::MoveBaseGoal goal;
		            // Set the frame of reference for the goal's target pose to "map".
            // This means the goal coordinates will be interpreted in the map's coordinate frame.
            	goal.target_pose.header.frame_id = "map";
		if(msg->parameters[1].value == "init"){
			std::cout << "Going to init" << std::endl;
			goal.target_pose.pose.position.x = 0.0;
			goal.target_pose.pose.position.y = 2.0;
			goal.target_pose.pose.orientation.w = 1.0;
		}
		else if (msg->parameters[1].value == "m11"){
			std::cout << "Going to m11" << std::endl;
			goal.target_pose.pose.position.x = 6.0;
			goal.target_pose.pose.position.y = 2.0;
			goal.target_pose.pose.orientation.w = 1.0;
		}
		else if (msg->parameters[1].value == "m12"){
			std::cout << "Going to m12" << std::endl;
			goal.target_pose.pose.position.x = 7.0;
			goal.target_pose.pose.position.y = -5.0;
			goal.target_pose.pose.orientation.w = 1.0;
		}
		else if (msg->parameters[1].value == "m13"){
			std::cout << "Going to m13" << std::endl;
			goal.target_pose.pose.position.x = -3.0;
			goal.target_pose.pose.position.y = -8.0;
			goal.target_pose.pose.orientation.w = 11.0;
		}
		else if (msg->parameters[1].value == "m15"){
			std::cout << "Going to m15" << std::endl;
			goal.target_pose.pose.position.x = -7.0;
			goal.target_pose.pose.position.y = -1.5;
			goal.target_pose.pose.orientation.w = 1.0;
		}
		std::cout << "Before send goal" << std::endl;
		ac.sendGoal(goal);
		std::cout << "before wait" << std::endl;
		ac.waitForResult();	
		
		ROS_INFO("Action (%s) performed: completed!", msg->name.c_str());
		return true;
	}
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "my_rosplan_action", ros::init_options::AnonymousName);
	ros::NodeHandle nh("~");
	KCL_rosplan::MyActionInterface my_aci(nh);
	my_aci.runActionInterface();
	return 0;
}





#include "rosplan_interface/my_action.h" 
#include <unistd.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <motion_plan/PlanningAction.h>
#include <motion_plan/PlanningGoal.h>
//#include <move_base_msgs/MoveBaseAction.h>

namespace KCL_rosplan {
	MyActionInterface::MyActionInterface(ros::NodeHandle &nh) {
	// here the initialization
	}

	bool MyActionInterface::concreteCallback(const
	rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg) {
		// here the implementation of the action
		std::cout << "Going from " << msg->parameters[0].value << " to " << msg->parameters[1].value << std::endl;
		actionlib::SimpleActionClient<motion_plan::PlanningAction> ac("reaching_goal", true);
		
		 
		 motion_plan::PlanningGoal goal;
		 
		
		ac.waitForServer();
		//move_base_msgs::MoveBaseGoal goal;
		            // Set the frame of reference for the goal's target pose to "map".
            // This means the goal coordinates will be interpreted in the map's coordinate frame.
            	//goal.target_pose.header.frame_id = "map";
		if(msg->parameters[1].value == "init"){
			std::cout << "Going to init" << std::endl;
			goal.target_pose.pose.position.x = 0.0;
			goal.target_pose.pose.position.y = 2.0;
			goal.target_pose.pose.orientation.w = 1.0;
		}
		else if (msg->parameters[1].value == "m11"){
			std::cout << "Going to" << msg->parameters[1].value << std::endl;
			goal.target_pose.pose.position.x = 6.0;
			goal.target_pose.pose.position.y = 2.0;
			goal.target_pose.pose.orientation.w = 1.0;
		}
		else if (msg->parameters[1].value == "m12"){
			std::cout << "Going to m12" << std::endl;
			goal.target_pose.pose.position.x = 7.0;
			goal.target_pose.pose.position.y = -5.0;
			goal.target_pose.pose.orientation.w = 1.0;
		}
		else if (msg->parameters[1].value == "m13"){
			std::cout << "Going to m13" << std::endl;
			goal.target_pose.pose.position.x = -3.0;
			goal.target_pose.pose.position.y = -8.0;
			goal.target_pose.pose.orientation.w = 11.0;
		}
		else if (msg->parameters[1].value == "m15"){
			std::cout << "Going to m15" << std::endl;
			goal.target_pose.pose.position.x = -7.0;
			goal.target_pose.pose.position.y = -1.5;
			goal.target_pose.pose.orientation.w = 1.0;
		}
		std::cout << "Before send goal" << std::endl;
		ac.sendGoal(goal);
		std::cout << "before wait" << std::endl;
		ac.waitForResult();	
		
		ROS_INFO("Action (%s) performed: completed!", msg->name.c_str());
		return true;
	}
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "my_rosplan_action", ros::init_options::AnonymousName);
	ros::NodeHandle nh("~");
	KCL_rosplan::MyActionInterface my_aci(nh);
	my_aci.runActionInterface();
	return 0;
}

