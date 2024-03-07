#include <ros/ros.h>
#include "rosplan_action_interface/RPActionInterface.h"
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>

namespace KCL_rosplan {
	class Detect_Interface: public RPActionInterface
	{
		private:
		ros::NodeHandle nh_;
		ros::Publisher cmd_vel_publisher;
		ros::Subscriber marker_id_subscriber;
		int expected_marker_id;
    		int marker_id;
	
		
		public:
				// constructor 
				Detect_Interface(ros::NodeHandle &nh);
			
				// listen to and process action_dispatch topic 
				bool concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg);
				
				void marker_id_Callback(const std_msgs::Int32::ConstPtr& msg);
	};
}



/*
#include <ros/ros.h>
#include "rosplan_action_interface/RPActionInterface.h"
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <nav_msgs/Odometry.h>

namespace KCL_rosplan {
	class Detect_Interface: public RPActionInterface
	{
		private:
		// ros::NodeHandle nh_;
		//ros::Publisher cmd_vel_pub_;
		//ros::Subscriber marker_center_sub
	        //marker_center = 0.0;
        	//camera_width = 320.0;
        	//pixel_thr = 18.0;
        	//is_rotating_ = false;
		
		public:
				/* constructor 
				Detect_Interface(ros::NodeHandle &nh);
			
				/* listen to and process action_dispatch topic 
				bool concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg);
				
				void subCallback(const geometry_msgs::Point::ConstPtr& msg);
	};
}

*/
