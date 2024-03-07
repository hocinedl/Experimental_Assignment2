#include "ros/ros.h"
#include "rosplan_interface/detect_action.h"
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int32.h>

namespace KCL_rosplan {
	// Constructor for Detect_Interface class
    Detect_Interface::Detect_Interface(ros::NodeHandle &nh) : nh_(nh) {
    
            // Advertise the cmd_vel topic to publish Twist messages
        cmd_vel_publisher = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
        
            // Subscribe to the marker_id topic to receive Int32 messages
        marker_id_subscriber = nh_.subscribe("/marker_id", 10, &Detect_Interface::marker_id_Callback, this);
        
        
        // Initializing marker IDs
        expected_marker_id =1;
        marker_id =0;
    }
    
	// Callback function for the marker_id topic
    void Detect_Interface::marker_id_Callback(const std_msgs::Int32::ConstPtr& msg) {
        marker_id = msg->data;
        ROS_INFO("Received marker id: %d", marker_id);
    }

    bool Detect_Interface::concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg) { 
            // Determining expected marker ID based on action parameters
            
        if (msg->parameters[1].value == "m11") {
            expected_marker_id = 11;
        }
        if (msg->parameters[1].value == "m12") {
            expected_marker_id = 12;
        }
        if (msg->parameters[1].value == "m13") {
            expected_marker_id = 13;
        }
        if (msg->parameters[1].value == "m15") {
            expected_marker_id = 15;
        }
        
        // Creating a Twist message for velocity control
        geometry_msgs::Twist cmd_vel;
        
        // Adjusting robot's movement based on expected marker ID
        
        if (expected_marker_id == marker_id) {
            cmd_vel.angular.z = 0.0;
        } else {
            cmd_vel.angular.z = 0.5;
        }
        
       // Publishing velocity command
        cmd_vel_publisher.publish(cmd_vel);

        return true;
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "detect_action", ros::init_options::AnonymousName);
    ros::NodeHandle nh("~");
    KCL_rosplan::Detect_Interface detect_interface(nh);
    detect_interface.runActionInterface();
    return 0;
}


