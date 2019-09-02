#include <iostream>

#include <cstdlib>

#include "ros/ros.h"
#include "tf/transform_datatypes.h"
#include "sensor_msgs/Range.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/Twist.h"


//const float checkf_distance = 0.05;
//const float checks_distance = 0.07;
const float checkf_distance = 0.15;
const float checks_distance = 0.2;


    ros::Publisher cmd_vel_pub;
    ros::Subscriber front_sensor_sub;
    ros::Subscriber left_sensor_sub;
    ros::Subscriber right_sensor_sub;
    ros::Subscriber odom_sub;
    //ros::Publisher set_pose_pub;


//---------------------------------------------------------------------------

    enum State { BEGIN, FOLLOWING, TURNRIGHT, TURNLEFT };

    float x, y, theta = 0.0;

    double front_distance = -999;
    double left_distance=0.0;
    double right_distance=0.0;
    bool robot_stopped;
    State state = BEGIN;
    int pledge_number = 0; //save the total rotation done, see Pledge algorithm
    
// ROS callback functions
    void frontSensorCallback(const sensor_msgs::Range::ConstPtr& range_msg) {
	    front_distance = range_msg->range;
    }

    void rightSensorCallback(const sensor_msgs::Range::ConstPtr& range_msg) {
        right_distance = range_msg->range;
    }
    
    void leftSensorCallback(const sensor_msgs::Range::ConstPtr& range_msg) {
		left_distance = range_msg->range;
    }

    void odomCallback(const nav_msgs::Odometry::ConstPtr& odom_msg) {
        x = odom_msg->pose.pose.position.x;
        y = odom_msg->pose.pose.position.y;
        theta = tf::getYaw(odom_msg->pose.pose.orientation);
    }


//-----------------------------------------------------------
    /* Pledge algorithm : used to avoid the robot begin stuck on wall not connected to exit
     * We sum the rotation that the robot does, we add the angle of CCW turn and
     * substract for CW turn. When this value come back to 0, the robot will go
     * straight instead of turning (ie. BEGIN state)
     */
//---------------------------------------------------
void updatePledge(int increment) {
	//deactivated
	return;

	pledge_number += increment;

        if(pledge_number == 0 && state != BEGIN) {
            state = BEGIN;
        }
    }

//---------------------------------------------------------------------
    void stateBegin()
    {
        geometry_msgs::Twist msg;
       
	if( front_distance < checkf_distance ) {
  	    state = TURNRIGHT;
             	    
	}

	msg.linear.x = 1.0;
        cmd_vel_pub.publish(msg);
    }
//--------------------------------------------------
    void stateFollowing()
    {
        geometry_msgs::Twist msg;

	//state transition
	if( front_distance < checkf_distance ) {
  	    state = TURNRIGHT;
	}
	if( left_distance > checks_distance ) {
  	    state = TURNLEFT;
            return;
	}

	//add some correction for left
        if( left_distance < checks_distance/2 + 0.02) {
            msg.angular.z = -0.4;
            msg.linear.x = 0.06;
        } else {
            msg.linear.x = 1.0;
        }
	cmd_vel_pub.publish(msg);
    }
 //-----------------------------------------------------   
    void stateTurnRight()
    {
        geometry_msgs::Twist msg;

	//state transition
	//if( front_distance > checkf_distance && left_distance >= checkf_distance ) {
  	if( front_distance > 2*checkf_distance) {
            state = FOLLOWING;
	}
	
	msg.angular.z = -0.4;
        //msg.linear.x = 0.06;
	updatePledge(-1);
	cmd_vel_pub.publish(msg);
    }
 //-----------------------------------------------------   
    void stateTurnLeft()
    {
        geometry_msgs::Twist msg;

	//state transition
	if( left_distance <= checks_distance ) {
  	    state = FOLLOWING;
	}
	
	msg.angular.z = 0.4;
        msg.linear.x = 0.06;
	updatePledge(+1);
	cmd_vel_pub.publish(msg);
    
    }


 
//-------------------------------------------------------------------
    void calculateCommand()
    {
	ROS_INFO("Current state is %d", state);
        
        switch(state) {
	    case BEGIN:
  	    	stateBegin();
  	    	break;
  	    case FOLLOWING:
    	        stateFollowing();
    	    	break;
    	    case TURNRIGHT:
      	    	stateTurnRight();
      	    	break;
      	    case TURNLEFT:
		stateTurnLeft();
        	break;
        }
    }

//-------------------------------------------

    void run(){


    }
//----------------------------------------------
int main(int argc, char **argv){
    // Initialize ROS
    ros::init(argc, argv, "reactive_controller");
   
    ros::NodeHandle n;
   
    //Set the rate
    ros::Rate loop_rate(10);
    
    //Publishers and Subscribers
    front_sensor_sub = n.subscribe("ir_front_sensor", 10, frontSensorCallback);
    right_sensor_sub = n.subscribe("ir_right_sensor", 10, rightSensorCallback);
    left_sensor_sub = n.subscribe("ir_left_sensor", 10, leftSensorCallback);
    odom_sub = n.subscribe("odom", 10, &odomCallback);

    cmd_vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 5);

    //wait for the sensor message to arrive before starting
    while(front_distance == -999) {
         ros::spinOnce();
        loop_rate.sleep();
    }

    while (ros::ok())
    {
        //Start the FSM
        // run();
        calculateCommand();
        ros::spinOnce();


        // ROS_INFO("Sending velocity..");

        loop_rate.sleep();
    }
    //4 And make good on our promise
    return 0;
}
