#include <iostream>

#include <cstdlib>

#include "ros/ros.h"
#include "tf/transform_datatypes.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Twist.h"

const float checkf_distance = 0.4;
const float checks_distance = 0.5;


class MazeBasicController 
{
private:

    enum State { BEGIN, FOLLOWING, TURNRIGHT, TURNLEFT };
  
    ros::NodeHandle n;
    ros::Publisher cmd_vel_pub;
    ros::Subscriber front_sensor_sub;
    ros::Subscriber left_sensor_sub;
    ros::Subscriber right_sensor_sub;
    ros::Subscriber odom_sub;
    //ros::Publisher set_pose_pub;

    float x, y, theta = 0;

    void frontSensorCallback(const sensor_msgs::LaserScan::ConstPtr& range_msg) {
	front_distance = range_msg->ranges[0];
    }

    void rightSensorCallback(const sensor_msgs::LaserScan::ConstPtr& range_msg) {
	right_distance = range_msg->ranges[0];
    }
    
    void leftSensorCallback(const sensor_msgs::LaserScan::ConstPtr& range_msg) {
	left_distance = range_msg->ranges[0];
    }

    void odomCallback(const nav_msgs::Odometry::ConstPtr& odom_msg) {
        x = odom_msg->pose.pose.position.x;
        y = odom_msg->pose.pose.position.y;
        theta = tf::getYaw(odom_msg->pose.pose.orientation);
    }

    double front_distance = -999;
    double left_distance;
    double right_distance;
    bool robot_stopped;

    State state;
    int pledge_number = 0; //save the total rotation done, see Pledge algorithm

    geometry_msgs::Twist calculateCommand()
    {
        auto msg = geometry_msgs::Twist();
        
        switch(state) {
	    case BEGIN:
  	    	return stateBegin();
  	    	break;
  	    case FOLLOWING:
    	    	return stateFollowing();
    	    	break;
    	    case TURNRIGHT:
      	    	return stateTurnRight();
      	    	break;
      	    case TURNLEFT:
		return stateTurnLeft();
        	break;
        }
    }

    /* Pledge algorithm : used to avoid the robot begin stuck on wall not connected to exit
     * We sum the rotation that the robot does, we add the angle of CCW turn and
     * substract for CW turn. When this value come back to 0, the robot will go
     * straight instead of turning (ie. BEGIN state)
     */
    void updatePledge(int increment) {
	pledge_number += increment;

        if(pledge_number == 0 && state != BEGIN) {
            state = BEGIN;
        }
    }


    geometry_msgs::Twist stateBegin()
    {
        auto msg = geometry_msgs::Twist();

	if( front_distance < checkf_distance ) {
  	    state = TURNRIGHT;
            
  	    return msg;
	}

	msg.linear.x = 1.0;
	return msg;
    }

    geometry_msgs::Twist stateFollowing()
    {
        auto msg = geometry_msgs::Twist();
	//state transition
	if( front_distance < checkf_distance ) {
  	    state = TURNRIGHT;
            
  	    return msg;
	}
	if( left_distance > checks_distance ) {
  	    state = TURNLEFT;
            
  	    return msg;
	}

	msg.linear.x = 1.0;
	return msg;
    }
    
    geometry_msgs::Twist stateTurnRight()
    {
        auto msg = geometry_msgs::Twist();
	//state transition
	if( front_distance > 0.6 && left_distance >=0.5 ) {
  	    state = FOLLOWING;
  	    return msg;
	}
	
	msg.angular.z = -0.4;
        msg.linear.x = 0.06;
	updatePledge(-1);
	return msg;
    }

    geometry_msgs::Twist stateTurnLeft()
    {
        auto msg = geometry_msgs::Twist();
	//state transition
	if( left_distance < checks_distance ) {
  	    state = FOLLOWING;
  	    return msg;
	}
	
	msg.angular.z = +0.4;
        msg.linear.x = 0.1;
	updatePledge(+1);
	return msg;
    }
public:
    MazeBasicController(){
        // Initialize ROS
        this->n = ros::NodeHandle();

        front_sensor_sub = n.subscribe("ir_front_sensor", 10, &MazeBasicController::frontSensorCallback, this);
        right_sensor_sub = n.subscribe("ir_right_sensor", 10, &MazeBasicController::rightSensorCallback, this);
        left_sensor_sub = n.subscribe("ir_left_sensor", 10, &MazeBasicController::leftSensorCallback, this);
        odom_sub = n.subscribe("odom", 10, &MazeBasicController::odomCallback, this);

        cmd_vel_pub = this->n.advertise<geometry_msgs::Twist>("cmd_vel", 5);

	//Initialize state
	state = BEGIN;
    }

    void run(){
        // Send messages in a loop
        ros::Rate loop_rate(10);
        while(front_distance == -999) {
		
            ros::spinOnce();
            loop_rate.sleep();
        }
        while (ros::ok())
        {
            // Calculate the command to apply
            auto msg = calculateCommand();

            // Publish the new command
            this->cmd_vel_pub.publish(msg);

            ros::spinOnce();

            // And throttle the loop
            loop_rate.sleep();
        }
    }

};


int main(int argc, char **argv){
    // Initialize ROS
    ros::init(argc, argv, "reactive_controller");

    // Create our controller object and run it
    auto controller = MazeBasicController();
    controller.run();

    // And make good on our promise
    return 0;
}
