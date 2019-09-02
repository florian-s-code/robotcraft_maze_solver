#include <stdint.h>

#include "ros/ros.h"
#include "nav_msgs/MapMetaData.h"
#include "nav_msgs/OccupancyGrid.h"
#include "maze_robotcraft/TargetPose.h"
#include "geometry_msgs/Pose2D.h"

#include "WPA/WPA.hpp"

typedef WavefrontPlanner::Coord Coord;

class MazeWPAPlanner
{
  private:
    ros::NodeHandle n;
    ros::Subscriber map_sub;
    ros::Subscriber map_metadata_sub;
    ros::Publisher pose_pub;

    ros::ServiceServer get_pose_server;

    WavefrontPlanner planner;

    Coord start_pos;
    Coord end_pos;

    std::vector<Coord> path;

    bool has_map = false;
    bool has_path = false;

    geometry_msgs::Pose2D getPoseFromCoord(const Coord &coord) {
	geometry_msgs::Pose2D pose;
	pose.x = coord.x;
	pose.y = coord.y;

	return pose;
    }

    bool getPoseCallback(maze_robotcraft::TargetPose::Request& request, maze_robotcraft::TargetPose::Response& response)
    {
        if(!has_map) {
            ROS_INFO("No map.");
            return false;
        }

        if(!has_path) {
            ROS_INFO("No path yet.... Creating one.");
            planner.createPath();
            path = planner.getPath();
            has_path = true;
        }

	for(int i = 0; i < path.size(); i++) {
    	    geometry_msgs::Pose2D pose = getPoseFromCoord( path[i] );
    	    ROS_INFO("%f, %f", pose.x, pose.y);
	}

	if(request.id < path.size()) {
	    response.id = request.id + 1;
	    geometry_msgs::Pose2D target = getPoseFromCoord( path[request.id+1] );
	    response.pose = target; 
    	    ROS_INFO("Responding to request, with target (%f, %f, %f).", target.x, target.y, target.theta);
	    return true;
	}



        return false;
    }

    /* When receiving a map, reinitialize the planner with a map */
    void mapSubCallback(const nav_msgs::OccupancyGrid::ConstPtr& map_msg)
    {
        planner = WavefrontPlanner((int)map_msg->info.width, (int)map_msg->info.height, (float)map_msg->info.origin.position.x, (float)map_msg->info.origin.position.y, (float)map_msg->info.resolution,  map_msg->data);
        planner.setStart(start_pos.x, start_pos.y );
        planner.setEnd(end_pos.x, end_pos.y);

        if (!has_map)
            has_map = true;

        ROS_INFO("Got the map !");
    }

  public:
    MazeWPAPlanner()
    {
        // Initialize ROS
        ros::NodeHandle nh("~"); //private NodeHandle to read private parameters
        nh.param<float>("start_x", start_pos.x, 0.0);
        nh.param<float>("start_y", start_pos.y, 0.0);
        nh.param<float>("end_x", end_pos.x, 3.920);
        nh.param<float>("end_y", end_pos.y, -2.994);

        get_pose_server = n.advertiseService("get_next_pos", &MazeWPAPlanner::getPoseCallback, this);

        map_sub = n.subscribe("map", 2, &MazeWPAPlanner::mapSubCallback, this);

        ROS_INFO("Initialized. Parameters : end_x = %f, end_y = %f.", end_pos.x, end_pos.y);
    }

    void run()
    {
        // Send messages in a loop
        ros::Rate loop_rate(10);
        while (ros::ok())
        {
            ros::spinOnce();

            // And throttle the loop
            loop_rate.sleep();
        }
    }
};

int main(int argc, char** argv)
{
    // Initialize ROS
    ros::init(argc, argv, "WPA_planner");

    // Create our controller object and run it
    auto node = MazeWPAPlanner();
    node.run();

    // And make good on our promise
    return 0;
}
