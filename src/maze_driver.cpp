#include <cstdlib>

#include "ros/ros.h"
#include "tf/transform_datatypes.h"
#include "sensor_msgs/Range.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/Twist.h"
#include "maze_robotcraft/TargetPose.h"

#define SQUARE_SIZE 0.2
#define ANGULAR_SPEED 1.0
#define LINEAR_SPEED 1.0

class SquareController
{
  private:
    ros::NodeHandle n;
    ros::Subscriber front_sensor_sub;
    ros::Subscriber left_sensor_sub;
    ros::Subscriber right_sensor_sub;
    ros::Subscriber pose_sub;
    ros::Publisher cmd_vel_pub;
    ros::Publisher set_pose_pub;

    float x, y, theta = 0;
    float last_x, last_y, last_theta = 0;
    float new_x, new_y, new_theta = 0;
    int orientation = 0;
    int new_orientation = 0;
    int last_orientation = 0;
    int current_id = 0; //current id of the target we want to go to

    enum State
    {
        STRAIGHT_X,
        STRAIGHT_Y,
        TURN
    };

    State state = TURN;
    State last_state = STRAIGHT_Y;

    void frontSensorCallback(const sensor_msgs::Range::ConstPtr& range_msg)
    {
        if (range_msg->range < 0.15)
        {
            //ROS_WARN("Collision risk [FRONT] : robot is %f meters away from obstacle.", range_msg->range);
        }
    }

    void rightSensorCallback(const sensor_msgs::Range::ConstPtr& range_msg)
    {
        if (range_msg->range < 0.15)
        {
            //ROS_WARN("Collision risk [RIGHT] : robot is %f meters away from obstacle.", range_msg->range);
        }
    }

    void leftSensorCallback(const sensor_msgs::Range::ConstPtr& range_msg)
    {
        if (range_msg->range < 0.15)
        {
            //ROS_WARN("Collision risk [LEFT] : robot is %f meters away from obstacle.", range_msg->range);
        }
    }

    void newposCallback(const geometry_msgs::Pose2D::ConstPtr& new_pose)
    {
        x = new_pose->x;
        y = new_pose->y;
        theta = new_pose->theta;
    }

    /* Send a m/set_pos message to the robot so it restarts its tracking to origin */
    void setRobotToOrigin()
    {
        geometry_msgs::Pose2D zero_msg;
        set_pose_pub.publish(zero_msg);  // set position of the robot to 0, 0, 0
    }

    void stateStraightX()
    {
        geometry_msgs::Twist msg;
        if (std::abs(new_x - x) <= 0.05)
        {
            last_x = x;
            last_state = STRAIGHT_X;
            getNewPosition();
            state = TURN;
            return;
        }

        msg.linear.x = LINEAR_SPEED;
        cmd_vel_pub.publish(msg);
        
    }

    void stateStraightY()
    {
        geometry_msgs::Twist msg;
        if (std::abs(new_y - y) <= 0.05)
        {
            last_y = y;
            last_state = STRAIGHT_Y;
            getNewPosition();
            state = TURN;
            return;
        }
        msg.linear.x = LINEAR_SPEED;
        cmd_vel_pub.publish(msg);
    }

    /*geometry_msgs::Twist stateTurn()
    {
        geometry_msgs::Twist msg;
        int k = orientation - last_orientation;
        if (k > 0 && k < 3)
        {
            if (std::abs(last_theta - theta) >= k * M_PI / 2)
            {
                last_theta = theta;
                last_orientation = orientation;
                if (last_state == STRAIGHT_X)
                    state = STRAIGHT_Y;
                else
                    state = STRAIGHT_X;
                return msg;
            }
            msg.angular.z = ANGULAR_SPEED;
            return msg;
        }
        else if (k < 0 && k > -3)
        {
            if (std::abs(last_theta - theta) >= k * M_PI / 2)
            {
                last_theta = theta;
                last_orientation = orientation;
                if (last_state == STRAIGHT_X)
                    state = STRAIGHT_Y;
                else
                    state = STRAIGHT_X;
                return msg;
            }
            msg.angular.z = -ANGULAR_SPEED;
            return msg;
        }
        else if (k == -3)
        {
            if (std::abs(last_theta - theta) >= M_PI / 2)
            {
                last_theta = theta;
                last_orientation = orientation;
                if (last_state == STRAIGHT_X)
                    state = STRAIGHT_Y;
                else
                    state = STRAIGHT_X;
                return msg;
            }
            msg.angular.z = ANGULAR_SPEED;
            return msg;
        }
        else if (k == 3)
        {
            if (std::abs(last_theta - theta) >= M_PI / 2)
            {
                last_theta = theta;
                last_orientation = orientation;
                if (last_state == STRAIGHT_X)
                    state = STRAIGHT_Y;
                else
                    state = STRAIGHT_X;
                return msg;
            }
            msg.angular.z = -ANGULAR_SPEED;
            return msg;
        }
        else if (k == 0)
        {
            last_orientation = orientation;
            if (last_state == STRAIGHT_X)
                state = STRAIGHT_Y;
            else
                state = STRAIGHT_X;
            return msg;
        }
    }*/

    void stateTurn() 
    {
        if (std::abs(new_theta - theta) <= 0.01)
        {

            last_theta = theta;
            if (last_state == STRAIGHT_X)
                state = STRAIGHT_Y;
            else
                state = STRAIGHT_X;

            return;
        }
	geometry_msgs::Twist msg;
        msg.angular.z = ANGULAR_SPEED;
        cmd_vel_pub.publish(msg);
    }

    void getNewPosition() {
        maze_robotcraft::TargetPose service;
        service.request.id = current_id;

        ROS_INFO("Requesting new position");

        if (ros::service::call("get_next_pos", service) ) {
            new_x = -service.response.pose.x;
            new_y = -service.response.pose.y;
            new_theta = service.response.pose.theta;

            decideRotation(new_x, new_y);

            current_id = service.response.id;

            ROS_INFO("New position receivedi (%f, %f, %f)", new_x, new_y, new_theta);
        } else {
            ROS_WARN("Could not get position");
        }
    }

    void calculateCommand()
    {
        ROS_INFO("Theta : %f , X : %f, Y : %f", theta, x, y);
        switch (state)
        {
            case (STRAIGHT_X):
                stateStraightX();
                break;
            case (STRAIGHT_Y):
                stateStraightY();
                break;
            case (TURN):
                stateTurn();
                break;
        }
    }

    // Function made by shekhar
    void decideRotation(float new_x, float new_y)
    {
        if (new_x > x)
        {
            orientation = 0;
            last_y = new_y;
            state = TURN;
        }
        else if (new_x < x)
        {
            orientation = 2;
            last_y = new_y;
            state = TURN;
        }
        else if (new_y > y)
        {
            orientation = 1;
            last_x = new_x;
            state = TURN;
        }
        else if (new_y < y)
        {
            orientation = 3;
            last_x = new_x;
            state = TURN;
        }

        new_theta = orientation * M_PI/2;
    }

  public:
    SquareController()
    {
        n = ros::NodeHandle();
        front_sensor_sub = n.subscribe("ir_front_sensor", 10, &SquareController::frontSensorCallback, this);
        right_sensor_sub = n.subscribe("ir_right_sensor", 10, &SquareController::rightSensorCallback, this);
        left_sensor_sub = n.subscribe("ir_left_sensor", 10, &SquareController::leftSensorCallback, this);
        pose_sub = n.subscribe("pose", 10, &SquareController::newposCallback, this);

        cmd_vel_pub = this->n.advertise<geometry_msgs::Twist>("cmd_vel", 5);
        set_pose_pub = this->n.advertise<geometry_msgs::Pose2D>("set_pose", 5);

        setRobotToOrigin();
    }

    void run()
    {
        // Send messages in a loop

        last_x = x;
        last_y = y;
        last_theta = theta;

        ros::Rate loop_rate(10);
        while (ros::ok())
        {
            ROS_INFO("State is %d", state);

            //calculate and publish cmd_vel
            calculateCommand();
            // spin
            ros::spinOnce();
            // And throttle the loop
            loop_rate.sleep();
        }
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "talker");
    auto controller = SquareController();
    controller.run();

    return 0;
}
