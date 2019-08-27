#include <iostream>
#include <cstdlib>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <rosserial_arduino/Test.h>
#include <string>
#include <list>
#include <tuple>

#include <vector>
#include <algorithm>
#include <utility>

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose2D.h"
#include "sensor_msgs/Range.h"
#include "std_msgs/UInt8MultiArray.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Header.h"

#include "nav_msgs/OccupancyGrid.h"
#include "std_msgs/Header.h"
#include "nav_msgs/MapMetaData.h"


#ifndef M_PI
#define M_PI 3.141592653589793238462643383279502884L
#endif na


class RobotDriver
{
private:
    //Initialising node NodeHandle
    ros::NodeHandle n;

    //Keep track of current ctime
    ros::Time current_time;
    ros::Time last_time = ros::Time::now();


    //Declare transform broadcaster to send messages via tf and ROS
    tf::TransformBroadcaster odom_broadcaster;

    double v;
    double w;

    int endX, endY;
    int startX, startY;


    //Publisher Topics
    ros::Publisher cmd_vel_pub;

    //Subscriber Topics
    ros::Subscriber reactive_vel_sub;



    void reactiveVelCallback(const geometry_msgs::Twist& vel_msg){
        // Update globally stored velocities for further use/publishing
        v=vel_msg.linear.x;
        w=vel_msg.angular.z;
    }

    geometry_msgs::Twist cmdVelUpdate(){
        auto vel_MSG = geometry_msgs::Twist();
        // Update velocities to be published
        vel_MSG.linear.x = v;
        vel_MSG.angular.z = w;
        return vel_MSG;
    }

    geometry_msgs::Pose2D setPose(){
        auto pose_MSG = geometry_msgs::Pose2D();
        // Set position to zero
        pose_MSG.x = 4.0;
        pose_MSG.y = 4.0;
        pose_MSG.theta = 0.0;
        return pose_MSG;
    }


public:
  //constructor
    RobotDriver(){
        // Initialize ROS
        this->n = ros::NodeHandle();
        // Create a publisher object, able to push messages
        this->cmd_vel_pub = this->n.advertise<geometry_msgs::Twist>("cmd_vel", 10);
        // Create a subscriber for laser scans
        this->reactive_vel_sub = n.subscribe("path_following_vel", 10, &RobotDriver::reactiveVelCallback, this);

    }



    void run(){
        int count = 0;

        // Send messages in a loop
        ros::Rate loop_rate(10);

        while (ros::ok())
        {

            // Calculate the command to apply
            auto vel_MSG = cmdVelUpdate();

            // Publish the new command
            this->cmd_vel_pub.publish(vel_MSG);


            ros::spinOnce();

            // And throttle the loop
            loop_rate.sleep();
        }
    }

};


int main(int argc, char **argv){

    // Initialize ROS
    ros::init(argc, argv, "robot_driver");

    // Create our controller object and run it
    auto controller = RobotDriver();
    controller.run();
}
