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
    float initial_x, initial_y, initial_theta;

    int endX, endY;
    int startX, startY;


    //Publisher Topics
    ros::Publisher cmd_vel_pub;
    ros::Publisher set_pose_pub;
    ros::Publisher odom_pub;
    ros::Publisher rgb_leds_pub;
    ros::Publisher ir_front_sensor;
    ros::Publisher ir_left_sensor;
    ros::Publisher ir_right_sensor;
    ros::Publisher initial_pose_pub;

    //Subscriber Topics
    ros::Subscriber left_dist_sub;
    ros::Subscriber right_dist_sub;
    ros::Subscriber front_dist_sub;
    ros::Subscriber pose_sub;
    ros::Subscriber path_following_vel_sub;

    geometry_msgs::Twist calculateCommand(){
        auto msg = geometry_msgs::Twist();
    }

    //Callback functions for subscribers
    void poseCallback(const geometry_msgs::Pose2D& pose_msg){
        current_time = ros::Time::now();

        //since all odometry is 6DOF we'll need a quaternion created from yaw
        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(pose_msg.theta);

        //first, we'll publish the transform over tf - broadcasts to rviz
        geometry_msgs::TransformStamped odom_trans;
        odom_trans.header.stamp = current_time;
        odom_trans.header.frame_id = "odom";
        odom_trans.child_frame_id = "base_link";

        odom_trans.transform.translation.x = pose_msg.x;
        odom_trans.transform.translation.y = pose_msg.y;
        odom_trans.transform.translation.z = 0.0;
        odom_trans.transform.rotation = odom_quat;

        //send the transform
        odom_broadcaster.sendTransform(odom_trans);

        //next, we'll publish the odometry message over ROS
        nav_msgs::Odometry odom;
        odom.header.stamp = current_time;
        odom.header.frame_id = "odom";

        //set the position
        odom.pose.pose.position.x = pose_msg.x;
        odom.pose.pose.position.y = pose_msg.y;
        odom.pose.pose.position.z = 0.0;
        odom.pose.pose.orientation = odom_quat;

        //set the velocity
        odom.child_frame_id = "base_link";
        odom.twist.twist.linear.x = v;
        odom.twist.twist.linear.y = 0.0;
        odom.twist.twist.angular.z = w;

        //crate our covariance Matrix6x6
        float covariance[6][6];
        for (int i = 0;i<6;i++){
          for(int j=0;j<6;j==){
            covariance[i][j]=0.0;
          }
        }
        odom.pose.covaariance=covariance;

        //publish the message
        odom_pub.publish(odom);
        last_time = current_time;

    }

    void leftCallback( const std_msgs::Float32::ConstPtr& msg){
        sensor_msgs::Range ir_left;

        //.data is for acces to all the data from the msg
        float sensor_val_left = static_cast<float>(msg->data);
        if(sensor_val_left < 0.15){
            ROS_WARN("Collision risk! The robot is %f meters of an obsctacle, on the left side", sensor_val_left);
        }

        ir_left.header.frame_id = "base_link";
        ir_left.radiation_type = 1;
        ir_left.field_of_view = 0.034906585;
        ir_left.min_range = 0.1;
        ir_left.max_range = 0.8;
        ir_left.range = sensor_val_left;

        //Publish the left sensor value to ir_left_sensor
        ir_left_sensor.publish(ir_left);



    }

    void rightCallback(const std_msgs::Float32::ConstPtr& msg){
        sensor_msgs::Range ir_right;

        float sensor_val_right = static_cast<float>(msg->data);
        if(sensor_val_right < 0.15){
            ROS_WARN("Collision risk! The robot is %f meters of an obsctacle, on the right side", sensor_val_right);
        }

        ir_right.header.frame_id = "base_link";
        ir_right.radiation_type = 1;
        ir_right.field_of_view = 0.034906585;
        ir_right.min_range = 0.1;
        ir_right.max_range = 0.8;
        ir_right.range = sensor_val_right;

        //Publish the right sensor value to ir_right_sensor
        ir_right_sensor.publish(ir_right);
    }

    void frontCallback(const std_msgs::Float32::ConstPtr& msg){
        sensor_msgs::Range ir_front;

        float sensor_val_front = static_cast<float>(msg->data);
        if(sensor_val_front < 0.15){
            ROS_WARN("Collision risk! The robot is %f meters of an obsctacle, on the front side", sensor_val_front);
        }

        ir_front.header.frame_id = "base_link";
        ir_front.radiation_type = 1;
        ir_front.field_of_view = 0.034906585;
        ir_front.min_range = 0.1;
        ir_front.max_range = 0.8;
        ir_front.range = sensor_val_front;

        //Publish the front sensor value to ir_front_sensor
        ir_front_sensor.publish(ir_front);
    }


    void pathfollowingVelCallback(const geometry_msgs::Twist& vel_msg){
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
        pose_MSG.x = initial_x;
        pose_MSG.y = initial_y;
        pose_MSG.theta = initial_theta;
        return pose_MSG;
    }

    std_msgs::UInt8MultiArray setLEDs(){
    auto rgb_MSG = std_msgs::UInt8MultiArray();
    // Set color of led lights, the first 3 enteries are for LED_1 [255,0,0] and the last 3 for LED_2 [0,255,0]
    // THIS IS NOT SET UP CORRECTLY, FOLLOW THIS EXAMPKE: http://alexsleat.co.uk/2011/07/02/ros-publishing-and-subscribing-to-arrays/
    //rgb_MSG.data = [255,0,0,0,255,0];
    return rgb_MSG;
    }

    nav_msgs::PoseWithCovarianceStamped initPose(){
        auto pose_cov_MSG = nav_msgs::PoseWithCovarianceStamped();
        // Set position to zero
        pose_cov_MSG.pose.pose.position.x = initial_pose_x;
        pose_cov_MSG.pose.pose.positiony = initial_pose_y;
        pose_cov_MSG.pose.pose.position.theta = initial_pose_theta;

        float covariance[6][6];
        for (int i = 0;i<6;i++){
          for(int j=0;j<6;j==){
            covariance[i][j]=0.0;
          }
        }
        odom.pose.covaariance=covariance;

        return pose_cov_MSG;


    }

public:
  //constructor
    RobotDriver(){
        // Initialize ROS
        this->n = ros::NodeHandle();

        this->n.getParam("/initial_x", initial_x);
        this->n.getParam("/initial_y", initial_y);
        this->n.getParam("/initial_theta", initial_theta);

        this->n.getParam("/initial_x", initial_pose_x);
        this->n.getParam("/initial_y", initial_pose_y);
        this->n.getParam("/initial_theta", initial_pose_theta);

        // Create a publisher object, able to push messages
        this->cmd_vel_pub = this->n.advertise<geometry_msgs::Twist>("cmd_vel", 10);
        this->set_pose_pub = this->n.advertise<geometry_msgs::Pose2D>("set_pose", 10);
        this->init_pose_pub = this->n.advertise<nav_msgs::PoseWithCovarianceStamped>("initialpose", 10);
        // Create a subscriber for laser scans
        this->path_following_vel_sub = n.subscribe("path_following_vel", 10, &RobotDriver::pathfollowingVelCallback, this);
        this->pose_sub = n.subscribe("pose", 10, &RobotDriver::poseCallback, this);
        this->left_dist_sub = n.subscribe("left_distance", 10, &RobotDriver::leftCallback, this);
        this->right_dist_sub = n.subscribe("right_distance", 10, &RobotDriver::rightCallback, this);
        this->front_dist_sub = n.subscribe("front_distance", 10, &RobotDriver::frontCallback, this);

    }



    void run(){
        int count = 0;

        // Send messages in a loop
        ros::Rate loop_rate(10);

        while (ros::ok())
        {

            // Calculate the command to apply
            auto vel_MSG = cmdVelUpdate();
            auto msg = calculateCommand();
            auto rgb_MSG = setLEDs();
            auto pose_MSG = setPose();
            auto pose_cov_MSG = initPose();

            // Publish the new command
            this->cmd_vel_pub.publish(vel_MSG);
            this->set_pose_pub.publish(pose_MSG);
            this->init_pose_pub.publish(pose_cov_MSG);


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
