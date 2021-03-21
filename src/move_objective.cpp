/*
Node to move the objective in a pre-planned motion.
Motion is described by velocity comands depending on time.
*/

#include "ros/ros.h"
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
//#include <string>
//#include <tf/tf.h>
//#include <tf/transform_listener.h>

ros::Publisher pubV_objetivo;
int t_objetivo;

void publish_obj_actions(){
    //Publish objective pre-planned motion
    geometry_msgs::Twist cmd_vel_obj;

    if (t_objetivo<50){
        cmd_vel_obj.linear.x = 0.0;
        cmd_vel_obj.angular.z = 0.0;
    }else if(t_objetivo<100){
        cmd_vel_obj.linear.x = 0.5;
        cmd_vel_obj.angular.z = 0.0;
    }else if(t_objetivo<150){
        cmd_vel_obj.linear.x = 0.5;
        cmd_vel_obj.angular.z = -0.2;
    }else if(t_objetivo<200){
        cmd_vel_obj.linear.x = 0.5;
        cmd_vel_obj.angular.z = 0.0;
    }else{
        cmd_vel_obj.linear.x = 0.0;
        cmd_vel_obj.angular.z = 0.0;
    }

    cmd_vel_obj.linear.x = 0.0;
    cmd_vel_obj.angular.z = 0.0;

    t_objetivo++;
    pubV_objetivo.publish(cmd_vel_obj);
}

int main(int argc, char **argv){
    t_objetivo=0;
    //--------------- ROS COMUNICATION -------------------------------------------------
    // Init ROS, Publishers and suscribers
    //----------------------------------------------------------------------------------

    ros::init(argc, argv, "obj_node");
    ros::NodeHandle n;
    ros::Rate rate(10); //10 Hz. T=0.1s

    ROS_INFO("Objective actions node started");

    pubV_objetivo = n.advertise<geometry_msgs::Twist>("robot_1/cmd_vel", 10);

    while (ros::ok()) {
        publish_obj_actions();

        ros::spinOnce();
        rate.sleep();
    }
    return 0;
    ROS_INFO("Out of wphile loop");
}