#include "ros/ros.h"
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <string>
#include <tf/tf.h>

//Robot0 = Robot Ego
//Robot1 = Person
//Suscribe Person and Robot position and publish desired path.
ros::Publisher pubPath;
ros::Subscriber subPose0;
ros::Subscriber subPose1;


geometry_msgs::PoseWithCovarianceStamped pose0;
geometry_msgs::PoseWithCovarianceStamped pose1;
nav_msgs::Path path;
bool recived0 = false;
bool recived1 = false;
double xr;
double yr;//For some reason y0 gives faillures, so I had to change to yr ¿?
double wr;
double xp;
double yp;
double wp;
 
//Need to do 2 almost equal callbacks?
void pose0_callback(geometry_msgs::PoseWithCovarianceStamped pose){
    ROS_INFO("Pose0 recived");
    pose0 = pose; 
    recived0 = true;
}
void pose1_callback(geometry_msgs::PoseWithCovarianceStamped pose){
    ROS_INFO("Pose1 recived");
    pose1 = pose;
    recived1 = true;
}
void calculate_newp_path(double _xr, double _yr, double _wr,double _xp, double _yp,double _wp){

    

    double x_goal = _xp + 3*sin(_wp);
    double y_goal = _yp + 3*cos(_wp);


    path.poses.resize(10);
    //Create path with 10 points between actual position and goal 
    for(int i=0; i<10; i++){
    geometry_msgs::PoseStamped poseNew;
    poseNew.pose.position.x = _xr + (x_goal-_xr)/(10-i);
    poseNew.pose.position.y = _yr + (y_goal-_yr)/(10-i);
    double w = _wr + (_wp-_wr)/(10-i);
    poseNew.pose.orientation.w = cos(w/2);
    poseNew.pose.orientation.z = sin(w/2);
    poseNew.header.frame_id = "map";

    path.poses[i] = poseNew;

    }

    pubPath.publish(path);

}

int main(int argc, char **argv){

    ros::init(argc, argv, "MPC_node");
    ros::NodeHandle n;
    ros::Rate rate(50);

    ROS_INFO("MPC_Node started");

    subPose0 = n.subscribe("/robot_0/amcl_pose", 10, pose0_callback);
    subPose1 = n.subscribe("robot_1/amcl_pose", 10, pose1_callback);
    pubPath = n.advertise<nav_msgs::Path>("robot_0/matlab/path", 10);
    //frame_id es base respecto a la cual está representado pose. Por eso es map.(En este caso, en el MPC puede que sea el robot o la persona etc.)
    path.header.frame_id= "map";


    //Starts loop
    while (ros::ok()) {
        if (recived0 or recived1) {
            ROS_INFO("Inside If");
            if (recived0){
                xr = pose0.pose.pose.position.x;
                yr = pose0.pose.pose.position.y;
                //IMPORTANTE!!!!! ORIENTATOIN.W NO ES EL ÁNGULO! Q=(vk·sin(tita/2),cos(tita/2))
                wr = tf::getYaw(pose0.pose.pose.orientation);
            }
            if (recived1){
                xp = pose1.pose.pose.position.x;
                yp = pose1.pose.pose.position.y;
                wp = tf::getYaw(pose1.pose.pose.orientation);
            }
           
            calculate_newp_path(xr,yr,wr,xp,yp,wp);


            recived0 = false;
            recived1 = false;
        }



        ros::spinOnce();
        rate.sleep();

    }

    return 0;
    ROS_INFO("Out of wphile loop");

}