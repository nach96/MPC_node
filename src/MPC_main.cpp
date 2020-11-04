#include "ros/ros.h"
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <string>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include "mynlp.cpp"

/*
Description:

 -SUSCRIBE Person (Robot1) and Robot(Robot0) position 
 -PUBLISH desired path.
 -PUBISH velocity commands to follow that path //TODO

//Path and actions are calculated on an MPC fashion, each 10 ms.

*/

ros::Publisher pubPath;
ros::Subscriber subPose0;
ros::Subscriber subPose1;
//Publisher v and w.


ros::Time time_start; //Start time of each NLP

geometry_msgs::PoseWithCovarianceStamped pose0;
geometry_msgs::PoseWithCovarianceStamped pose1;
nav_msgs::Path path;
bool recived0 = false;
bool recived1 = false;


//-----------------------Create object from class myNLP------------------------------
myNLP mynlp;


//----------------------- FUNCTIONS--------------------------------------------------
 
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
void calculate_fake_path(double _xr, double _yr, double _wr,double _xp, double _yp,double _wp){

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

void publish_new_path(tf::TransformListener* transform_listener){
    path.poses.resize(N);
    for( unsigned i=0; i<mynlp.Xr.size(); ++i){
        geometry_msgs::PoseStamped poseNew;
        poseNew.pose.position.x = mynlp.Xr[i];
        poseNew.pose.position.y = mynlp.Yr[i];
        poseNew.pose.orientation.w = cos(mynlp.Wr[i]/2);
        poseNew.pose.orientation.z = sin(mynlp.Wr[i]/2);
        poseNew.header.frame_id = "robot_0/base_link"; //Robot_0/base_link but as this node will be inside Robot_0 namespace, you can say directly base_link

        geometry_msgs::PoseStamped poseMap;

        try{
            transform_listener->waitForTransform("map", "robot_0/base_link",time_start, ros::Duration(5.0));
            transform_listener->transformPose("map", time_start, poseNew, "map", poseMap);

        }catch(tf::TransformException ex){
            ROS_ERROR("orm XX %s", ex.what());
        }
        

        path.poses[i] = poseMap;
    }
    pubPath.publish(path);
}

int main(int argc, char **argv){

    //----------------Variables----------------------------------------------
    double xr;
    double yr;//For some reason y0 gives faillures, so I had to change to yr ¿Why?
    double titar;
    double xp;
    double yp;
    double titap;
    double xpr;
    double ypr;
    double titapr;
    ros::WallTime t0,tf,t1,t2; //Meassure computation time

    //--------------- ROS COMUNICATION -------------------------------------------------
    // Init ROS, Publishers and suscribers
    //----------------------------------------------------------------------------------

    ros::init(argc, argv, "MPC_node");
    ros::NodeHandle n;
    ros::Rate rate(1); //10 Hz. T=0.1s

    ROS_INFO("MPC_Node started");

    subPose0 = n.subscribe("/robot_0/amcl_pose", 10, pose0_callback);
    subPose1 = n.subscribe("robot_1/amcl_pose", 10, pose1_callback);
    pubPath = n.advertise<nav_msgs::Path>("robot_0/matlab/path", 10);
    //frame_id es base respecto a la cual está representado pose. Por eso es map.(En este caso, en el MPC puede que sea el robot o la persona etc.)
    path.header.frame_id= "map";


    //Transform_litener to get Transform robot0->World and store it in T
    tf::TransformListener transform_listener;
    tf::StampedTransform RT_person_robot;

    //Frames as Parameters

    //std::string base_link_frame="base_link";
    //std::string global_frame="map";

    //nh.getParam("base_link_frame", base_link_frame);
    //nh.getParam("global_frame", global_frame);
    


    //-------------------------Starts MPC loop--------------------------------------------
    //-------------  Creates and solves a new NLP each 10Hz  -----------------------------
    //------------------------------------------------------------------------------------
    while (ros::ok()) {

         t0 = ros::WallTime::now();

        if (recived0){
            xr = pose0.pose.pose.position.x;
            yr = pose0.pose.pose.position.y;
            //IMPORTANTE!!!!! ORIENTATOIN.W NO ES EL ÁNGULO! Q=(vk·sin(tita/2),cos(tita/2))
            titar = tf::getYaw(pose0.pose.pose.orientation);
            recived0 = false;
        }
        if (recived1){
            xp = pose1.pose.pose.position.x;
            yp = pose1.pose.pose.position.y;
            titap = tf::getYaw(pose1.pose.pose.orientation);
            recived1 = false;
        }
           
          
        //Get transform, robot_1 location relative to robot_0
        try {
            //transform_listener.waitForTransform("robot_0/base_link", "robot_1/base_link", ros::Time(0), ros::Duration(10.0));
            transform_listener.lookupTransform("robot_0/base_link", "robot_1/base_link", ros::Time(0), RT_person_robot);
        }
        catch (tf::TransformException ex) {
            ROS_ERROR("1st transform orm XX %s", ex.what());
        }
        
        
        //Get time when optimal control starts. Use location at this time as reference for the full generated path.
        time_start = ros::Time::now();
        //Calculate person location relative to the robot_0 frame
        xpr = RT_person_robot.getOrigin().x();
        ypr = RT_person_robot.getOrigin().y();
        titapr = tf::getYaw(RT_person_robot.getRotation());

        std::cout<<"xpr"<<xpr<<"\n";
        std::cout<<"ypr"<<ypr<<"\n";

        //----------------------------Solve the nonlinear problem------------------------------------
        
        t1 = ros::WallTime::now();
        mynlp.my_solve(xpr,ypr,titapr);
        t2 = ros::WallTime::now();
        publish_new_path(&transform_listener);

        tf = ros::WallTime::now();

        


        double exec_time = (tf-t0).toNSec()*1e-6;
        double nlp_time = (t1-t2).toNSec()*1e-6;

        std::cout <<"cylce exec time [ms]: "<<exec_time << "\n";
        std::cout <<"nlp time [ms]: "<<nlp_time << "\n";



        ros::spinOnce();
        rate.sleep();

    }

    return 0;
    ROS_INFO("Out of wphile loop");

}