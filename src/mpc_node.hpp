#include "ros/ros.h"
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <string>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include "mynlp.cpp"

/*
Description:

 -SUSCRIBE Person (Robot1) and Robot(Robot0) position 
 -PUBLISH desired path.
 -PUBISH velocity commands to follow that path

//Path and actions are calculated with MPC at rate 10 ms.

*/
class Mpc_node
{
public:
    

    //FUNCTIONS
    
    void pose0_callback(geometry_msgs::PoseWithCovarianceStamped pose);
    void pose1_callback(geometry_msgs::PoseWithCovarianceStamped pose);
    void calculate_fake_path(double _xr, double _yr, double _wr,double _xp, double _yp,double _wp);
    void publish_new_path(tf::TransformListener* transform_listener, myNLP* mynlp);
    void publish_new_actions(myNLP* mynlp);
    void get_parameters();
    void control_loop();

    //Constructor
    Mpc_node();

private:
    
    

    //--------------  ROS stuff  ------------------------------
    ros::NodeHandle n;
    ros::Rate rate; //10 Hz. T=0.1s
    ros::Publisher pubPath;
    ros::Publisher pubV;
    ros::Subscriber subPose0;
    ros::Subscriber subPose1;
    
    

    //Transform_litener to get Transform robot0->World and store it in T
    tf::TransformListener transform_listener;
    tf::StampedTransform RT_person_robot;
    ros::Time time_start; //Start time of each NLP

    //geometry_msgs::PoseWithCovarianceStamped pose0;
    //geometry_msgs::PoseWithCovarianceStamped pose1;
    nav_msgs::Path path;
    

    //-------------------      ROS PARAM  -----------------------------------------------
    double dist = 3.0; // Desired distance between robot and person
    double ang = 0.0; //Desired tita of the robot
    double yaw = -1.57; //Desired yaw of the camera (Perspective to look at the person)
    double K1 = 1.0; //Weight of distance
    double K2 = 1.0; //Weight of ang
    double K3 = 1.0; //Weigth of yaw
    double K4 = 0.5; //Weigth of yaw
    double K5 = 0.5; //Weigth of yaw    
    double vmax = 1.5;
    double wmax = 0.78;
    bool useGroundTruth = false;

    //-----------------------Object from class myNLP------------------------------
    myNLP mynlp;

    //Robot and person loclalizations
    double xr;
    double yr;
    double titar;
    double xp;
    double yp;
    double titap;

    //ros::WallTime t0,tf,t1,t2; //Meassure computation time






};