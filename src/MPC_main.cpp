#include "mpc_node.cpp"
#include "ros/ros.h"


int main(int argc, char **argv){
    ros::init(argc,argv,"MPC_node");
    Mpc_node my_mpc_node;
   // my_mpc_node.control_loop();
   ros::spin();
   

    return 0;
    

}