#include "mpc_node.hpp"

Mpc_node::Mpc_node(): mynlp(vmax,wmax), rate(10){
    //--------------- ROS COMUNICATION -------------------------------------------------
    // Init ROS, Publishers and suscribers
    //----------------------------------------------------------------------------------

    //ros::NodeHandle n;
    //ros::Rate rate(10); //10 Hz. T=0.1s

    ROS_INFO("MPC_Node started");

    //ROS Parameters
    get_parameters();

    subPose0 = n.subscribe("/robot_0/amcl_pose", 10, &Mpc_node::pose0_callback,this);
    subPose1 = n.subscribe("robot_1/amcl_pose", 10, &Mpc_node::pose1_callback,this);

    pubPath = n.advertise<nav_msgs::Path>("robot_0/matlab/path", 10);
    pubV = n.advertise<geometry_msgs::Twist>("robot_0/cmd_vel", 10);

    //Control timer
    control_timer = n.createTimer(control_dt, &Mpc_node::control_Callback,this);
    
    //Record cost
    record_timer = n.createTimer(ros::Duration(0.1), &Mpc_node::record_Callback,this);
    pubCost = n.advertise<std_msgs::Float64>("pubCost",10);

}

void Mpc_node::control_Callback(const ros::TimerEvent&){
       t0 = ros::WallTime::now();

        //Get new params if available
        get_parameters();
          
          
        //Get transform, robot_1 location relative to robot_0
        try {
            transform_listener.waitForTransform("robot_0/base_link", "robot_1/base_link", ros::Time(0), ros::Duration(10.0));
            transform_listener.lookupTransform("robot_0/base_link", "robot_1/base_link", ros::Time(0), RT_person_robot);
        }
        catch (tf::TransformException ex) {
            ROS_ERROR("1st transform orm XX %s", ex.what());
            //continue;
        }
        //Get time when optimal control starts. Use location at this time as reference for the full generated path.
        time_start = ros::Time::now();
        //Calculate person location relative to the robot_0 frame
        xpr = RT_person_robot.getOrigin().x();
        ypr = RT_person_robot.getOrigin().y();
        titapr = tf::getYaw(RT_person_robot.getRotation());

        //----------------------------Solve the nonlinear problem------------------------------------
        
        t1 = ros::WallTime::now();

        //K4 weigths the goal position. If you are far from the objective K4 = 0, to take into account full trajectory. If you are close, weight it a lot.
        //std::cout<<"yaw_error"<<abs(atan2(ypr,xpr)-yaw)<<"\n";
        //std::cout<<"tita_error"<<abs(titapr-ang)<<"\n";
        
        if( abs(atan2(ypr,xpr)-yaw) <= 0.4 and abs(titapr-ang) <= 0.4 and abs(sqrt(xpr*xpr+ypr*ypr) - dist) <= 0.5){
            n.getParam("K_v", K4);
        }else{
            K4 = 0;
        }
        std::cout<<"K4 = "<<K4<<"\n";

        mynlp.my_solve(xpr,ypr,titapr, dist, ang, yaw, K1, K2, K3, K4, K5);    //Solve nlp  

        t2 = ros::WallTime::now();

        publish_new_path(&transform_listener, &mynlp);
        publish_new_actions(&mynlp);

        tf = ros::WallTime::now();
        double exec_time = (tf-t0).toNSec()*1e-6;
        double nlp_time = (t2-t1).toNSec()*1e-6;

        std::cout <<"cylce exec time [ms]: "<<exec_time << "\n";
        std::cout <<"nlp time [ms]: "<<nlp_time << "\n";
}


void Mpc_node::get_parameters(){
    n.getParam("distance", dist);
    n.getParam("angle_tita", ang);
    n.getParam("yaw", yaw);
    n.getParam("K_distance", K1);
    n.getParam("K_ang", K2);
    n.getParam("K_yaw", K3);
    n.getParam("K_v", K4);
    n.getParam("K_w", K5);
    n.getParam("v_max", vmax);
    n.getParam("w_max", wmax);
    n.getParam("useGroundTruth", useGroundTruth);
}

void Mpc_node::pose0_callback(geometry_msgs::PoseWithCovarianceStamped pose){
    ROS_INFO("Pose0 recived");
    //pose0 = pose; 

    xr = pose.pose.pose.position.x;
    yr = pose.pose.pose.position.y;
    titar = tf::getYaw(pose.pose.pose.orientation);

}

void Mpc_node::pose1_callback(geometry_msgs::PoseWithCovarianceStamped pose){
    ROS_INFO("Pose1 recived");
    //pose1 = pose;
    xp = pose.pose.pose.position.x;
    yp = pose.pose.pose.position.y;
    titap = tf::getYaw(pose.pose.pose.orientation);
}

void Mpc_node::calculate_fake_path(double _xr, double _yr, double _wr,double _xp, double _yp,double _wp){

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
    path.header.frame_id= "map";
    pubPath.publish(path);
}

void Mpc_node::publish_new_path(tf::TransformListener* transform_listener, myNLP* mynlp){
    path.poses.resize(N-2);
    for( unsigned i=2; i<mynlp->Xr.size(); ++i){ //I dont send the 0 pose. Start with 1 
        geometry_msgs::PoseStamped poseNew;
        poseNew.pose.position.x = mynlp->Xr[i];
        poseNew.pose.position.y = mynlp->Yr[i];
        poseNew.pose.orientation.w = cos(mynlp->Wr[i]/2);
        poseNew.pose.orientation.z = sin(mynlp->Wr[i]/2);
        poseNew.header.frame_id = "robot_0/base_link"; //Robot_0/base_link but as this node will be inside Robot_0 namespace, you can say directly base_link

        geometry_msgs::PoseStamped poseMap;

        try{
            transform_listener->waitForTransform("map", "robot_0/base_link",time_start, ros::Duration(2.0));
            transform_listener->transformPose("map", time_start, poseNew, "map", poseMap);

        }catch(tf::TransformException ex){
            ROS_ERROR("orm XX %s", ex.what());
        }
        

        path.poses[i-2] = poseMap;
    }
    path.header.frame_id= "map";
    pubPath.publish(path);
}

void Mpc_node::publish_new_actions(myNLP* mynlp){
    geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.x = mynlp->Vr[0];
    cmd_vel.angular.z = mynlp->Wr[0];
    pubV.publish(cmd_vel);
}

void Mpc_node::record_Callback(const ros::TimerEvent&){
    Jd += K1*pow((sqrt(pow(xpr,2)+pow(ypr,2))-dist),2);
    Jang += K2*pow((titapr-ang),2);
    Jyaw += K3*pow((atan2(-ypr,-xpr)-yaw+titapr),2);

    std_msgs::Float64 cost;
    cost.data = Jd + Jang + Jyaw;
    pubCost.publish(cost);
}