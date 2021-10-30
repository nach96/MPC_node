# MPC_node
ROS MPC node based on IpOpt and CppAD library for differential drive robot following a person at given:
-Distance
-Perspective
-Relative angle

Workflow for simulation:
-roscore
-roslaunch simulation control-2-robots-empty.launch  (Launches stage, map_server and fake_localization nodes.)
-roslaunch mpc_node mpc-node-2-robots-MPC.launch     (MPC control of the robot)

*Debugging and visualization:

-rqt_bag   		(Save topics publsihed in a bagfile)
-PlotJuggler  		(Plot timeseries data easily. Online or from bagfile. Can re-publish the topics for rviz)
-rosrun rviz rviz	(Visualization of data over map)
