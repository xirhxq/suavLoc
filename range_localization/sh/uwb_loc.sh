roscore &
sleep 2 &
source catkin_ws/devel/setup.bash
roslaunch xsens_mti_driver xsens_mti300_node.launch &
sleep 2 &
roslaunch range_localization range_localization.launch &
sleep 10 &
rviz -d ~/ros/mbzirc_ws/src/range_localization/rviz_cfg/uwb_loc.rviz 
# rosrun rqt_plot rqt_plot /pid_debug/P:I:D:PID &
# rosrun rqt_plot rqt_plot /pid_debug/Error &
# rqt_plot /predict_odom/pose/pose/position:x:y:z &
# rqt_plot /extrinsic_lb/pose/position:x:y:z &
# rqt_plot /debug/imu_rot/vector/y /debug/laser_rot/vector/y