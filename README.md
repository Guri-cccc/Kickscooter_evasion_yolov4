# Kickscooter_evasion_yolov4
Avoiding kick-scooter of mobile robot using yolo v4

Package discriptions:


yolov4-for-darknet_ros: yolov4 for kick scooter detection

coordinate: coordinate estimation with realsense rdgd camera

move_node: instructions for the mobile robot

dr_naviagtion: driver for the mobile robot

teleop_twist_joy: toggle for the mobile robot (menual mode and auto mode)


To run the codes

1. Kick scooter detection

roslaunch darknet_ros yolo_v4_kick_6000.launch

roslaunch realsense2_camera rs_camera.launch

rosrun coordinate coordinate_estimation

2. Move the mobile robot

rosrun dr_driver dr_driver.py

roslaunch differential_joy.launch

rosrun move move_node
