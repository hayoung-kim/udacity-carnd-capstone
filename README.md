# udacity-carnd-capstone
This is the project repo for the final project of the Udacity Self-Driving Car Nanodegree: Programming a Real Self-Driving Car. 

# System Architecture
The following is a system architecture diagram showing the ROS nodes and topics used in the project.
![architecture](https://d17h27t6h515a5.cloudfront.net/topher/2017/September/59b6d115_final-project-ros-graph-v2/final-project-ros-graph-v2.png)

## Traffic Light Detection
Since we know the locations of the traffic lights and the vechile, we can get reduce the classification problem to transformation and detection problem. Color is easier to detect in HSV space. In our use case, red light is very important and in HSV space red has two different ranges, since we want to be very sensitive to red light, I include both range in the the mask. Further improments can be made when dealing with unknown locations and complex data by applying Deep NN solutions.

## Waypoint Updater
The purpose of waypoint updater is to update the target velocity property of each waypoint based on traffic light and obstacle detection data. The target veloicty at normal situdation is given from `waypoint_loader` node. If the red light is detected, we genetated stopping trajectory considering vehicle's deceleration limits. 

## Waypoint Follower
The longitudinal target velocity was set in `waypoint_updater` node. This node determine the target yawrate to keep the lane by using pure-pursuit algorithm.

## DBW(Drive-By-Wire) Node
This node finally calculates throttle, brake and steering angle to follow longitudinal and lateral trajectory simultaneously. We used PID controller to calculate throttle and brake based on the difference between the current velocity and the target velocity. 

# Contributors
Our team have three members. 

| Name | E-mail | 
| ------ | ------ | 
| Hayoung Kim (lead) | altairyoung@gmail.com |
| Kyushik Min | kyushikmin@gmail.com |
| Luke Xu | jyxsxu@gmail.com |
| Tejas Varunjikar | tejas2687@gmail.com |
