// Matt Franklin Project 2 part 13

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "ball_chaser/DriveToTarget.h"

// ROS::Publisher motor commands;
ros::Publisher motor_command_publisher;

bool handle_drive_request(ball_chaser::DriveToTarget::Request& req,
    ball_chaser::DriveToTarget::Response& res)
    {
    ROS_INFO(
        "Processing drive request - speed: %1.2f, direction: %1.2f", 
        (float)req.linear_x, (float)req.angular_z
        );

    geometry_msgs::Twist motor_command;
    motor_command.linear.x = (float)req.linear_x;
    motor_command.angular.z = (float)req.angular_z;
    motor_command_publisher.publish(motor_command);
    
    res.msg_feedback = "Confirmed: Speed set: " + std::to_string(motor_command.linear.x) 
                        + ", Angle set: " + std::to_string(motor_command.angular.z);

    ROS_INFO_STREAM(res.msg_feedback);

    return true;
}

int main(int argc, char** argv)
{
    // Initialize a ROS node
    ros::init(argc, argv, "drive_bot");

    // Create a ROS NodeHandle object
    ros::NodeHandle n;

    motor_command_publisher = n.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

    ros::ServiceServer command_robot_server = n.advertiseService("/ball_chaser/command_robot", handle_drive_request);

    ros::spin();

    return 0;
}