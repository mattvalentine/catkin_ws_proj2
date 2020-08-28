#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>

// Define a global client that can request services
ros::ServiceClient client;

// This function calls the command_robot service to drive the robot in the specified direction
void drive_robot(float lin_x, float ang_z)
{
    ROS_INFO("Driving robot to chase ball.");

    ball_chaser::DriveToTarget srv;
    srv.request.linear_x = lin_x;
    srv.request.angular_z = ang_z;

    if (!client.call(srv))
        ROS_ERROR("Failed to call service command_robot");
}

// This callback function continuously executes and reads the image data
void process_image_callback(const sensor_msgs::Image img)
{

    int white_pixel = 255;
    int column = 0;
    int width = img.step;
    int left_thresh = width / 3;
    int right_thresh = width - left_thresh;

    enum direction {Left, Center, Right, Noball} dir = Noball;

    for (int i = 0; i < img.height * img.step; i++) {
        if (img.data[i] == white_pixel) {
            column = i % width;
            if(column < left_thresh){
                dir = Left;
            } else if(column > right_thresh){
                dir = Right;
            } else {
                dir = Center;
            }
            break;
        }

    }

    switch (dir)
    {
        case Left:   
            drive_robot(0.1,  2.0);
            ROS_INFO("Found Ball - Left");
            break;
        case Center: 
            drive_robot(0.3,  0.0); 
            ROS_INFO("Found Ball - Center");
            break;
        case Right:  
            drive_robot(0.1, -2.0); 
            ROS_INFO("Found Ball - Right");
            break;
        default:     
            drive_robot(0.0,  0.0); 
            ROS_INFO("No ball found.");
            break;
    }
    
}

int main(int argc, char** argv)
{
    // Initialize the process_image node and create a handle to it
    ros::init(argc, argv, "process_image");
    ros::NodeHandle n;

    // Define a client service capable of requesting services from command_robot
    client = n.serviceClient<ball_chaser::DriveToTarget>("/ball_chaser/command_robot");

    // Subscribe to /camera/rgb/image_raw topic to read the image data inside the process_image_callback function
    ros::Subscriber sub1 = n.subscribe("/camera/rgb/image_raw", 10, process_image_callback);

    // Handle ROS communication events
    ros::spin();

    return 0;
}