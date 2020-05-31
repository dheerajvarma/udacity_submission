#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
//TODO: Include the ball_chaser "DriveToTarget" header file
#include "ball_chaser/DriveToTarget.h"


// TODO: Create a handle_drive_request callback function that executes whenever a drive_bot service is requested
// This function should publish the requested linear x and angular velocities to the robot wheel joints
// After publishing the requested velocities, a message feedback should be returned with the requested wheel velocities

class SubscribeAndPublish
{
public:
  SubscribeAndPublish()
  {
    // Inform ROS master that we will be publishing a message of type geometry_msgs::Twist on the robot actuation topic with a publishing queue size of 10
    motor_command_publisher = n_.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

  }

  ros::NodeHandle n(){return n_;}

  bool handle_drive_request(ball_chaser::DriveToTarget::Request& req,
    ball_chaser::DriveToTarget::Response& res)
  {

    ROS_INFO("DriveToTargetRequest received - l:%1.2f, a:%1.2f", (float)req.linear_x, (float)req.angular_z);


    // Create a motor_command object of type geometry_msgs::Twist
    geometry_msgs::Twist motor_command;
    
        

    // Set wheel velocities, forward [0.5, 0.0] 
    motor_command.linear.x = req.linear_x;
    motor_command.angular.z = req.angular_z;
    // Publish angles to drive the robot
    motor_command_publisher.publish(motor_command);

    // Return a response message
    res.msg_feedback = "linear and angular velocities requested - linear: " + std::to_string(req.linear_x) + " , angular: " + std::to_string(req.angular_z);
    ROS_INFO_STREAM(res.msg_feedback);

    return true;
  }

private:
  ros::NodeHandle n_; 
  ros::Publisher motor_command_publisher;

};//End of class SubscribeAndPublish

int main(int argc, char** argv)
{
  //Initiate ROS
  ros::init(argc, argv, "drive_bot");

  //Create an object of class SubscribeAndPublish that will take care of everything
  SubscribeAndPublish SAPObject;

  // Define a safe_move service with a handle_safe_move_request callback function
  ros::ServiceServer service = SAPObject.n().advertiseService("/ball_chaser/command_robot", &SubscribeAndPublish::handle_drive_request, &SAPObject);
  ROS_INFO("Ready to send joint commands");
  
  // Handle ROS communication events
  ros::spin();

  return 0;
}
