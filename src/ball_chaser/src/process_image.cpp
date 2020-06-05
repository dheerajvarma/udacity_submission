#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>

// Define a global client that can request services
ros::ServiceClient client;


class SubscribeAndPublish
{
public:
  SubscribeAndPublish()
  {
    //Topic you want to publish
    //pub_ = n_.advertise<PUBLISHED_MESSAGE_TYPE>("/published_topic", 1);

    //Topic you want to subscribe 
    // Subscribe to /camera/rgb/image_raw topic to read the image data inside the process_image_callback function
    sub_ = n_.subscribe("/camera/rgb/image_raw", 10, &SubscribeAndPublish::process_image_callback, this);

  }

  
  ros::NodeHandle n(){return n_;}

  // This function calls the command_robot service to drive the robot in the specified direction
  void drive_robot(float lin_x, float ang_z)
  {
    // TODO: Request a service and pass the velocities to it to drive the robot
     ROS_INFO_STREAM("driving the robot to the ball");

    // Request to drive the robot
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

    bool ball_found = false;

    int i,j,s,e,f; // intializing for height, width, intial, end and final coordinates of the white pixel

    i=j=s=e=f=0;

    // TODO: Loop through each pixel in the image and check if there's a bright white one
    // Then, identify if this pixel falls in the left, mid, or right side of the image
    // Depending on the white ball position, call the drive_bot function and pass velocities to it
    // Request a stop when there's no white ball seen by the camera

    //my_method: finding the average of the first and last white pixels found to find the average position of the ball
    for(i ; i < img.height; i++) {
        for (j=0; j < img.width; j++) {
            if( ( img.data[3 *  (j + ( i * img.width ) ) ] == white_pixel) && ( img.data[(3 *  (j + ( i * img.width ) ) ) + 1 ] == white_pixel) && ( img.data[(3 *  (j + ( i * img.width ) ) ) + 2 ] == white_pixel)) {
                ball_found = true;
		if (s=0){
                	s=j;
 		}
                e=j;
                f= int ((e+s)/2);
            }
        }
    }

    if ( ball_found== true ){

        if ( f <= img.width/3){
            drive_robot(0,0.05);
        }
        else if ( f >= (2 * img.width)/3){
            drive_robot(0,-0.05);
        }
        else{
            drive_robot(0.6,0);
        }
    }

    else {
        drive_robot(0,0);
    }


  }

private:
  ros::NodeHandle n_; 
  ros::Subscriber sub_;

};//End of class SubscribeAndPublish


int main(int argc, char** argv)
{
    // Initialize the process_image node and create a handle to it
    ros::init(argc, argv, "process_image");

    //Create an object of class SubscribeAndPublish that will take care of everything
    SubscribeAndPublish SAPObject;

    // Define a client service capable of requesting services from command_robot
    client = SAPObject.n().serviceClient<ball_chaser::DriveToTarget>("/ball_chaser/command_robot");
 
    // Handle ROS communication events
    ros::spin();

    return 0;
}
