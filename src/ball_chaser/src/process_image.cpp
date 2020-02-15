#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>

class ProcessImage
{
public:
  ProcessImage()
  {
    client_ = n_.serviceClient<ball_chaser::DriveToTarget>("/ball_chaser/command_robot");
    sub_ = n_.subscribe("/camera/rgb/image_raw", 10, &ProcessImage::process_image_callback, this);
  }

  // Continuously executes and reads the image data
  void process_image_callback(const sensor_msgs::Image img)
  {
    int white_pixel = 255;
    int one_third = img.step / 3;
    int two_third = img.step * 2 / 3;
    presence_ = false;
    int i = 0;
    while (i < img.height*img.step) {
      if (img.data[i] == white_pixel && img.data[i+1] == white_pixel && img.data[i+2] == white_pixel) {
        presence_ = true;
        if (i % img.step < one_third)
          pos_ = left;
        else if (i % img.step < two_third)
          pos_ = mid;
        else
          pos_ = right;

        break;
      }
      i += 3;
    }

    float lin_x;
    float ang_z;

    if (!presence_) {
      lin_x = 0.0;
      ang_z = 0.0;
    } else {
      switch (pos_) {
        case left:
          lin_x = 0.0;
          ang_z = 0.2;
          break;
        case mid:
          lin_x = 0.5;
          ang_z = 0.0;
          break;
        default:
          lin_x = 0.0;
          ang_z = -0.2;
          break;
      }
    }

    drive_robot(lin_x, ang_z);
  }

  // Calls the command_robot service to drive the robot in the specified direction
  void drive_robot(float lin_x, float ang_z)
  {
    ball_chaser::DriveToTarget srv;
    srv.request.linear_x = lin_x;
    srv.request.angular_z = ang_z;
    if (!client_.call(srv))
      ROS_ERROR("Failed to call service command_robot");
  }


private:
  ros::NodeHandle n_;
  ros::Subscriber sub_;
  ros::ServiceClient client_;
  enum Position { left, mid, right };
  bool presence_{false};
  Position pos_{mid};
};

int main(int argc, char** argv)
{
  // Initialize the process_image node
  ros::init(argc, argv, "process_image");

  ProcessImage PIObject;

  // Handle ROS communication events
  ros::spin();

  return 0;
}
