#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "ball_chaser/DriveToTarget.h"


class DriveBot
{
public:
  DriveBot()
  {
    // Inform ROS master that we will be publishing a message of type geometry_msgs::Twist on the robot actuation topic with a publishing queue size of 10
    motor_command_publisher_ = n_.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    service_ = n_.advertiseService("/ball_chaser/command_robot", &DriveBot::handle_drive_request, this);
  }

  // Publish the requested linear x and angular velocities to the robot wheel joints
  // After publishing the requested velocities, a message feedback should be returned with the requested wheel velocities
  bool handle_drive_request(ball_chaser::DriveToTarget::Request& req,
                            ball_chaser::DriveToTarget::Response& res)
  {
    ROS_INFO("DriveToTargetRequest received - linear_x:%1.2f, angular_z:%1.2f", (float)req.linear_x, (float)req.angular_z);

    // Create a motor_command object of type geometry_msgs::Twist
    geometry_msgs::Twist motor_command;

    motor_command.linear.x = req.linear_x;
    motor_command.angular.z = req.angular_z;
    // Publish angles to drive the robot
    motor_command_publisher_.publish(motor_command);

    res.msg_feedback = "linear_x set: " + std::to_string(req.linear_x) + " , angular_z: " + std::to_string(req.angular_z);
    ROS_INFO_STREAM(res.msg_feedback);

    return true;
  }

private:
  ros::Publisher motor_command_publisher_;
  ros::NodeHandle n_;
  ros::ServiceServer service_;
};


int main(int argc, char** argv)
{
  // Initialize a ROS node
  ros::init(argc, argv, "drive_bot");

  DriveBot DBObject;

  ros::spin();

  return 0;
}
