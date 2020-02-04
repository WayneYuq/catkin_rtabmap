# gazebo_ros_robot_demo

## Usage
1. Pull the code and go into the top level directory of this project.
2. `catkin_make`
3. `source devel/setup.bash`
4. Launch the robot inside your world 
5. `roslaunch my_robot world.launch`
6. Run drive_bot and process_image node 
7. open a new terminal and go into the top level again
8. `source devel/setup.bash`
9. `roslaunch ball_chaser ball_chaser.launch`
10. Visualize the image sensor
11. open a new terminal and go into the top level again
12. `source devel/setup.bash`
13. `rosrun rqt_image_view rqt_image_view`
14. move the write ball to the view field of the robot
