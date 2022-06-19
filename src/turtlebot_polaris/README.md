## Requirement

This project is built in an Ubuntu 16.04 and ROS Kinetic environment. Besides, this project requires turtlebot packages.

`sudo apt-get install ros-kinetic-turtlebot ros-kinetic-turtlebot-apps ros-kinetic-turtlebot-interactions ros-kinetic-kobuki-ftdi ros-kinetic-ar-track-alvar-msgs ros-kinetic-turtlebot-simulator`

## Implementation

1.  Place the Turtlebot in anywhere on Earth.
2.  Calculate the bearing between the robot position and true north.
3.  Publish the message and rotate the robot to true north

## Execution

1.  Launch the Turtlebot in gazebo environment.  
    `roslaunch turtlebot_gazebo turtlebot_world.launch`
2.  Run turtlebot-polaris node.  
    `rosrun turtlebot_polaris ttlebot_polaris_node.py`
3.  Run the place\_turtlebot\_server to place turtlebot anywhere on Earth.  
    `rosrun turtlebot_polaris pce_turtlebot_server.py`
4.  Turtlebot will rotate to Polaris automatically
