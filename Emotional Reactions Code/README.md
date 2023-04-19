# MIE-443-Contest-3
Follow Me Robot Companion

Follow the instructions below to run the code on a TurtleBot:

In separate terminals run the following commands -
1) catkin_make
2) roslaunch turtlebot_bringup minimal.launch
3) rosrun sound_play soundplay_node.py
4) roslaunch turtlebot_follower follower.launch

The robot can display 4 different emotions:
1) Sadness - Triggered by losing the person it is following.
2) Anger - Triggered by hitting any one of its bumpers.
3) Fear - Triggered by tilting the robot such that one of its wheels is off the floor.
4) Infatuation - Triggered by lifting the robot off the floor. 
