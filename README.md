# Improved Turtlebot Follower
Group Final Project for the Intro to Robotics Lab

Members:
* Patrick Herke
* Julius Pallotta
* Evan Nguyen

Compared to the original code this project has four major changes.
* Reduced distractability of the following beahvior.
* Increased range at which it can pick up a target by 2x.
* Incremental rotation in place to find new target if no target is seen for five seconds.
* Addition of sounds to indicate stopping and starting of movement.

The advantages in distractability and range both come from using AprilTags rather than a depth point cloud to determine the location of the object to follow.
By using AprilTags the robot doesn't get distracted by other people or objects coming into its field of view. It can also pick up objects at further range because it just needs to pick up the tag rather than enough points to find a centroid.\
The incremental rotation uses the timestamps of the tag_detections topic to determine time since the robot has last seen a tag.\
The sounds that indicate stopping and starting of movement are from the default turtlebot sounds. They are accessed by publishing to the /mobile_base/commands/sound topic.

To modify the standard turtlebot package installation to run the improved follower code the following steps must be taken.

1. Install and set up the apriltag_ros package.
2. Replace the old follower.cpp and follower.launch files with their new versions.

## Step 1
Install the apriltag_ros package into the same catkin workspace as the other turtlebot packages.\
For help, access the apriltag_ros package [ROS wiki](http://wiki.ros.org/apriltag_ros) or [github repository](https://github.com/AprilRobotics/apriltag_ros).

From the apriltag_ros package modify the following files.
* /config/setting.yaml
* /config/tags.yaml

Check to make sure the settings.yaml file lists the tag family that you plan to use.\
For the tags.yaml file, add all the tag ids you want to detect and their sizes to the the standalone_tags portion.

For our testing we used tag family: 36h11, tag id: 0, and size: 0.08m.\
If you want to use the same settings you can simply replace the setting.yaml and tags.yaml files with the files of the same name located in the apriltag_ros_config folder of this repository.
However, you can select any other tag family or tag ids and the follower code will still function.


## Step 2  
In the turtlebot_follower package replace the
* /src/follower.cpp
* /launch/follower.launch

files with the
* follower.cpp
* follower.launch

files from this repository.

## Running the Follower Code
Be sure to build and source your workspace after making the above changes.

Run the minimal.launch file to get the turtlebot up and running.
```
roslaunch turtlebot_bringup minimal.launch
```

Then run the follower.launch file to start the follower.
```
roslaunch turtlebot_follower follower.launch
```
