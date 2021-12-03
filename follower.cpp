/*
 * Copyright (c) 2011, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Image.h>
#include <visualization_msgs/Marker.h>
#include <turtlebot_msgs/SetFollowState.h>

#include "dynamic_reconfigure/server.h"
#include "turtlebot_follower/FollowerConfig.h"

#include <depth_image_proc/depth_traits.h>

#include "apriltag_ros/AprilTagDetection.h"
#include "apriltag_ros/AprilTagDetectionArray.h"

#include "kobuki_msgs/Sound.h"

namespace turtlebot_follower
{

//* The turtlebot follower nodelet.
/**
 * The turtlebot follower nodelet. Subscribes to point clouds
 * from the 3dsensor, processes them, and publishes command vel
 * messages.
 */
class TurtlebotFollower : public nodelet::Nodelet
{
public:
  /*!
   * @brief The constructor for the follower.
   * Constructor for the follower.
   */
  TurtlebotFollower() : min_y_(0.1), max_y_(0.5),
                        min_x_(-0.2), max_x_(0.2),
                        max_z_(5.0), goal_z_(0.5),
                        z_scale_(1.0), x_scale_(5.0)
  {

  }

  ~TurtlebotFollower()
  {
    delete config_srv_;
  }

private:
  double min_y_; /**< The minimum y position of the points in the box. */
  double max_y_; /**< The maximum y position of the points in the box. */
  double min_x_; /**< The minimum x position of the points in the box. */
  double max_x_; /**< The maximum x position of the points in the box. */
  double max_z_; /**< The maximum z position of the points in the box. */
  double goal_z_; /**< The distance away from the robot to hold the centroid */
  double z_scale_; /**< The scaling factor for translational robot speed */
  double x_scale_; /**< The scaling factor for rotational robot speed */
  bool   enabled_; /**< Enable/disable following; just prevents motor commands */

  // Service for start/stop following
  ros::ServiceServer switch_srv_;

  // Dynamic reconfigure server
  dynamic_reconfigure::Server<turtlebot_follower::FollowerConfig>* config_srv_;

  /*!
   * @brief OnInit method from node handle.
   * OnInit method from node handle. Sets up the parameters
   * and topics.
   */
  virtual void onInit()
  {
    ros::NodeHandle& nh = getNodeHandle();
    ros::NodeHandle& private_nh = getPrivateNodeHandle();

    private_nh.getParam("min_y", min_y_);
    private_nh.getParam("max_y", max_y_);
    private_nh.getParam("min_x", min_x_);
    private_nh.getParam("max_x", max_x_);
    private_nh.getParam("max_z", max_z_);
    private_nh.getParam("goal_z", goal_z_);
    private_nh.getParam("z_scale", z_scale_);
    private_nh.getParam("x_scale", x_scale_);
    private_nh.getParam("enabled", enabled_);

    soundpub_ = private_nh.advertise<kobuki_msgs::Sound>("/mobile_base/commands/sound",1);
    cmdpub_ = private_nh.advertise<geometry_msgs::Twist> ("cmd_vel", 1);
    markerpub_ = private_nh.advertise<visualization_msgs::Marker>("marker",1);
    bboxpub_ = private_nh.advertise<visualization_msgs::Marker>("bbox",1);
    sub_ = nh.subscribe<apriltag_ros::AprilTagDetectionArray>("/tag_detections", 1, &TurtlebotFollower::imagecb, this);

    switch_srv_ = private_nh.advertiseService("change_state", &TurtlebotFollower::changeModeSrvCb, this);

    config_srv_ = new dynamic_reconfigure::Server<turtlebot_follower::FollowerConfig>(private_nh);
    dynamic_reconfigure::Server<turtlebot_follower::FollowerConfig>::CallbackType f =
        boost::bind(&TurtlebotFollower::reconfigure, this, _1, _2);
    config_srv_->setCallback(f);
  }

  void reconfigure(turtlebot_follower::FollowerConfig &config, uint32_t level)
  {
    min_y_ = config.min_y;
    max_y_ = config.max_y;
    min_x_ = config.min_x;
    max_x_ = config.max_x;
    max_z_ = config.max_z;
    goal_z_ = config.goal_z;
    z_scale_ = config.z_scale;
    x_scale_ = config.x_scale;
  }

  /*!
   * @brief Callback for point clouds.
   * Callback for depth images. It finds the centroid
   * of the points in a box in the center of the image. 
   * Publishes cmd_vel messages with the goal from the image.
   * @param cloud The point cloud message.
   */

  double timeOfStop = 0, timeOfStart = 0;
  double timeStopped = -10.0, timeMoving = -10.0;
  int startup = 1;
  float longPause = 1.0, shortPause = 0.5;

  void imagecb(const apriltag_ros::AprilTagDetectionArray::ConstPtr& depth_msg)
  {
    int n = 0;

    float x = 0;
    float y = 0;
    float z = 0;
    
    max_z_ = 5.0;
    goal_z_ = 0.5;
    
    z_scale_ = 1.0;
    x_scale_ = 5.0;

    double currentTime;

    // checking that a tag has been detected
    n = depth_msg->detections.size();

    //ROS_INFO_THROTTLE(1, "Number of tags detected: %d", n);
    ROS_INFO("Number of tags detected: %d", n);

    if (n>0)
    {
      // Checks if the turtlebot has been stopped for at least 1 second or is first starting up
      // If so, plays sound indicating start of movement
      if (timeStopped > shortPause and timeMoving > shortPause)
      {
	ROS_INFO("Playing sound, time stopped was %f and time moving is %f", timeStopped, timeMoving);
	kobuki_msgs::SoundPtr soundPlay(new kobuki_msgs::Sound());
	soundPlay->value = 6;
	soundpub_.publish(soundPlay);
      }
      if (timeMoving < 0)
      {
        timeOfStart = (depth_msg->header.stamp.sec) + (depth_msg->header.stamp.nsec)*1e-9;
      }
      // If turtlebot was previously stopped, resets stopped time and records timeOfStart
      if (timeStopped != 0 and timeMoving == 0)
      {
        timeOfStart = (depth_msg->header.stamp.sec) + (depth_msg->header.stamp.nsec)*1e-9;
	ROS_INFO("If 1 activated timeOfStart: %f, time stopped was %f and time moving is %f", timeOfStart, timeStopped, timeMoving);
      }
      else if (timeStopped != 0 and timeMoving > shortPause)
      {
        timeStopped = 0;

	ROS_INFO("If 2.1 activated timeOfStart: %f, time stopped was %f and time moving is %f", timeOfStart, timeStopped, timeMoving);
      }

      // updates time turtlbot has been moving
      currentTime = (depth_msg->header.stamp.sec) + (depth_msg->header.stamp.nsec)*1e-9;
      timeMoving = currentTime - timeOfStart + 1e-5;
      ROS_INFO("currentTime: %f, timeOfStart: %f, and timeMoving: %f", currentTime, timeOfStart, timeMoving);

      // getting average x, y, and z coordinates relative to the camera of all the tags detected from tag_detection topic
      for (int i=0; i<n; i++)
      {
        x += (depth_msg->detections[i].pose.pose.pose.position.x)/n;
        y += (depth_msg->detections[i].pose.pose.pose.position.y)/n;
        z += (depth_msg->detections[i].pose.pose.pose.position.z)/n;
      }

      // stops the robot because the tags deteceted are too far away
      if(z > max_z_){
        ROS_INFO_THROTTLE(1, "Average tag center is too far away %f, stopping the robot", z);
        if (enabled_)
        {
          cmdpub_.publish(geometry_msgs::TwistPtr(new geometry_msgs::Twist()));
        }
        return;
      }

      ROS_INFO_THROTTLE(1, "Average tag center is at x: %f, y: %f, z: %f with %d tags", x, y, z, n);
      publishMarker(x, y, z);

      // publishes the new velocity commands
      if (enabled_)
      {
        geometry_msgs::TwistPtr cmd(new geometry_msgs::Twist());
        cmd->linear.x = (z - goal_z_) * z_scale_;
        cmd->angular.z = -x * x_scale_;
        cmdpub_.publish(cmd);
      }
    }
    else
    {
      ROS_INFO_THROTTLE(1, "No tags detected(%d), stopping the robot", n);
      publishMarker(x, y, z);

      // Checks if the turtlebot has been moving for 1 second
      // If so, plays sound indicating stopping of movement
      // If turtlebot was previously moving, resets moving time and records timeOfStop
      if (timeMoving > shortPause and timeStopped > shortPause)
      {
	ROS_INFO("Playing sound, time moving was %f and time stopped is %f", timeMoving, timeStopped);
	kobuki_msgs::SoundPtr soundPlay(new kobuki_msgs::Sound());
	soundPlay->value = 5;
	soundpub_.publish(soundPlay);
      }

      if (timeStopped < 0)
      {
        timeOfStop = (depth_msg->header.stamp.sec) + (depth_msg->header.stamp.nsec)*1e-9;
      }
      if (timeMoving != 0 and timeStopped == 0)
      {
        timeOfStop = (depth_msg->header.stamp.sec) + (depth_msg->header.stamp.nsec)*1e-9;
	ROS_INFO("If 1 activated timeOfStop: %f, time moving was %f and time stopped is %f", timeOfStop, timeMoving, timeStopped);
      }
      else if (timeMoving != 0 and timeStopped > shortPause)
      {
        timeMoving = 0;

	ROS_INFO("If 2.1 activated timeOfStop: %f, time moving was %f and time stopped is %f", timeOfStop, timeMoving, timeStopped);
      }
      

      // updates time turtlebot has been stopped
      currentTime = (depth_msg->header.stamp.sec) + (depth_msg->header.stamp.nsec)*1e-9;
      timeStopped = currentTime - timeOfStop + 1e-5;
      ROS_INFO("currentTime: %f, timeOfStop: %f, and timeStopped: %f", currentTime, timeOfStop, timeStopped);

      // splits stopped time into seconds and nanoseconds
      double timeStoppedSec, timeStoppedNsec;
      timeStoppedNsec = modf(timeStopped, &timeStoppedSec);

      ROS_INFO("Current time is %f, time of stop is %f, and time stopped is %f", currentTime, timeOfStop, timeStopped);
      //ROS_INFO_THROTTLE(1, "Current time is %f, time of stop is %f, and time stopped is %f", currentTime, timeOfStop, timeStopped);
      if (enabled_)
      {
        // if turtlebot has been stopped more than five seconds start slowly spinning
        // timeStoppedNsec term cause robot to spin for portions of a second and stop the rest of the time
        // the intermittent motion allows the turtlebot to better register April tags
        if (timeStopped >= 5 && timeStoppedNsec < 0.5)
        {
          geometry_msgs::TwistPtr cmd(new geometry_msgs::Twist());
          cmd->angular.z = 1.0;
          cmdpub_.publish(cmd);
        }
        else
        {
          cmdpub_.publish(geometry_msgs::TwistPtr(new geometry_msgs::Twist()));
        }
      }
    }

    publishBbox();
  }

  bool changeModeSrvCb(turtlebot_msgs::SetFollowState::Request& request,
                       turtlebot_msgs::SetFollowState::Response& response)
  {
    if ((enabled_ == true) && (request.state == request.STOPPED))
    {
      ROS_INFO("Change mode service request: following stopped");
      cmdpub_.publish(geometry_msgs::TwistPtr(new geometry_msgs::Twist()));
      enabled_ = false;
    }
    else if ((enabled_ == false) && (request.state == request.FOLLOW))
    {
      ROS_INFO("Change mode service request: following (re)started");
      enabled_ = true;
    }

    response.result = response.OK;
    return true;
  }

  void publishMarker(double x,double y,double z)
  {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "/camera_rgb_optical_frame";
    marker.header.stamp = ros::Time();
    marker.ns = "my_namespace";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = x;
    marker.pose.position.y = y;
    marker.pose.position.z = z;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.2;
    marker.scale.y = 0.2;
    marker.scale.z = 0.2;
    marker.color.a = 1.0;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    //only if using a MESH_RESOURCE marker type:
    markerpub_.publish( marker );
  }

  void publishBbox()
  {
    double x = (min_x_ + max_x_)/2;
    double y = (min_y_ + max_y_)/2;
    double z = (0 + max_z_)/2;

    double scale_x = (max_x_ - x)*2;
    double scale_y = (max_y_ - y)*2;
    double scale_z = (max_z_ - z)*2;

    visualization_msgs::Marker marker;
    marker.header.frame_id = "/camera_rgb_optical_frame";
    marker.header.stamp = ros::Time();
    marker.ns = "my_namespace";
    marker.id = 1;
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = x;
    marker.pose.position.y = -y;
    marker.pose.position.z = z;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = scale_x;
    marker.scale.y = scale_y;
    marker.scale.z = scale_z;
    marker.color.a = 0.5;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    //only if using a MESH_RESOURCE marker type:
    bboxpub_.publish( marker );
  }

  ros::Subscriber sub_;
  ros::Publisher soundpub_;
  ros::Publisher cmdpub_;
  ros::Publisher markerpub_;
  ros::Publisher bboxpub_;
};

PLUGINLIB_EXPORT_CLASS(turtlebot_follower::TurtlebotFollower, nodelet::Nodelet)

}
