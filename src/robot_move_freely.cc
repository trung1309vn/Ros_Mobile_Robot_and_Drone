/*
 * Copyright (C) 2018 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/
#include <chrono>
#include <stdlib.h>
#include <time.h> 
#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <std_srvs/SetBool.h>
#include <rosgraph_msgs/Clock.h>
#include <sensor_msgs/LaserScan.h>
#include <typeinfo>
#include <iostream>
#include <string>
#include <typeinfo>
#include <vector>

class Listener
{
  public: 
    sensor_msgs::LaserScan laser;
    void callback(const sensor_msgs::LaserScan::ConstPtr& msg)
    {
      laser = *msg;
    }
};

/// \brief. Example control class, running as a ROS node to control a robot.
class Controller
{
  /// \brief Constructor.
  /// The controller uses the given name as a prefix of its topics, e.g.,
  /// "/x1/cmd_vel" if _name is specified as "x1".
  /// \param[in] _name Name of the robot.
  public: Controller();

  /// \brief A function that will be called every loop of the ros spin
  /// cycle.
  public: void Update();

  /// \brief ROS node handler.
  private: ros::NodeHandle n;

  /// \brief publisher to send cmd_vel
  public: ros::Publisher velPub;

  /// \brief subscriber to get laser_scan
  public: ros::Subscriber laserSub;

  /// \brief listener to get laser_scan
  public: Listener listener;

  /// \brief Client to request pose from origin.
  private: ros::ServiceClient originClient;

  /// \brief True if started.
  private: bool started{false};

  /// \brief Last time a comms message to another robot was sent.
  private: std::chrono::time_point<std::chrono::system_clock> lastMsgSentTime;

  /// \brief Name of this robot.
  private: std::string name;
};

/////////////////////////////////////////////////
Controller::Controller()
{
  ROS_INFO("Waiting for /clock, /subt/start");

  ros::topic::waitForMessage<rosgraph_msgs::Clock>("/clock", this->n);

  // Wait for the start service to be ready.
  ros::service::waitForService("/subt/start", -1);

  // Get Param
  std::string _name = "X1";
  this->name = _name;
  ROS_INFO("Using robot name[%s]\n", this->name.c_str());
}

/////////////////////////////////////////////////
void Controller::Update()
{
  if (!this->started)
  {
    // Send start signal
    std_srvs::SetBool::Request req;
    std_srvs::SetBool::Response res;
    req.data = true;
    if (!ros::service::call("/subt/start", req, res))
    {
      ROS_ERROR("Unable to send start signal.");
    }
    else
    {
      ROS_INFO("Sent start signal.");
      this->started = true;
    }

    if (this->started)
    {
      // Create a cmd_vel publisher to control a vehicle.
      this->velPub = this->n.advertise<geometry_msgs::Twist>(
          this->name + "/cmd_vel", 1);

      // Create a laser_scan subscriber.
      this->laserSub = this->n.subscribe<sensor_msgs::LaserScan>(
          this->name + "/front_scan", 1000, &Listener::callback, &this->listener);
    }
    else
      return;
  }

  // Add code that should be processed every iteration.

  std::chrono::time_point<std::chrono::system_clock> now =
    std::chrono::system_clock::now();
  sensor_msgs::LaserScan laser_scan = this->listener.laser;
  if (!laser_scan.ranges.empty())
  {
    // Simple example for robot to go to entrance
    geometry_msgs::Twist msg;
    // Set direction for robot
    double linVel = 1.0;
    double angVel = 3.14159265 / 4;
    int direction = 0; // 0: forward, -1: left, 1: right;
    if (laser_scan.ranges[359] > laser_scan.ranges[119] && laser_scan.ranges[359] > laser_scan.ranges[599])
    {
      direction = 0;
      ROS_INFO("Distance [%f], Direction [%d]", laser_scan.ranges[359], direction);
    }
    else if (laser_scan.ranges[119] > laser_scan.ranges[359] && laser_scan.ranges[119] > laser_scan.ranges[599])
    {
      direction = 1;
      ROS_INFO("Distance [%f], Direction [%d]", laser_scan.ranges[119], direction);
    }
    else if (laser_scan.ranges[599] > laser_scan.ranges[359] && laser_scan.ranges[599] > laser_scan.ranges[119])
    {
      direction = -1;
      ROS_INFO("Distance [%f], Direction [%d]", laser_scan.ranges[599], direction);
    }
    else if (laser_scan.ranges[359] < laser_scan.ranges[119])
    {
      direction = (rand() > RAND_MAX/2) ? -1: 1;
      ROS_INFO("Distance [%f], Direction [%d]", laser_scan.ranges[119], direction);
    }
    else
    {
      direction = 0;
      ROS_INFO("Distance [%f], Direction [%d]", laser_scan.ranges[359], direction);
    }

    switch (direction)
    {
      case -1:
      {
        msg.angular.z = angVel;
        msg.linear.x = 0;
        break;
      }
      case 1:
      {
        msg.angular.z = -angVel;
        msg.linear.x = 0;
        break;
      }
      case 0:
      {
        msg.linear.x = linVel;
      }
    }
    ROS_INFO("Rotation: %f, Linear: %f", msg.angular.z, msg.linear.x);
    this->velPub.publish(msg);
  }
}

/////////////////////////////////////////////////
int main(int argc, char** argv)
{
  // Initialize ros
  ros::init(argc, argv, "robot_free");

  ROS_INFO("Starting seed competitor\n");

  // Create the controller
  Controller controller;

  // This sample code iteratively calls Controller::Update. This is just an
  // example. You can write your controller using alternative methods.
  // To get started with ROS visit: http://wiki.ros.org/ROS/Tutorials
  ros::Rate loop_rate(1);
  while (ros::ok())
  {
    controller.Update();
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
