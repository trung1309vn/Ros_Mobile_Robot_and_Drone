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

#include <string>

#include <subt_communication_broker/subt_communication_client.h>
#include <subt_ign/CommonTypes.hh>

class Listener
{
  public: 
    sensor_msgs::LaserScan laser_scan;
    void callback(const sensor_msgs::LaserScan::ConstPtr& msg)
    {
      laser_scan = *msg;
    }
}

/// \brief. Example control class, running as a ROS node to control a robot.
class Controller
{
  /// \brief Constructor.
  /// The controller uses the given name as a prefix of its topics, e.g.,
  /// "/x1/cmd_vel" if _name is specified as "x1".
  /// \param[in] _name Name of the robot.
  public: Controller(const std::string &_name);

  /// \brief A function that will be called every loop of the ros spin
  /// cycle.
  public: void Update();

  /// \brief Callback function for message from other comm clients.
  /// \param[in] _srcAddress The address of the robot who sent the packet.
  /// \param[in] _dstAddress The address of the robot who received the packet.
  /// \param[in] _dstPort The destination port.
  /// \param[in] _data The contents the packet holds.
  private: void CommClientCallback(const std::string &_srcAddress,
                                   const std::string &_dstAddress,
                                   const uint32_t _dstPort,
                                   const std::string &_data);

  /// \brief ROS node handler.
  private: ros::NodeHandle n;

  /// \brief publisher to send cmd_vel
  public: ros::Publisher velPub;

  /// \brief Communication client.
  private: std::unique_ptr<subt::CommsClient> client;

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
Controller::Controller(const std::string &_name)
{
  ROS_INFO("Waiting for /clock, /subt/start");

  ros::topic::waitForMessage<rosgraph_msgs::Clock>("/clock", this->n);

  // Wait for the start service to be ready.
  ros::service::waitForService("/subt/start", -1);
  this->name = _name;
  ROS_INFO("Using robot name[%s]\n", this->name.c_str());
}

/////////////////////////////////////////////////
void Controller::CommClientCallback(const std::string &_srcAddress,
                                    const std::string &_dstAddress,
                                    const uint32_t _dstPort,
                                    const std::string &_data)
{
  // Add code to handle communication callbacks.
  ROS_INFO("Message from [%s] to [%s] on port [%u]:\n", _srcAddress.c_str(),
      _dstAddress.c_str(), _dstPort);
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
      // Create subt communication client
      this->client.reset(new subt::CommsClient(this->name));
      this->client->Bind(&Controller::CommClientCallback, this);

      // Create a cmd_vel publisher to control a vehicle.
      this->velPub = this->n.advertise<geometry_msgs::Twist>(
          this->name + "/cmd_vel", 1);
    }
    else
      return;
  }

  // Add code that should be processed every iteration.

  std::chrono::time_point<std::chrono::system_clock> now =
    std::chrono::system_clock::now();

  if (std::chrono::duration<double>(now - this->lastMsgSentTime).count() > 5.0)
  {
    // Here, we are assuming that the robot names are "X1" and "X2".
    if (this->name == "X1")
    {
      this->client->SendTo("Hello from " + this->name, "X2");
    }
    else
    {
      this->client->SendTo("Hello from " + this->name, "X1");
    }
    this->lastMsgSentTime = now;
  }

  // Check laser scan information
  Listener listener;
  ros::Subscriber sub = this->n.subscribe<sensor_msgs::LaserScan>(this->name + "/front_scan", 1000, &Listener::callback, &listener);

  auto laser_scan = listener.laser_scan;
  // Simple example for robot to go to entrance
  geometry_msgs::Twist msg;
  // Set direction for robot
  double linVel = 3.0;
  double angVel = 1.5;
  int direction = 0; // 0: forward, -1: left, 1: right;
  if (laser_scan.ranges[359] > laser_scan.ranges[119] && laser_scan.ranges[359] > laser_scan.ranges[599])
  {
    direction = 0;
  }
  else if (laser_scan.ranges[119] > laser_scan.ranges[359] && laser_scan.ranges[119] > laser_scan.ranges[599])
  {
    direction = 1;
  }
  else if (laser_scan.ranges[599] > laser_scan.ranges[359] && laser_scan.ranges[599] > laser_scan.ranges[119])
  {
    direction = -1;
  }
  else if (laser_scan.ranges[359] < laser_scan.ranges[119])
  {
    direction = (rand() > RAND_MAX/2) ? -1: 1;
  }
  else
  {
    direction = 0;
  }

  switch (direction)
  {
    case -1:
    {
      msg.angular.z = angVel;
      msg.linear.x = linVel;
    }
    case 1:
    {
      msg.angular.z = -angVel;
      msg.linear.x = linVel;
    }
    case 0:
    {
      msg.angular.x = linVel;
    }
  }
  this->velPub.publish(msg);
}

/////////////////////////////////////////////////
int main(int argc, char** argv)
{
  // Initialize ros
  ros::init(argc, argv, argv[1]);

  ROS_INFO("Starting seed competitor\n");
  std::string name;

  // Get the name of the robot based on the name of the "cmd_vel" topic if
  // the name was not passed in as an argument.
  if (argc < 2 || std::strlen(argv[1]) == 0)
  {
    ros::master::V_TopicInfo masterTopics;
    ros::master::getTopics(masterTopics);

    while (name.empty())
    {
      for (ros::master::V_TopicInfo::iterator it = masterTopics.begin();
          it != masterTopics.end(); ++it)
      {
        const ros::master::TopicInfo &info = *it;
        if (info.name.find("cmd_vel") != std::string::npos)
        {
          int rpos = info.name.rfind("/");
          name = info.name.substr(1, rpos - 1);
        }
      }
      if (name.empty())
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
  }
  // Otherwise use the name provided as an argument.
  else
  {
    name = argv[1];
  }

  // Create the controller
  Controller controller(name);

  // This sample code iteratively calls Controller::Update. This is just an
  // example. You can write your controller using alternative methods.
  // To get started with ROS visit: http://wiki.ros.org/ROS/Tutorials
  ros::Rate loop_rate(10);
  while (ros::ok())
  {
    controller.Update();
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
