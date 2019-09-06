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
#include <unistd.h>
#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <std_srvs/SetBool.h>
#include <rosgraph_msgs/Clock.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/FluidPressure.h>
#include <typeinfo>
#include <iostream>
#include <string>
#include <vector>

/// \brief. Class for getting information from subscriber
// class Listener
// {
//   public: 
//     sensor_msgs::LaserScan laser;
//     void callback(const sensor_msgs::LaserScan::ConstPtr& msg)
//     {
//       laser = *msg;
//     }
// };

/// \brief. Class for getting information from subscriber
class Listener_Pressure
{
  public: 
    sensor_msgs::FluidPressure air;
    void callback(const sensor_msgs::FluidPressure::ConstPtr& msg)
    {
      air = *msg;
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
  public: ros::Subscriber airSub;

  /// \brief listener to get laser_scan
  public: Listener_Pressure listener;

  /// \brief Get drone height using fluid pressure sensor
  public: float getHeight();

  /// \brief Take off function
  public: void takeOff();

  /// \brief Landing function
  public: void landing();

  /// \brief Move up and down function
  public: void thrustMove(float dest);

  /// \brief Move forward and backward
  public: float pitchMove(bool isForward, bool isBrake, float mass);

  /// \brief Move to the left and right
  public: float rollMove(bool isLeft, bool isBrake, float mass);

  /// \brief Drone yaw
  public: float yawMove(bool isClockWise, bool isBrake, float mass);
  
  /// \brief Client to request pose from origin.
  private: ros::ServiceClient originClient;

  /// \brief True if started.
  private: bool started{false};

  /// \brief Last time a comms message to another robot was sent.
  private: std::chrono::time_point<std::chrono::system_clock> lastMsgSentTime;

  /// \brief Name of this robot.
  private: std::string name;

  /// \brief Tick variable for calculating time
  private: int tick{0};

  /// \brief Last Height
  private: float lastHeight{0.0};

  /// \brief Flight path
  private: std::vector<std::string> path;

  /// \brief Current action
  private: int currentAction{0};
};

/////////////////////////////////////////////////
Controller::Controller()
{
  ROS_INFO("Waiting for /clock, /subt/start");

  ros::topic::waitForMessage<rosgraph_msgs::Clock>("/clock", this->n);

  // Wait for the start service to be ready.
  ros::service::waitForService("/subt/start", -1);

  // Get Param
  std::string _name = "X3";
  this->name = _name;
  ROS_INFO("Using robot name[%s]\n", this->name.c_str());

  // Init path (Demo path: square shape route)
  this->path.push_back("Move_Forward");
  this->path.push_back("Turn_Right");
  this->path.push_back("Move_Forward");
  this->path.push_back("Turn_Right");
  this->path.push_back("Move_Forward");
  this->path.push_back("Turn_Right");
  this->path.push_back("Move_Forward");
  this->path.push_back("Turn_Right");
}

/////////////////////////////////////////////////
float Controller::getHeight()
{
  // Standard air pressure = 101325 Pascal
  // Standard density of air = 1.225 kg / m3
  // Standard gravitational acceleration = 9.8  m / s2
  return abs(this->listener.air.fluid_pressure - 101325) / (1.225 * 9.8);
}

/////////////////////////////////////////////////
float Controller::pitchMove(bool isForward, bool isBrake, float mass)
{
  if (isForward)
  {
    if (isBrake)
      return -0.1;
    else
      return 0.05;
  }
  else
  {
    if (isBrake)
      return 0.1;
    else
      return -0.05;
  }
}

/////////////////////////////////////////////////
float Controller::rollMove(bool isLeft, bool isBrake, float mass)
{
  if (isLeft)
  {
    if (isBrake)
      return -0.1;
    else
      return 0.05;
  }
  else
  {
    if (isBrake)
      return 0.1;
    else
      return -0.05;
  }
}

/////////////////////////////////////////////////
float Controller::yawMove(bool isClockWise, bool isBrake, float mass)
{
  float Pi = 3.14159265;
  if (isClockWise)
  {
    if (isBrake)
      return Pi / 2;
    else
      return -Pi / 2;
  }
  else
  {
    if (isBrake)
      return -Pi / 2;
    else
      return Pi / 2;
  }  
}

/////////////////////////////////////////////////
void Controller::takeOff()
{
  float height = this->getHeight();
  if (height > 100.0)
    return;
  height -= 0.053036;
  float dest = 1.0;

  float gravity = 9.79967;
  float mass = 1.52;

  // PID gain
  float weight = gravity * mass;
  float offset = 0.1; // Adding variable to thrust
  // Error
  float error = dest - height;
  float thrust;
  float pitch = -0.04975;
  float yaw = 0.0;
  float roll = 0.0;
  float Pout = 0;
  geometry_msgs::Twist msg;
 
  // Take off phase 
  if (error >= 0.05)
  {
    Pout = offset * error / (dest - 0.05);
    thrust = weight + Pout; 
  }
  // Brake phase
  else
  {
    // Braking state 
    // Note: Adjust the threshold to make the brake more sensitive
    if (abs(height - this->lastHeight) > 0.004)
      thrust = weight + 0.05 - 0.5 * dest;
    // Stabilizing state - making drone hover
    else
    {
      thrust = weight;
      if (error <= -0.05)
        thrust = weight - 0.025;
      
      // Action state
      if (this->tick < 40)
      {
        pitch += this->path[this->currentAction] == "Move_Forward" ? this->pitchMove(true, false, mass) : 0.0;
        yaw += this->path[this->currentAction] == "Turn_Right" ? this->yawMove(true, false, mass) : 0.0;
        //roll += this->path[this->currentAction] == "Roll" ? this->rollMove(isForward, false, mass) : 0.0;
        thrust -= 0.01;
      }
      // Breaking action state
      else if (this->tick <= 50)
      {
        pitch += this->path[this->currentAction] == "Move_Forward" ? this->pitchMove(true, true, mass) : 0.0;
        yaw += this->path[this->currentAction] == "Turn_Right" ? this->yawMove(true, true, mass) : 0.0;
        //roll += this->rollMove(isForward, true, mass) : 0.0;
        thrust -= 0.01;
      }
      // Stabilizing state
      else if (this->tick <= 60)
      {
        if (this->path[this->currentAction] == "Turn_Right")
          pitch += 0.05;
        thrust -= 0.01;
      }
      // Change action state
      else if (this->tick == 80)
      {
        this->tick = 0;
        this->currentAction += 1;
        if (this->currentAction > 7)
          this->currentAction = 0;
      }
      this->tick += 1;
    } 
  }

  ROS_INFO("Delta height: %f", abs(height - this->lastHeight));
  ROS_INFO("Tick: %d", this->tick);
  this->lastHeight = height;
  
  ROS_INFO("Drone's height: %f", height);
  ROS_INFO("Thrust: %f", thrust);
  ROS_INFO("Current action: %d", this->currentAction);
  //ROS_INFO("Pitch: %f\n", pitch);
  //ROS_INFO("Yaw: %f\n", yaw);
  ROS_INFO("Roll: %f\n", roll);

  msg.linear.z = thrust;
  msg.linear.x = pitch;
  msg.linear.y = roll;
  msg.angular.z = yaw;
  this->velPub.publish(msg);
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
      this->airSub = this->n.subscribe<sensor_msgs::FluidPressure>(
          this->name + "/air_pressure", 1000, &Listener_Pressure::callback, &this->listener);    
    }
    else
      return;
  }

  // Add code that should be processed every iteration.

  std::chrono::time_point<std::chrono::system_clock> now =
    std::chrono::system_clock::now();
  

  this->takeOff();
}

/////////////////////////////////////////////////
int main(int argc, char** argv)
{
  // Initialize ros
  ros::init(argc, argv, "drone_free");

  ROS_INFO("Starting seed competitor\n");
  std::string name;

  // Create the controller
  Controller controller;

  // This sample code iteratively calls Controller::Update. This is just an
  // example. You can write your controller using alternative methods.
  // To get started with ROS visit: http://wiki.ros.org/ROS/Tutorials
  ros::Rate loop_rate(20);
  while (ros::ok())
  {
    controller.Update();
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
