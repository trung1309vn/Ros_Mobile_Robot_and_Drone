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
#include <map>

std::map<std::string, int> regions_ = {"right":0, 
                                       "fright":0,
                                       "front":0,
                                       "fleft":0,
                                       "left":0};

int state_ = 0;
std::map<int, std::string> state_dict_ = {0: "find the wall",
                                          1: "turn left",
                                          2: "follow the wall"};

class Listener_Laser
{
  public: 
    sensor_msgs::LaserScan laser;
    void callback(const sensor_msgs::LaserScan::ConstPtr& msg)
    {
      laser = *msg;
      ::regions_ = {
        "right":  min(laser.ranges[0:143]),
        "fright": min(laser.ranges[144:287]),
        "front":  min(laser.ranges[288:431]),
        "fleft":  min(laser.ranges[432:575]),
        "left":   min(laser.ranges[576:713]),
        }
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

  /// \brief A function to change robot state
  public: void ChangeState(int state);

  /// \brief A function to determine the action of robot
  public: void TakeAction();

  /// \brief ROS node handler.
  private: ros::NodeHandle n;

  /// \brief publisher to send cmd_vel
  private: ros::Publisher velPub;

  /// \brief subscriber to get laser_scan
  private: ros::Subscriber laserSub;

  /// \brief listener to get laser_scan
  private: Listener_Laser listener;

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
void Controller::ChangeState(int state)
{
  if (state != ::state_)
  {
    ROS_INFO("Wall follower - %s", ::state_dict_[state]);
    ::state_ = state;
  }
}

void Controller::TakeAction()
{
  std::string state_description = "";
  float d = 1.5;
  std::map<std::string, int> regions = ::regions_;

  if (region["front"] > d && region["fleft"] > d && region["fright"] > d)
  {
    state_description = "case 1 - nothing";
    this->change_state(0);
  }
  else if (region["front"] <= d && region["fleft"] > d && region["fright"] > d)
  {
    state_description = "case 2 - front"
    this->change_state(1);
  }
  else if (region["front"] > d && region["fleft"] > d && region["fright"] <= d)
  {
    state_description = "case 3 - fright";
    this->change_state(2);
  }
  else if (region["front"] > d && region["fleft"] <= d && region["fright"] > d)
  {
    state_description = "case 4 - fleft";
    this->change_state(0);
  }
  else if (region["front"] <= d && region["fleft"] > d && region["fright"] <= d)
  {
    state_description = "case 5 - front and fright";
    this->change_state(1);
  }
  else if (region["front"] <= d && region["fleft"] <= d && region["fright"] > d)
  {
    state_description = "case 6 - front and fleft";
    this->change_state(1);
  }
  else if (region["front"] <= d && region["fleft"] <= d && region["fright"] <= d)
  {
    state_description = "case 7 - front and fright and fleft";
    this->change_state(1);
  }
  else if (region["front"] > d && region["fleft"] <= d && region['fright'] <= d)
  {
    state_description = "case 8 - fright and fleft";
  }
  else
  {
    state_description = "Unknown case";
  }
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
    
  // Simple example for robot to go to entrance
  geometry_msgs::Twist msg;
  this->TakeAction();

  switch (::state_)
  {
    case 0: // find_wall
    {
      msg.angular.z = -0.3;
      msg.linear.x = 0.2;
      break;
    }
    case 1: // turn_left
    {
      msg.angular.z = 0.3;
      break;
    }
    case 2: // follow_the_wall
    {
      msg.linear.x = 0.5;
    }
  }
  ROS_INFO("Rotation: %f, Linear: %f", msg.angular.z, msg.linear.x);
  this->velPub.publish(msg);
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
  ros::Rate loop_rate(20);
  while (ros::ok())
  {
    controller.Update();
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
