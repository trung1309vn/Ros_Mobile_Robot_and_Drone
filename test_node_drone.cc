#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <string>
#include <subt_msgs/PoseFromArtifact.h>
using namespace std;

#include <subt_ign/CommonTypes.hh>
#include <subt_ign/protobuf/artifact.pb.h>

class Controller
{
    // _name: name of robot. i.e: "x1"
    public: Controller(const string &_name);

    // Called for every loop
    public: void Update();

    // ROS node handler
    private: ros::NodeHandle n;

    // Send cmd_vel
    private: ros::Publisher velPub;

    // Client to request pose from origin
    private: ros::ServiceClient originClient;

    // Service to request pose from origin
    private: subt_msgs::PoseFromArtifact originSrv;

    private: double linVel{0.0};

    private: double current_height{0.0};

    private: int count{0};

    private: bool arrived{false};
};

Controller::Controller(const string &_name)
{
    // Create cmd_vel publisher
    this->velPub = this->n.advertise<geometry_msgs::Twist>(_name + "cmd_vel", 1);

    this->originClient = this->n.serviceClient<subt_msgs::PoseFromArtifact(
        "/subt/pose_from_artifact_origin");
    this->originSrv.request.robot_name.data = _name;

    // Send start signal
    std_srvs::SetBool::Request req;
    std_srvs::SetBool::Response res;
    req.data = true;
    if (!ros::service::call("/subt/start", req, res))
    {
        ROS_ERROR("Unable to send start signal.");
    }

    if (!res.success)
    {
        ROS_ERROR("Failed to send start signal [%s]", res.message.c_str());
    }
    else
    {
        ROS_INFO("Send start signal.");
    }
}

void Controller::Update()
{
    if (this->arrived)
        return;

    // Query current robot position w.r.t entrance
    if (!this->originClient.call(this->originSrv) ||
        !this->originSrv.response.success)
    {
        ROS_ERROR_ONCE("Failed to call pose_from_artifact_origin service, \
robot may not exist, or be outside staging area.");

        // Stop robot
        geometry_msgs::Twist msg;
        this->velPub.publish(msg);

        // If out of range once, we consider it arrived
        this->arrived = true;

        return;
    }

    auto pose = this->originSrv.response.pose.pose;

    geometry_msgs::Twist msg;

    // Height
    double height = pose.position.z;
    double linVel_cur = 14.0;

    if (abs(height - this->current_height) <= 0.1 && this->current_height >= 5.0)
    {
        this->count += 1;
        if (this->count >= 10)
        {
            msg.linear.x = 0;
            msg.angular.z = 0;
            this->arrived = true;
            ROS_INFO("Arrived at entrance!");
        }
    }
    else
    {
        this->current_height = height;
        msg.linear.z = linVel_cur + this->linVel;
        if (height < 5.0)
            this->linVel += 0.5;
        else
            this->linVel -= 0.5;
    }
    this->velPub.publish(msg);
}

/////////////////////////////////////////////////
int main(int argc, char** argv)
{
  if (argc < 2)
  {
    ROS_ERROR_STREAM("Needs an argument for the competitor's name.");
    return -1;
  }

  // Initialize ros
  ros::init(argc, argv, argv[1]);

  ROS_INFO("Starting seed competitor\n");

  // Create the controller
  Controller controller(argv[1]);

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