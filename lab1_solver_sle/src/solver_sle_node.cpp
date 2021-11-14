#include "ros/ros.h"
#include "lab1_solver_sle/solve_sle.h"
#include "math.h"
#include "std_msgs/Float32MultiArray.h"

ros::Publisher g_publisher;
ros:: Subscriber g_subscriber;
ros::ServiceServer g_service;

bool solve(lab1_solver_sle::solve_sle::Request  &req,
          lab1_solver_sle::solve_sle::Response &res)
{
  std_msgs::Float32MultiArray msg;

  float det = req.a * req.e - req.b * req.d;
  if (fabs(det) < 0.001)
  {
    g_publisher.publish(msg);
    return true;
  }
  float det_x = req.c * req.e - req.b * req.f;
  float det_y = req.a * req.f - req.c * req.d;
  res.roots.push_back(det_x / det);
  res.roots.push_back(det_y / det);

  msg.data.push_back(res.roots[0]);
  msg.data.push_back(res.roots[1]);
  g_publisher.publish(msg);

  return true;
}

void callback(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
  if (msg->data.empty())
  {
    ROS_INFO("No solutions");
  }
  else
  {
    ROS_INFO("x = %.2f, y = %.2f", msg->data[0], msg->data[1]);
  }
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "solve_sle_server");
  std::string topic_name;
  ros::NodeHandle n;
  n.getParam("lab1_solver_sle/topic_name", topic_name);
  g_publisher = n.advertise<std_msgs::Float32MultiArray>(topic_name, 1000);
  g_subscriber = n.subscribe(topic_name, 1000, callback);
  g_service = n.advertiseService("solve_sle", solve);
  ROS_INFO("Ready to solve sle.");
  ros::spin();

  return 0;
}