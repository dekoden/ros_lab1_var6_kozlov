#include "ros/ros.h"
#include "lab1_solver_sle/solve_sle.h"
#include "math.h"
#include "std_msgs/Float32MultiArray.h"

ros::Publisher g_publisher;
ros::Subscriber g_subscriber;
ros::ServiceServer g_service;

bool solve(lab1_solver_sle::solve_sle::Request  &req,
          lab1_solver_sle::solve_sle::Response &res)
{
  std_msgs::Float32MultiArray msg;

  float eps = 1e-5;

  float ae = req.a * req.e;
  float bd = req.b * req.d;
  float af = req.a * req.f;
  float cd = req.c * req.d;
  float bf = req.b * req.f;
  float ce = req.c * req.e;

  // One solution
  if (fabs(ae - bd) > eps) // ae != bd
  {
    res.roots.push_back((ce - bf) / (ae - bd));
    res.roots.push_back((af - cd) / (ae - bd));
    msg.data.push_back(res.roots[0]);
    msg.data.push_back(res.roots[1]);
    g_publisher.publish(msg);
    return true;
  }
  // No solutions
  if (fabs(ae - bd) < eps && fabs(af - cd) > eps) // ae == bd && af != cd
  {
    g_publisher.publish(msg);
    return true;
  }
  // Infinite number of solutions
  res.roots.push_back(INFINITY);
  msg.data.push_back(res.roots[0]);
  g_publisher.publish(msg);
  return true;
}

void callback(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
   // One solution
  if (msg->data.size() == 2)
  {
    ROS_INFO("x = %.2f, y = %.2f", msg->data[0], msg->data[1]);
  }
  // Infinite number of solutions
  if (msg->data.size() == 1 && msg->data[0] == INFINITY)
  {
    ROS_INFO("Infinite number of solutions");
  }
  // No solutions
  if (msg->data.empty())
  {
    ROS_INFO("No solutions");
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
