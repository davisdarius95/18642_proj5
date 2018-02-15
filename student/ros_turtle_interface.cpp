#include <boost/bind.hpp>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int8.h>
#include <ece642rtle/RTIbump.h>
#include <ece642rtle/RTIatend.h>
#include <ece642rtle/Pose.h>
#include <QPointF>

#include "student.h"
QPointF TRI_pose_;
bool TRI_at_end_;

ros::Publisher TRI_solv_pub_;
ros::Publisher TRI_pose_pub_;
ros::Publisher TRI_ornt_pub_;
ros::Publisher TRI_vist_pub_;
ros::Subscriber TRI_pose_sub_;
ros::Subscriber TRI_solv_sub_;
ros::ServiceClient RTIbumpClient;
ros::ServiceClient RTIatendClient;

bool bumped(int x1, int y1, int x2, int y2) {
  ece642rtle::RTIbump bump_srv;
  bump_srv.request.x1 = x1;
  bump_srv.request.x2 = x2;
  bump_srv.request.y1 = y1;
  bump_srv.request.y2 = y2;
  if (RTIbumpClient.call(bump_srv)) {
      return bump_srv.response.bump;
  } else {
    ROS_ERROR("Failed to call service bump");
    return true;
  }
}

bool atend(int x, int y) {
  ece642rtle::RTIatend atend_srv;
  atend_srv.request.x = x;
  atend_srv.request.y = y;
  if (RTIatendClient.call(atend_srv)) {
    return atend_srv.response.atend;
  } else {
    ROS_ERROR("Failed to call service at end");
    return false;
  }
}

void displayTurtle(int nw_or) {
  displayTurtle(nw_or, -1);
}

void displayTurtle(int nw_or, int visits) {
  ece642rtle::Pose p;
  p.x = TRI_pose_.x();
  p.y = TRI_pose_.y();
  TRI_pose_pub_.publish(p);
  std_msgs::Int8 o;
  o.data = (int8_t) nw_or;
  TRI_ornt_pub_.publish(o);
  std_msgs::Int8 v;
  v.data = (int8_t) visits;
  TRI_vist_pub_.publish(v);
}


void TRIposeCallback(const ece642rtle::PoseConstPtr& pose)
{
  TRI_pose_.setX(pose->x);
  TRI_pose_.setY(pose->y);
}

void TRIsolvCallback(const std_msgs::Bool&) {
  TRI_at_end_ = moveTurtle(TRI_pose_);
  std_msgs::Bool b;
  b.data = TRI_at_end_;
  TRI_solv_pub_.publish(b);
}



int main(int argc, char** argv)
{
    ros::init(argc, argv, "ros_turtle_interface");
    ros::NodeHandle nh;
    TRI_solv_pub_ = nh.advertise<std_msgs::Bool>("turtle1/RTI_solv", 1);
    TRI_pose_pub_ = nh.advertise<ece642rtle::Pose>("turtle1/RTI_pose", 1);
    TRI_ornt_pub_ = nh.advertise<std_msgs::Int8>("turtle1/RTI_ornt", 1);
    TRI_vist_pub_ = nh.advertise<std_msgs::Int8>("turtle1/RTI_vist", 1);
    TRI_pose_sub_ = nh.subscribe("turtle1/pose", 1, TRIposeCallback);
    TRI_solv_sub_ = nh.subscribe("turtle1/solved", 1, TRIsolvCallback);
    RTIbumpClient = nh.serviceClient<ece642rtle::RTIbump>("turtle1/RTI_bump");
    RTIatendClient = nh.serviceClient<ece642rtle::RTIatend>("turtle1/RTI_at_end");
    TRI_at_end_ = false;

    ros::Rate loop_rate(10);
    while(TRI_solv_pub_.getNumSubscribers() < 1) {
      loop_rate.sleep();
    }
    
    std_msgs::Bool b;
    b.data = TRI_at_end_;
    TRI_solv_pub_.publish(b);
    ros::spinOnce();
    
    ros::spin();
}
