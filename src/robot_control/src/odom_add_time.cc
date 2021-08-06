#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>




class SubscribeAndPublish
{
public:
  SubscribeAndPublish()
  {
    //Topic you want to publish
    pub_ = n_.advertise<nav_msgs::Odometry>("odom", 10);

    //Topic you want to subscribe
    sub_ = n_.subscribe("/stm32_odom", 1, &SubscribeAndPublish::posecallback, this);
  }

  void posecallback(const nav_msgs::Odometry::ConstPtr& msg)
  {
    nav_msgs::Odometry odom;
    odom = *msg;
    odom.header.stamp = ros::Time::now();
    odom.header.frame_id = "odom";
    //odom.child_frame_id = "base_link";
    //odom.pose = msg->pose;
    //odom.twist = msg->twist;
    odom.pose.covariance[0] = 0.001;
    odom.pose.covariance[7] = 0.001;
    odom.pose.covariance[14] = 1e6;
    odom.pose.covariance[21] = 1e6;
    odom.pose.covariance[28] = 1e6;
    odom.pose.covariance[35] = 1e3;
    odom.twist.covariance = odom.pose.covariance;

    //if(odom_pub)  odom_pub.publish(odom);
    pub_.publish(odom);
  }

  ~SubscribeAndPublish(){}
private:
  ros::NodeHandle n_; 
  ros::Publisher pub_;
  ros::Subscriber sub_;
  
};//End of class SubscribeAndPublish

int main(int argc, char **argv)
{
  //Initiate ROS
  ros::init(argc, argv, "odom_add_time");

  //Create an object of class SubscribeAndPublish that will take care of everything
  SubscribeAndPublish SAPObject;

  ros::spin();

  return 0;
}
