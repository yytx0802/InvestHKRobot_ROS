#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <sensor_msgs/Imu.h>



class SubscribeAndPublish
{
public:
  SubscribeAndPublish()
  {
    //Topic you want to publish
    pub_ = n_.advertise<sensor_msgs::Imu>("/imu", 10);

    //Accel publish slower
    sub1_ = n_.subscribe("/camera/accel/sample", 1, &SubscribeAndPublish::imucallback, this);
    sub2_ = n_.subscribe("/camera/gyro/sample", 1, &SubscribeAndPublish::gyrocallback, this);
    imu_buffer.header.stamp = ros::Time::now();
    imu_buffer.header.frame_id = "imu";
  
}

  void imucallback(const sensor_msgs::Imu::ConstPtr& msg)
  {
    sensor_msgs::Imu imu;
    imu = *msg;
    imu_buffer.header.stamp = imu.header.stamp;
    //leave imu_frame empty
    imu_buffer.linear_acceleration.x  = imu.linear_acceleration.x;
    imu_buffer.linear_acceleration.y  = imu.linear_acceleration.y;
    imu_buffer.linear_acceleration.z  = imu.linear_acceleration.z;
                                                                                                   
    //if(odom_pub)  odom_pub.publish(odom);
    pub_.publish(imu_buffer);

  }

  void gyrocallback(const sensor_msgs::Imu::ConstPtr& msg)
  {
    sensor_msgs::Imu imu;
    imu = *msg;
    imu_buffer.angular_velocity.x  = imu.angular_velocity.x;
    imu_buffer.angular_velocity.y  = imu.angular_velocity.y;
    imu_buffer.angular_velocity.z  = imu.angular_velocity.z;
  }

private:
  ros::NodeHandle n_; 
  ros::Publisher pub_;
  ros::Subscriber sub1_;
  ros::Subscriber sub2_;
  sensor_msgs::Imu imu_buffer;

};//End of class SubscribeAndPublish

int main(int argc, char **argv)
{
  //Initiate ROS
  ros::init(argc, argv, "fuse_imu");

  //Create an object of class SubscribeAndPublish that will take care of everything
  SubscribeAndPublish SAPObject;

  ros::spin();

  return 0;
}

