#include "ros/ros.h"
#include "std_srvs/Empty.h"

ros::ServiceClient clearcostmap_client;
ros::Timer clearcostmap_timer;

void ClearCostMapTimerCallBack(const ros::TimerEvent &event)
{
  std_srvs::Empty clear_costmap;
  if (clearcostmap_client.call(clear_costmap))
    ROS_INFO("clear costmap success");
  else
    ROS_ERROR("fail to clear costmap");
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "clear_map");
  ros::NodeHandle nh;
  
  clearcostmap_client = nh.serviceClient<std_srvs::Empty>("/move_base/clear_costmaps");
  clearcostmap_timer = nh.createTimer(ros::Duration(5.0), ClearCostMapTimerCallBack);
  
  ros::spin();
  
  return 0;
}
