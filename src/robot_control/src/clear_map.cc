#include "ros/ros.h"
#include "std_srvs/Empty.h"

class clearCostMapObject {
 private:
  ros::NodeHandle _nh;
  ros::ServiceClient clearcostmap_client;
  ros::Timer clearcostmap_timer;

  void ClearCostMapTimerCallBack(const ros::TimerEvent &event) {
    std_srvs::Empty clear_costmap;
    if (clearcostmap_client.call(clear_costmap))
      ROS_INFO("clear costmap success");
    else
      ROS_ERROR("fail to clear costmap");
  }

 public:
  clearCostMapObject() {
    clearcostmap_client = _nh.serviceClient<std_srvs::Empty>("/move_base/clear_costmaps");
    clearcostmap_timer = _nh.createTimer(ros::Duration(5.0),
        &clearCostMapObject::ClearCostMapTimerCallBack, this);
  }
  ~clearCostMapObject() {
  }
};  // End of class clearCostMapObject

int main(int argc, char** argv) {
  ros::init(argc, argv, "clear_map");
  clearCostMapObject ccmo;
  ros::spin();
  return 0;
}
