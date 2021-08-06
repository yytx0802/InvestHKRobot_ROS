#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <clear_costmap_recovery/clear_costmap_recovery.h>
#include <rotate_recovery/rotate_recovery.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "rotate_drive");
    
  //ros::NodeHandle node;

  tf::TransformListener tf(ros::Duration(10));
  costmap_2d::Costmap2DROS global_costmap("global_costmap", tf);
  costmap_2d::Costmap2DROS local_costmap("local_costmap", tf);

  //clear_costmap_recovery::ClearCostmapRecovery ccr;
  //ccr.initialize("my_clear_costmap_recovery", &tf, &global_costmap, &local_costmap);

  //ccr.runBehavior();

  rotate_recovery::RotateRecovery rr;
  rr.initialize("my_rotate_recovery", &tf, &global_costmap, &local_costmap);

  rr.runBehavior();
  ros::spin();
  return 0;
};
