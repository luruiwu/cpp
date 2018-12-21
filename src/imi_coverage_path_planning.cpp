#include<imi_nav_v2/imi_coverage_path_planning.h>

//imi_coverage_path_planning.h
int main(int argc, char** argv)
{
  ros::init(argc, argv, "imi_nav_node_v2");
  ros::NodeHandle nh("~");

  ImiCoveragePathPlanning cpp(nh);
  cpp.start();
  ros::spin();
  return 0;
}
