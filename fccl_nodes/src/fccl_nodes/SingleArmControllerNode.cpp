#include <fccl_nodes/SingleArmController.h>

// The entry to our application...
int main(int argc, char **argv)
{
  ros::init(argc, argv, "single_arm_controller");
  ros::NodeHandle n("~");
  fccl::nodes::SingleArmController controller(n);
  controller.run();
  ros::spin();
  return 0;
}
