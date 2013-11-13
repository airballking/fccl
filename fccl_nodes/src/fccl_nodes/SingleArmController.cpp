#include <fccl_nodes/SingleArmController.h>

// The entry to our application...
int main(int argc, char **argv)
{
  ros::init(argc, argv, "single_arm_controller");
  ros::NodeHandle n("~");
  SingleArmController controller(n);
  ros::spin();
  controller.stop();
  return 0;
}
