#include <seedhand_std/SeedHandInterface.hh>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "seedhand_interface_test");
  ros::NodeHandle nh;

  aero::interface::seedhandAeroMoveitInterface::Ptr robot(new aero::interface::seedhandAeroMoveitInterface(nh, true, false));
  // robot->sendThumb(aero::arm::rarm, 0.0, 1.0);
  // robot->sendThumb(aero::arm::rarm, -0.6, 1.0);
  // robot->sendThumb(aero::arm::rarm, 0.26, 1.0);
  // robot->sendIndex(aero::arm::rarm, 0.0, 1.0);
  // robot->sendIndex(aero::arm::rarm, 0.86, 1.0);
  robot->sendIndex(aero::arm::rarm, -1.134, 1.0);
  sleep(2);
  ros::shutdown();
  return 0;
}
