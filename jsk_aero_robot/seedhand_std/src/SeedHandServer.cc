#include <ros/ros.h>
#include <seedhand_std/SeedHandInterface.hh>
#include <seedhand_std/SetHand.h>

aero::interface::seedhandAeroMoveitInterface::Ptr robot_;

bool callback(seedhand_std::SetHand::Request &req, seedhand_std::SetHand::Response &res){
  if(req.arm == "larm"){
    ROS_WARN("typeJSK aero has seedhand only rarm");
    res.status = false;
    return false;
  }

  res.status = true;
  if(req.finger == "thumb"){
    robot_->sendThumb(aero::arm::rarm, req.thumb_angle, req.time);
  } else if(req.finger == "index"){
    robot_->sendIndex(aero::arm::rarm, req.index_angle, req.time);
  } else if(req.finger == "both"){
    robot_->sendSeedHand(aero::arm::rarm, req.thumb_angle, req.index_angle, req.time);
  } else if(req.finger == "trx"){
    robot_->sendTrxHand(aero::arm::larm, req.thumb_angle, req.time);
  } else {
    ROS_WARN("set correct finger name: {thumb, index, both}");
    res.status = false;
  }
  return true;
}

int main(int argc, char **argv){
  ros::init(argc, argv, "seedhand_server");
  ros::NodeHandle nh;
  robot_.reset(new aero::interface::seedhandAeroMoveitInterface(nh));
  ros::ServiceServer seedhand_server =
    nh.advertiseService("/seedhand_server/set_hand", &callback);
  ros::spin();
  return 0;
}
