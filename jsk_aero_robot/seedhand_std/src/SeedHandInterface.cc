#include <seedhand_std/SeedHandInterface.hh>

aero::interface::seedhandAeroMoveitInterface::seedhandAeroMoveitInterface
(ros::NodeHandle _nh, const bool _use_rarm, const bool _use_larm) : aero::interface::AeroMoveitInterface(_nh) {
  if (_use_rarm) {
    rhand.reset(new robot_interface::TrajectoryClient
                (_nh, "rhand_controller/follow_joint_trajectory",
                 "rhand_controller/state", {"r_thumb_joint", "r_indexbase_joint"}));
    ri->add_controller("rhand", rhand);
  }

  if (_use_larm) {
    lhand.reset(new robot_interface::TrajectoryClient
                (_nh, "lhand_controller/follow_joint_trajectory",
                 "lhand_controller/state", {"l_thumb_joint", "l_indexbase_joint"}));
    ri->add_controller("lhand", lhand);
  }
};

void aero::interface::seedhandAeroMoveitInterface::cancelGrasp(const aero::arm _arm) {
  aero_startup::GraspControl srv;
  if (_arm == aero::arm::rarm) srv.request.position = POSITION_Right;
  else srv.request.position = POSITION_Left;
  srv.request.script = aero_startup::GraspControlRequest::SCRIPT_CANCEL;
  srv.request.power = (100 << 8) + 30;
  ros::service::call("/aero_ros_controller/grasp_control", srv.request, srv.response);
}

void aero::interface::seedhandAeroMoveitInterface::sendThumb
(const aero::arm _arm, const double _rad, const double _t) {
  cancelGrasp(_arm);
  if (_arm == aero::arm::rarm)
    ri->sendAngles({{"r_thumb_joint", _rad}}, _t, ros::Time::now());
}

void aero::interface::seedhandAeroMoveitInterface::sendIndex
(const aero::arm _arm, const double _rad, const double _t) {
  cancelGrasp(_arm);
  if (_arm == aero::arm::rarm)
    ri->sendAngles({{"r_indexbase_joint", _rad}}, _t, ros::Time::now());
}

void aero::interface::seedhandAeroMoveitInterface::sendSeedHand
(const aero::arm _arm, const double _thumb_rad, const double _index_rad, const double _t) {
  cancelGrasp(_arm);
  if (_arm == aero::arm::rarm) {
    ri->sendAngles({{"r_indexbase_joint", _index_rad}}, _t*0.5, ros::Time::now());
    ros::Duration(_t*0.5 + 1.0).sleep();
    ri->sendAngles({{"r_thumb_joint", _thumb_rad}}, _t*0.5, ros::Time::now());
  }
}

void aero::interface::seedhandAeroMoveitInterface::sendTrxHand
(const aero::arm _arm, const double _thumb_rad, const double _t) {
  cancelGrasp(_arm);
  if (_arm == aero::arm::larm) {
    ri->sendAngles({{"l_thumb_joint", _thumb_rad}}, _t*0.5, ros::Time::now());
  }
}
