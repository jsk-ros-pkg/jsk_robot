#ifndef SEED_HAND_INTERFACE_
#define SEED_HAND_INTERFACE_

#include <aero_std/AeroMoveitInterface.hh>
#include <aero_startup/GraspControl.h>
#include <aero_extra_controllers/AeroHandController.hh>

namespace aero {
  namespace interface {
    class seedhandAeroMoveitInterface : public aero::interface::AeroMoveitInterface {
    public:
      typedef std::shared_ptr<seedhandAeroMoveitInterface> Ptr;
      seedhandAeroMoveitInterface(ros::NodeHandle _nh, const bool _use_rarm=true, const bool _use_larm=false);
      void cancelGrasp(const aero::arm _arm);
      void sendThumb(const aero::arm _arm, const double _rad, const double _t);
      void sendIndex(const aero::arm _arm, const double _rad, const double _t);
      void sendSeedHand(const aero::arm _arm, const double _thumb_rad, const double _index_rad, const double _t);
      void sendTrxHand(const aero::arm _arm, const double _thumb_rad, const double _t);
      robot_interface::TrajectoryClient::Ptr lhand;
      robot_interface::TrajectoryClient::Ptr rhand;
    };
  }
}

#endif
