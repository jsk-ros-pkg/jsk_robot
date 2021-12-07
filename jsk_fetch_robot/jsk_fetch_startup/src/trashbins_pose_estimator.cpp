#include <boost/shared_ptr.hpp>

#include <ros/ros.h>
#include <ros/console.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>
#include <jsk_recognition_msgs/BoundingBox.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>

class TrashbinsPoseEstimator{
private:
  double _trashbinL, _trashbinW, _trashbinH; // trashbin size param
  ros::Subscriber _trashbinHandleSub;
  ros::Subscriber _pointCloud;
  ros::Publisher _virtualTrashbinContainerPub;
  jsk_recognition_msgs::BoundingBoxArray _trashbins; // trashbin object
  geometry_msgs::Quaternion defaultOrientation;

public:
  // constructor
  TrashbinsPoseEstimator(ros::NodeHandle &_nh, ros::NodeHandle &_pnh){
    _pnh.getParam("trashbin_l", this->_trashbinL);
    _pnh.getParam("trashbin_w", this->_trashbinW);
    _pnh.getParam("trashbin_h", this->_trashbinH);
    this->_trashbinHandleSub = _nh.subscribe("trashbin_handle/boxes", 10, &TrashbinsPoseEstimator::DrawVirtualTrashbinContainerCB, this);
    // this->_pointCloud = _nh.subscribe("points", 10, &TrashbinOccupancyDetector::CalculateOccupancyCB, this);
    this->_virtualTrashbinContainerPub = _nh.advertise<jsk_recognition_msgs::BoundingBoxArray>("trashbin/boxes", 10);
    // init default orientation const
    this->defaultOrientation.w = 1.0;
    this->defaultOrientation.x = 0.0;
    this->defaultOrientation.y = 0.0;
    this->defaultOrientation.z = 0.0;
  };  
  // destructor
  ~TrashbinsPoseEstimator(){
    ROS_INFO("Killing node...");
  };

  // draw virtual trashbin container and publish for debugging
  void DrawVirtualTrashbinContainerCB(const jsk_recognition_msgs::BoundingBoxArray& msg){
  boost::shared_ptr<jsk_recognition_msgs::BoundingBoxArray> handles(new jsk_recognition_msgs::BoundingBoxArray), trashbins(new jsk_recognition_msgs::BoundingBoxArray);
  boost::shared_ptr<jsk_recognition_msgs::BoundingBox> handle(new jsk_recognition_msgs::BoundingBox), trashbin(new jsk_recognition_msgs::BoundingBox);
  boost::shared_ptr<geometry_msgs::Point> handleCenterPosition(new geometry_msgs::Point), trashbinCenterPosition(new geometry_msgs::Point);

  if (msg.boxes.empty()) {
    ROS_DEBUG("No handle candidates");
  } else {
    ROS_DEBUG_STREAM(msg.boxes.size() << " handle candidates" << std::endl);
    *handles = msg;
    for (int i = 0; i < handles->boxes.size(); i++) {
      *handle = handles->boxes.at(i);
      // decide trashbin center position
      *handleCenterPosition = handle->pose.position;
      trashbinCenterPosition->x =
          handleCenterPosition->x + this->_trashbinL / 2.0;
      trashbinCenterPosition->y = handleCenterPosition->y;
      trashbinCenterPosition->z =
          handleCenterPosition->z - this->_trashbinH / 2.0;
      // make BoundingBox trashbin
      trashbin->header = handle->header;
      trashbin->pose.position = *trashbinCenterPosition;
      trashbin->pose.orientation = this->defaultOrientation;
      trashbin->dimensions.x = this->_trashbinL;
      trashbin->dimensions.y = this->_trashbinW;
      trashbin->dimensions.z = this->_trashbinH;
      trashbin->label = i;
      trashbins->boxes.push_back(*trashbin);
    }
    trashbins->header = handles->header;
    this->_virtualTrashbinContainerPub.publish(*trashbins);
    this->_trashbins = *trashbins;
  }
  }
};

int main(int argc, char **argv){
  ros::init(argc, argv, "trashbin_pose_estimator");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  TrashbinsPoseEstimator node = TrashbinsPoseEstimator(nh, pnh);
  ros::spin();
}
