#include <opencv/cv.hpp>
#include <boost/shared_ptr.hpp>

#include <ros/ros.h>
#include <ros/console.h>
#include <cv_bridge/cv_bridge.h>

#include <sensor_msgs/PointCloud2.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>
#include <jsk_recognition_msgs/BoundingBox.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>

class TrashbinOccupancyDetector{
private:
  double _trashbinL, _trashbinW, _trashbinH; // trashbin size param
  ros::Subscriber _trashbinHandleSub;
  ros::Publisher _virtualTrashbinContainerPub;
  geometry_msgs::Quaternion defaultOrientation;
  
public:
  // constructor
  TrashbinOccupancyDetector(ros::NodeHandle &_nh){
    _nh.getParam("trashbin_l", this->_trashbinL);
    _nh.getParam("trashbin_w", this->_trashbinW);
    _nh.getParam("trashbin_h", this->_trashbinH);
    this->_trashbinHandleSub = _nh.subscribe("trashbin_handle/boxes", 10, &TrashbinOccupancyDetector::CheckOccupancyCB, this);
    this->_virtualTrashbinContainerPub = _nh.advertise<jsk_recognition_msgs::BoundingBoxArray>("trashbin_handle/trashbin", 10);
    // init default orientation const
    this->defaultOrientation.w = 1.0;
    this->defaultOrientation.x = 0.0;
    this->defaultOrientation.y = 0.0;
    this->defaultOrientation.z = 0.0;
  };  
  // destructor
  ~TrashbinOccupancyDetector(){
  };

  // draw virtual trashbin container and publish for debugging
  void DrawVirtualTrashbinContainer(const jsk_recognition_msgs::BoundingBoxArray& handleCandidates){
    // Expected get non empty array
    boost::shared_ptr<jsk_recognition_msgs::BoundingBoxArray> handles, trashbins;
    boost::shared_ptr<jsk_recognition_msgs::BoundingBox> handle, trashbin;
    boost::shared_ptr<geometry_msgs::Point> handleCenterPosition, trashbinCenterPosition;

    *handles = handleCandidates; // TODO: Do not use all boundingboxes, use most likely trashbins

    for(int i=0; i<handles->boxes.size(); i++){
      *handle = handles->boxes.at(i);
      // decide trashbin center position
      *handleCenterPosition = handle->pose.position;
      trashbinCenterPosition->x = handleCenterPosition->x + this->_trashbinL/2.0;
      trashbinCenterPosition->y = handleCenterPosition->y;
      trashbinCenterPosition->z = handleCenterPosition->z - this->_trashbinH/2.0;
      // make BoundingBox trashbin
      trashbin->header = handle->header;
      trashbin->pose.position = *trashbinCenterPosition;
      trashbin->pose.orientation = this->defaultOrientation;
      trashbin->dimensions.x = this->_trashbinL;
      trashbin->dimensions.y = this->_trashbinW;
      trashbin->dimensions.z = this->_trashbinH;
      trashbin->label = i;
      // push to array
      trashbins->boxes.push_back(*trashbin);
    }
    trashbins->header = handles->header;
    this->_virtualTrashbinContainerPub.publish(*trashbins);
  };

  // main callback
  void CheckOccupancyCB(const jsk_recognition_msgs::BoundingBoxArray& msg){
    if (msg.boxes.empty()) {
      ROS_DEBUG("No handle candidates");
    } else {
      ROS_DEBUG_STREAM(msg.boxes.size() << " handle candidates" << std::endl);
      this->DrawVirtualTrashbinContainer(msg);
    };    
  };
};

int main(int argc, char **argv){
  ros::init(argc, argv, "trashbin_occupancy_detector");
  ros::NodeHandle pnh("~");
  TrashbinOccupancyDetector node = TrashbinOccupancyDetector(pnh);
  return 0;
}
