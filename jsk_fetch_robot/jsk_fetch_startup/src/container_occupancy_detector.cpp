#include <limits>

#include <boost/shared_ptr.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/common.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/passthrough.h>

#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/PointCloud2.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/transform_datatypes.h>
#include <tf2_eigen/tf2_eigen.h>

const float FLOAT_MIN = std::numeric_limits<float>::min();
const float FLOAT_MAX = std::numeric_limits<float>::max();

class ContainerOccupancyDetector{
private:
  std::string _inputBoxes, _inputCloud, _debugCloud, _outputBoxes;
  ros::Subscriber _containerBoxesSub, _pointCloudSub;
  ros::Publisher _debugPointPub, _occupancyPub;
  geometry_msgs::Quaternion defaultOrientation;
  jsk_recognition_msgs::BoundingBoxArray _containerBoxes;
  sensor_msgs::PointCloud2 _transformedCloudMsg;
  tf2_ros::Buffer _tfBuffer;
  tf2_ros::TransformListener* _tfListener;

public:
  // constructor
  ContainerOccupancyDetector(ros::NodeHandle &_nh, ros::NodeHandle &_pnh){
    this->_tfListener = new tf2_ros::TransformListener(_tfBuffer);
    this->_containerBoxesSub = _nh.subscribe("input_boxes", 10, &ContainerOccupancyDetector::CalculateOccupancyCB, this);
    this->_pointCloudSub = _nh.subscribe("input_cloud", 10, &ContainerOccupancyDetector::PointCloudTransformCB, this);
    this->_debugPointPub = _nh.advertise<sensor_msgs::PointCloud2>("debug_cloud", 10);
    this->_occupancyPub = _nh.advertise<jsk_recognition_msgs::BoundingBoxArray>("output_boxes", 10);
    // init default orientation const
    this->defaultOrientation.w = 1.0;
    this->defaultOrientation.x = 0.0;
    this->defaultOrientation.y = 0.0;
    this->defaultOrientation.z = 0.0;    
  };
  // destructor
  ~ContainerOccupancyDetector(){
    delete this->_tfListener;
    ROS_INFO("Killing node...");
  };

  void PointCloudTransformCB(const sensor_msgs::PointCloud2::ConstPtr& msg){
    geometry_msgs::TransformStamped transformStamped;
    try {
      transformStamped = _tfBuffer.lookupTransform("base_link", msg->header.frame_id, msg->header.stamp, ros::Duration(10.0));
      tf2::doTransform(*msg, this->_transformedCloudMsg, transformStamped);
      this->_debugPointPub.publish(this->_transformedCloudMsg);
    } catch(tf2::TransformException &ex) {
      ROS_WARN("Failed to transform tf");
      ROS_WARN("%s", ex.what());
    }
  };

  void CalculateOccupancyCB(const jsk_recognition_msgs::BoundingBoxArray& msg){
    const sensor_msgs::PointCloud2 inputCloud = _transformedCloudMsg;
    pcl::PCLPointCloud2 pclPC2;
    pcl::PointCloud<pcl::PointXYZ>::Ptr pclCloud(new pcl::PointCloud<pcl::PointXYZ>);
    jsk_recognition_msgs::BoundingBoxArray boxOccupancy = msg;
    if (!inputCloud.data.empty()){
      // convert to PCL cloud
      pcl_conversions::toPCL(inputCloud, pclPC2);
      pcl::fromPCLPointCloud2(pclPC2, *pclCloud);
      for (int i = 0; i < msg.boxes.size(); i++) {
        // calculate each bounding boxes
        // pcl::PassThrough<pcl::PointXYZ> filter;
        pcl::CropBox<pcl::PointXYZ> boxFilter;
        pcl::PointCloud<pcl::PointXYZ>::Ptr boxFilteredCloud(
            new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointXYZ minPt, maxPt;
        float occupancy;
        boxFilter.setMin(Eigen::Vector4f((msg.boxes.at(i).pose.position.x -
                                          msg.boxes.at(i).dimensions.x / 2.0),
                                         (msg.boxes.at(i).pose.position.y -
                                          msg.boxes.at(i).dimensions.y / 2.0),
                                         FLOAT_MIN, 1.0f));
        boxFilter.setMax(Eigen::Vector4f((msg.boxes.at(i).pose.position.x +
                                          msg.boxes.at(i).dimensions.x / 2.0),
                                         (msg.boxes.at(i).pose.position.y +
                                          msg.boxes.at(i).dimensions.y / 2.0),
                                         FLOAT_MAX, 1.0f));
        boxFilter.setInputCloud(pclCloud);
        boxFilter.filter(*boxFilteredCloud);
        pcl::getMinMax3D(*boxFilteredCloud, minPt, maxPt);
        ROS_DEBUG_STREAM(
            "box_id: " << i << " box_pos: " << msg.boxes.at(i).pose.position
                       << " box_height: "
                       << msg.boxes.at(i).pose.position.z +
                              msg.boxes.at(i).dimensions.z / 2.0
                       << " maxPt.z: " << maxPt.z);
        occupancy = maxPt.z / (msg.boxes.at(i).pose.position.z +
                               msg.boxes.at(i).dimensions.z / 2.0);
        // if some point clouds in boundingbox over bounding height
        // TODO publish occupancy instead
        boxOccupancy.boxes.at(i).value = occupancy;
      }
      this->_occupancyPub.publish(boxOccupancy);
    } else {
      ROS_WARN("No input point clouds.");
    }
    
  };
};

int main(int argc, char **argv){
  ros::init(argc, argv, "container_occupancy_detector");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  ContainerOccupancyDetector node(nh, pnh);
  ros::spin();
}
