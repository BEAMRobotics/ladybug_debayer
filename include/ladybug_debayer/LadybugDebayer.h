#include <sstream>
#include <chrono>

// ROS
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include "ladybug_msgs/LadybugTiles.h"

// OpenCV
#include <opencv2/core/core.hpp>

// DC1394
#include <dc1394/conversions.h>
#include <dc1394/dc1394.h>

namespace ladybug_debayer {
class LadybugDebayer {
 public:
  LadybugDebayer(ros::NodeHandle node_handle, ros::NodeHandle node_handle_private);
  ~LadybugDebayer() = default;

 private:
  bool ReadParameters();
  void DebayerTiles(const ladybug_msgs::LadybugTilesPtr &image_msg);

  ros::NodeHandle &node_handle_;
  ros::NodeHandle &node_handle_private_;

  //Topic names
  std::string image_tiles_topic_name_; // Topic name to subscribe to for ladybug_tile_subscriber_
  std::string debayered_topic_prefix_; // Prefix for ladybug_image_publisher_ topic names
  std::string debayering_method_name_; // For choosing debayering method
  std::string debayered_topic_suffix_; // Suffix for topic names

  ros::Subscriber ladybug_tile_subscriber_; //
  ros::Publisher ladybug_image_publisher_[6];

  sensor_msgs::Image colored_imgs_[6]; // Storage for new images
  unsigned int image_height_ = 2464;
  unsigned int image_width_ = 2048;

  unsigned int pub_image_height_ = 2464;
  unsigned int pub_image_width_ = 2048;
  sensor_msgs::Image image_in_, image_out_;
  cv::Size size_;
  size_t image_size_bytes_;

  dc1394error_t err_ = DC1394_SUCCESS;
  dc1394bayer_method_t debayering_method_;

  // ! Image transport members
  image_transport::ImageTransport image_transport_;
  image_transport::Subscriber it_subscriber_;
  image_transport::Publisher it_publisher_[6];
};
} //