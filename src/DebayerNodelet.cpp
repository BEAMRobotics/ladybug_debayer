/*-*-C++-*-*/
/**
   @file DebayerNodelet.cpp
   @author Steve Phillips
   @date November, 2018
   @brief ROS Nodelet for debayering images from the ladybug camera

   @attention Copyright (C) 2018
*/

#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>

#include <ladybug_debayer/LadybugDebayer.h>
namespace ladybug_debayer {
class DebayerNodelet : public nodelet::Nodelet {
 public:

  DebayerNodelet() = default;
  ~DebayerNodelet() override = default;

 private:

  virtual void onInit();
  boost::shared_ptr<LadybugDebayer> lb_debayer_;
};

/** @brief Nodelet initialization. */
void DebayerNodelet::onInit() {
  lb_debayer_.reset(new LadybugDebayer(getNodeHandle(), getPrivateNodeHandle()));
}
}

// Register this plugin with pluginlib.  Names must match nodelets.xml.
//
// parameters: class type, base class type
PLUGINLIB_EXPORT_CLASS(ladybug_debayer::DebayerNodelet, nodelet::Nodelet)