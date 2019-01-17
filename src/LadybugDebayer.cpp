#include "ladybug_debayer/LadybugDebayer.h"

namespace ladybug_debayer {
LadybugDebayer::LadybugDebayer(ros::NodeHandle node_handle, ros::NodeHandle node_handle_private)
    : node_handle_(node_handle),
      node_handle_private_(node_handle_private),
      image_transport_(node_handle) {
  if (!ReadParameters()) {
    ROS_ERROR("Could not read parameters.");
    ros::requestShutdown();
  }

  for (unsigned int cam = 0; cam < 6; cam++) {
    std::ostringstream s;
    s << debayered_topic_prefix_ << cam << debayered_topic_suffix_;
    //ladybug_image_publisher_[cam] = node_handle_.advertise<sensor_msgs::Image>(s.str(), 1);
    it_publisher_[cam] = image_transport_.advertise(s.str(), 1);
  }


  // subscribe to Image Tiles
  ladybug_tile_subscriber_ =
      node_handle_.subscribe(image_tiles_topic_name_, 1,
                             &LadybugDebayer::DebayerTiles, this,
                             ros::TransportHints().tcpNoDelay(true));

  // Setup image containers
  if (debayering_method_name_ == "DC1394_BAYER_METHOD_DOWNSAMPLE") {
    debayering_method_ = DC1394_BAYER_METHOD_DOWNSAMPLE;
    pub_image_height_ = 2464 / 2;
    pub_image_width_ = 2048 / 2;
  } else {
    debayering_method_ = DC1394_BAYER_METHOD_MIN;
  }

  image_out_.height = pub_image_height_;
  image_out_.width = pub_image_width_;
  image_out_.is_bigendian = false;
  image_out_.encoding = sensor_msgs::image_encodings::RGB8;
  image_out_.step = image_out_.width * 3;
  image_out_.data.resize(image_out_.height * image_out_.step);

  image_in_.height = pub_image_width_;
  image_in_.width = pub_image_height_;
  image_in_.is_bigendian = false;
  image_in_.encoding = sensor_msgs::image_encodings::RGB8;
  image_in_.step = image_in_.width * 3;
  image_in_.data.resize(image_in_.height * image_in_.step);

  size_.width = pub_image_height_;
  size_.height = pub_image_width_;
  image_size_bytes_ = pub_image_height_ * pub_image_width_ * 3;

}

bool LadybugDebayer::ReadParameters() {
  node_handle_private_.param<std::string>("image_tiles_topic_name", image_tiles_topic_name_, "image_tiles");
  node_handle_private_.param<std::string>("debayered_topic_prefix", debayered_topic_prefix_, "/ladybug/camera_");
  node_handle_private_.param<std::string>("debayered_topic_suffix", debayered_topic_suffix_, "/image_colored");
  node_handle_private_
      .param<std::string>("debayering_method_name", debayering_method_name_, "DC1394_BAYER_METHOD_DOWNSAMPLE");

  ROS_INFO("[Param] image_topic_name: '%s'", image_tiles_topic_name_.c_str());
  ROS_INFO("[Param] debayered_topic_prefix: '%s'", debayered_topic_prefix_.c_str());
  ROS_INFO("[Param] debayering_method: '%s'", debayering_method_name_.c_str());

  return true;
}
/** @brief Callback for raw scan messages. */
void LadybugDebayer::DebayerTiles(const ladybug_msgs::LadybugTilesPtr &image_tiles) {
  auto start = std::chrono::high_resolution_clock::now();
  auto end = std::chrono::high_resolution_clock::now();
  std::chrono::duration<float> duration = end - start;
  //std::cout << "Time for transposing / flipping image = " << duration.count() << std::endl;

  for (int i = 0; i < 6; ++i) {
    // Debayer using libdc1394
    err_ = dc1394_bayer_decoding_8bit(&(image_tiles->images[i].data[0]), &(image_in_.data[0]),
                                      image_height_, image_width_,
                                      DC1394_COLOR_FILTER_RGGB, //BGGR,
                                      debayering_method_);

    // Rotate image in OpenCV
    cv::Mat image(size_, CV_8UC3, &(image_in_.data[0])); //Create image container
    cv::transpose(image, image); //Transpose image
    cv::flip(image, image, 1); //Flip image

    // Copy rotated image into sensor_msgs::image container
    memcpy(&(image_out_.data[0]), image.data, image_size_bytes_);

    // Publish rotated sensor_msgs::image
    image_out_.header.stamp = image_tiles->images[i].header.stamp;
    std::ostringstream s;
    s << "camera_" << i;
    image_out_.header.frame_id = s.str();
    //ladybug_image_publisher_[i].publish(image_out_);
    it_publisher_[i].publish(image_out_);
    /*
      DC1394_BAYER_METHOD_NEAREST=0,
      DC1394_BAYER_METHOD_SIMPLE,
      DC1394_BAYER_METHOD_BILINEAR,
      DC1394_BAYER_METHOD_HQLINEAR,
      DC1394_BAYER_METHOD_DOWNSAMPLE,
      DC1394_BAYER_METHOD_EDGESENSE,
      DC1394_BAYER_METHOD_VNG,
      DC1394_BAYER_METHOD_AHD
    */
  }
}
}

//sensor_msgs::Image colored_imgs_[6]; // Storage for new images

/*  // convert to OpenCV Mat
  //receive Bayer Image, convert to Color 3 channels
  cv::Size size(2464, 2048);
  cv::Mat full_size;
  for(unsigned int i = 0; i < 6; i++)
  {
    cv::Mat rawImage(size, CV_8UC1, &(image_tiles->images[i].data[0])); // Raw image in
    cv::Mat image(size, CV_8UC3); //Create image container
    cv::cvtColor(rawImage, image, cv::COLOR_BayerBG2RGB);//cv::COLOR_BayerBG2RGB); //RGGB Debayer BGGR
    cv::transpose(image, image); //Transpose image
    cv::flip(image, image, 1); //Flip image

    size_t image_size = 2464 * 2048 * image.elemSize();
    std::cout << "Image size = " << image_size << std::endl;
    colored_imgs_[i].data.resize(image_size);
    memcpy(&(colored_imgs_[i].data[0]), image.data, image_size);

    colored_imgs_[i].header.seq = image_tiles->header.seq;
    colored_imgs_[i].header.frame_id = "camera";
    colored_imgs_[i].header.stamp = image_tiles->header.stamp;
    colored_imgs_[i].height = image.size().height;
    colored_imgs_[i].width = image.size().width;
    colored_imgs_[i].step = image.cols * image.elemSize();
    colored_imgs_[i].encoding = "rgb8";
    //std::cout << "New image " << i << " size = " << colored_imgs_[i].data.size() << std::endl;
  }


    //std::ostringstream s;
    //s << "/home/steve/test" << i << ".jpg";

    //std::string path = "/home/steve/test" + std::to_string(i)+ std::string(".jpg");
    //std::cout << s.str() << std::endl;
    //cv::imwrite(s.str(), image);
    //ladybug_image_publisher_[i].publish(colored_imgs_[i]);



  }

  cv::Size size(2464, 2048);
  cv::Mat full_size;
  for(unsigned int i = 0; i < 6; i++)
  {
    cv::Mat rawImage(size, CV_8UC1, &(image_tiles->images[i].data[0])); // Raw image in
    cv::Mat image(size, CV_8UC3); //Create image container
    cv::cvtColor(rawImage, image, cv::COLOR_BayerBG2RGB);//cv::COLOR_BayerBG2RGB); //RGGB Debayer BGGR
    cv::transpose(image, image); //Transpose image
    cv::flip(image, image, 1); //Flip image

    size_t image_size = 2464 * 2048 * image.elemSize();
    std::cout << "Image size = " << image_size << std::endl;
    colored_imgs_[i].data.resize(image_size);
    memcpy(&(colored_imgs_[i].data[0]), image.data, image_size);
  }
*/