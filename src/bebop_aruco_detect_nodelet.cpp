
// #include <rclcpp/rclcpp.hpp>

#include <ros/ros.h>

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>


#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Empty.h>

/**************************************************** OpenCv Libraries*/
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/features2d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/aruco.hpp>

/**************************************************** C++ Libraries*/
#include <iostream>
#include <string>

/*************************************************** Custom libraries */
#include "VisualControlToolbox/VisualControlToolbox.h"

class bebopVSNodelet : public nodelet::Nodelet
{
public:
  bebopVSNodelet();
  virtual ~bebopVSNodelet();

  virtual void onInit();
  void imageCallback( const sensor_msgs::ImageConstPtr &msg );

private:
  ros::NodeHandle m_nh;
  ros::Subscriber m_subImg;
  image_transport::Publisher m_pubCam;

  //  image message
  sensor_msgs::ImagePtr image_msg;
  // Image proessing parameters
  vct::parameters params;
  //  Desired configuration
  // vct::desired_configuration Desired_Configuration;
// Result of the matching operation

  //  configs
};

PLUGINLIB_EXPORT_CLASS( bebopVSNodelet, nodelet::Nodelet )
bebopVSNodelet::bebopVSNodelet(){}
bebopVSNodelet::~bebopVSNodelet() {  }


void bebopVSNodelet::onInit()
{

  m_nh = getNodeHandle();
  std::stringstream params_str;
  NODELET_INFO( "%s", params_str.str().c_str() );

  //  CONFIGS
  params.load(m_nh);
  params.initArucos(m_nh);

  m_subImg   = m_nh.subscribe( "image_raw", 100, &bebopVSNodelet::imageCallback, this );
  image_transport::ImageTransport it(m_nh);
  m_pubCam    = it.advertise( "matching", 10, this );
  NODELET_INFO( "Publisher and subscriber set up. Waiting for image callbacks..." );


}



void bebopVSNodelet::imageCallback( const sensor_msgs::ImageConstPtr &msg )
{

  NODELET_INFO("Callback");
  cv::Mat img;
  int success = 0;
  try{
      // img=cv_bridge::toCvShare(msg,"bgr8")->image;
      img=cv_bridge::toCvCopy(msg,"bgr8")->image;
      success = 1;
  }catch (cv_bridge::Exception& e){
      NODELET_ERROR("Could not convert from '%s' to 'bgr8'.",
                msg->encoding.c_str());
  }

  if (success )
  {
    std::vector<std::vector<cv::Point2f>> corners;
    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f> > _rejected;
    cv::aruco::detectMarkers(img,
                             params.dictionary,
                             corners,
                             ids,
                             params.parameters,
                             _rejected);
    cv::aruco::drawDetectedMarkers(img, corners, ids, cv::Scalar(0,255,0));

    cv::Mat p;

    for (int i = 0; i< corners.size(); i++)
      for (int j = 0; j< corners[i].size(); j++)
        p.push_back(corners[i][j]);
    if (! p.empty())
    {
      cv::Mat _p;
      _p = p.reshape(1);
      std::cout << "---" << std::endl;
      std::cout << p << std::endl;
      // p.convertTo(_p,CV_64F);
      std::cout << _p << std::endl;
      camera_norm(params, _p);
      std::cout << _p << std::endl;
    }

    // for (int i = 0; i< corners.size(); i++)
    //   std::cout << corners[i] << std::endl ;

    /***************************** Prepare message */

    image_msg = cv_bridge::CvImage(std_msgs::Header(),
                                  sensor_msgs::image_encodings::BGR8,
                                  img).toImageMsg();
    image_msg->header.frame_id = "matching_image";
    image_msg->width = img.cols;
    image_msg->height = img.rows;
    image_msg->is_bigendian = false;
    image_msg->step = sizeof(unsigned char) * img.cols*3;
    image_msg->header.stamp = ros::Time::now();
    // Publish image of the matching
    m_pubCam.publish(image_msg);
  }
}

