
// #include <rclcpp/rclcpp.hpp>

#include <ros/ros.h>

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

// #include <geometry_msgs/msg/pose.hpp>
// #include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h> // nav_msgs/Odometry

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
  vct::desired_configuration Desired_Configuration;
// Result of the matching operation

  //  configs
  bool verbose = false;
};

PLUGINLIB_EXPORT_CLASS( bebopVSNodelet, nodelet::Nodelet )
bebopVSNodelet::bebopVSNodelet(){}
  // : bebop_has_been_setup( false )
  // , m_cameraTilt( 0.0 )
  // , m_cameraPan( 0.0 )
  // , m_vec_ip_has_been_sorted( false )

// {
// }

bebopVSNodelet::~bebopVSNodelet() {  }
// bebopVSNodelet::~bebopVSNodelet() { m_task.kill(); }


void bebopVSNodelet::onInit()
{

  m_nh = getNodeHandle();
  std::stringstream params_str;
  NODELET_INFO( "%s", params_str.str().c_str() );

  //  CONFIGS
  verbose = m_nh.param(std::string("verbose"), false);
  // params.initArucos(m_nh, cv::aruco::DICT_6X6_250);
  // params.initArucos(m_nh, cv::aruco::DICT_4X4_250);
  // params.initArucos(m_nh, cv::aruco::DICT_7X7_250);
  params.initArucos(m_nh, cv::aruco::DICT_APRILTAG_36h11 );

  m_subImg   = m_nh.subscribe( "image_raw", 100, &bebopVSNodelet::imageCallback, this );
  image_transport::ImageTransport it(m_nh);
  m_pubCam    = it.advertise( "matching", 10, this );
  NODELET_INFO( "Publisher and subscriber set up. Waiting for image callbacks..." );


}



void bebopVSNodelet::imageCallback( const sensor_msgs::ImageConstPtr &msg )
{


  // std::cout << "Vel(x,wz) = " << VelX << ", " << VelWz << " Someval="<<Someval << std::endl;
  cv::Mat img;
  int success = 0;
  try{
      img=cv_bridge::toCvShare(msg,"bgr8")->image;
      success = 1;
  }catch (cv_bridge::Exception& e){
      NODELET_ERROR("Could not convert from '%s' to 'bgr8'.",
                msg->encoding.c_str());
  }

  geometry_msgs::Twist out_cmd_pos;
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

