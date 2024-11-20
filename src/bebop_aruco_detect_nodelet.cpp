
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


#define TWAIT 60
#define HERTZ 10


class bebopVSNodelet : public nodelet::Nodelet
{
public:
  bebopVSNodelet();
  virtual ~bebopVSNodelet();

  virtual void onInit();
  // void timerCallback(const  ros::TimerEvent& event );
  void timerCallback();
  void imageCallback( const sensor_msgs::ImageConstPtr &msg );
  void odomCallback( const nav_msgs::Odometry &msg );
  void landCallback( const std_msgs::Empty &msg  );
  void takeOffCallback( const std_msgs::Empty &msg  );
  // void timerCounterCallback(const  ros::TimerEvent& event);

private:
  ros::NodeHandle m_nh;
  ros::Publisher m_pubTwist;
  ros::Subscriber m_subImg;
  ros::Subscriber m_land;
  ros::Subscriber m_takeOff;
  ros::Subscriber m_subOdom;
  image_transport::Publisher m_pubCam;

  //  For openloop TOBETESTED
  ros::Timer _timer;
  int timerCounter = 0;
  // _timer = nh.createTimer(ros::Duration(1.0),
  //                         & hummingbirdNodelet::timerCb, this);

  bool bebop_has_been_setup = false;
  bool waiting = true;

  //  BEGIN Openloop params
  double VelX = 0.;
  double VelY = 0.;
  double VelZ = 0.;
  double VelWz = 0.;
  //  END Openloop params
  //  BEGIN IBVS
  //  Image for desired pose
  std::string reference;
  //  orb detector
  // Ptr<ORB> orb;
  //  image message
  sensor_msgs::ImagePtr image_msg;
  // Visual control state
  vct::state State;
  // Result of the matching operation
  vct::matching_result Matching_result;
  //  END IBVS


  // double m_cameraTilt;
  // double m_cameraPan;

  // bool m_vec_ip_has_been_sorted;

  //  configs
  bool verbose = false;
  //  Select behavior [0 = openloop, 1 = IBVS]
  int select = 0;
  std::string output;

  //  correction
  cv::Mat R,t ;

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
  State.load(m_nh);
  verbose = m_nh.param(std::string("verbose"), false);
  VelX = m_nh.param(std::string("VelX"), 0.);
  VelY = m_nh.param(std::string("VelY"), 0.);
  VelZ = m_nh.param(std::string("VelZ"), 0.);
  VelWz = m_nh.param(std::string("VelWz"), 0.);
  select = m_nh.param(std::string("select"), 0);
  reference = m_nh.param(std::string("reference"), std::string("reference.png"));
  output = m_nh.param(std::string("output"), std::string("output.dat"));

  // // NODELET_INFO("Read params");
  // image_transport::ImageTransport it(m_nh);
  //
  // /****************** CREATING PUBLISHER AND SUBSCRIBER */
  // // image_sub = it.subscribe("camera_nadir/image_raw",100,&hummingbirdNodelet::imageCallback);
  // image_sub = it.subscribe("camera_nadir/image_raw",2,&hummingbirdNodelet::imageCallback, this);
  // image_pub = it.advertise("matching",2, this);
  // // ros::Rate rate(40);
  // /******************************** MOVING TO A POSE */
  // pos_pub = m_nh.advertise<trajectory_msgs::MultiDOFJointTrajectory>("command/trajectory",2, this);
  // // pos_sub = m_nh.subscribe<geometry_msgs::Pose>("ground_truth/pose",100, &hummingbirdNodelet::poseCallback);
  // pos_sub = m_nh.subscribe<geometry_msgs::Pose>("ground_truth/pose",2, &hummingbirdNodelet::poseCallback, this);


  /*************************** OPENING DESIRED IMAGE */
  // NODELET_INFO("REF: %s ",reference.c_str());

  if (select ==0)
  {
    std::cout << "[BEBOP2 ARUCO] Cruise control at V= " << VelX << ", " << VelY << ", "  << VelZ << ", " << VelWz << std::endl << std::flush;
  }

  if (select == 1)
  {
    State.Desired_Configuration.img = cv::imread(reference,cv::IMREAD_GRAYSCALE);
    if(State.Desired_Configuration.img.empty()) {
          NODELET_INFO( "Could not open or find the reference image");
          return;
    }
    std::vector<std::vector<cv::Point2f> > _rejected;
    // State.params.initArucos(cv::aruco::DICT_4X4_250);
    State.params.initArucos(cv::aruco::DICT_6X6_1000);
    cv::aruco::detectMarkers(State.Desired_Configuration.img,
                             State.params.dictionary,
                             State.Desired_Configuration.arucos,
                             State.Desired_Configuration.arucos_ids,
                             State.params.parameters,
                             _rejected);
    State.params.initArucos(cv::aruco::DICT_6X6_1000);
    std::cout << "n Arucos = " << State.Desired_Configuration.arucos_ids.size() << std::endl << std::flush;
    int len = State.Desired_Configuration.arucos_ids.size();
    for (int i = 0; i < len ; i++)
      for (int j = 0; j < 4 ; j++)
        std::cout <<  State.Desired_Configuration.arucos[i][j]  << std::flush;
    std::cout << std::endl << std::flush;
    NODELET_INFO("[BEBOP2 ARUCO] INIT");
  }

  //  END IBVS

  NODELET_INFO( "[BEBOP2 ARUCO] Setting up publisher and subscriber..." );
  if (verbose)
    m_subOdom   = m_nh.subscribe( "odom", 100, &bebopVSNodelet::odomCallback, this );

  //  Timer config
  _timer = m_nh.createTimer(ros::Duration(1.0/HERTZ), boost::bind(& bebopVSNodelet::timerCallback, this));
  // if (select == 0)
  //   _timer = m_nh.createTimer(ros::Duration(1.0/10.), & bebopVSNodelet::timerCallback, this);
  // else
  //   _timer = m_nh.createTimer(ros::Duration(1.0), & bebopVSNodelet::timerCounterCallback, this);


  if (select == 1)
    m_subImg   = m_nh.subscribe( "image_raw", 100, &bebopVSNodelet::imageCallback, this );

  image_transport::ImageTransport it(m_nh);
  m_pubTwist  = m_nh.advertise< geometry_msgs::Twist >( "cmd_vel", 1 , this);
  m_pubCam    = it.advertise( "matching", 10, this );
  m_land      = m_nh.subscribe( "land", 2, &bebopVSNodelet::landCallback, this );
  m_takeOff   = m_nh.subscribe( "takeoff", 2, &bebopVSNodelet::takeOffCallback, this );
  NODELET_INFO( "Publisher and subscriber set up. Waiting for image callbacks..." );


  //  needed data
  R = cv::Mat::zeros(3,3,CV_64F);
  t = cv::Mat::zeros(3,1,CV_64F);
  R.at<double>(0,2) = 1.;
  R.at<double>(1,0) = -1.;
  R.at<double>(2,1) = -1.;
  t.at<double>(2,0) = -0.09;

  State.Z = 1.;

}


void bebopVSNodelet::odomCallback( const nav_msgs::Odometry &msg  )
{
  std::cout << "Odom_Pose = " //<< std::endl;
  << (float) msg.pose.pose.position.x << ", "
  << (float) msg.pose.pose.position.y << ", "
  << (float) msg.pose.pose.position.z
  << std::endl;
}
void bebopVSNodelet::landCallback( const std_msgs::Empty &msg  )
{
  waiting = true;
  VelX = 0.;
  VelY = 0.;
  VelZ = 0.;
  VelWz = 0.;
  timerCounter = 0;
}
void bebopVSNodelet::takeOffCallback( const std_msgs::Empty &msg  )
{
  waiting = false;
  timerCounter = 0;
}



// void bebopVSNodelet::timerCallback(const  ros::TimerEvent& event)
void bebopVSNodelet::timerCallback()
{
  // ROS_INFO("TIMER TIC");
  if (waiting)
    return;
  if (timerCounter < TWAIT)
  {
    timerCounter++;
    std::cout << "Counter = " << timerCounter << std::endl << std::flush;
    return;
  }
  if (select != 0)
    return;
  // ROS_INFO("TIMER EXEC C");
  //  BEGIN Openloop
  geometry_msgs::Twist out_cmd_pos;
  out_cmd_pos.linear.x  = VelX;
  // out_cmd_pos.linear.x  = 0.;
  out_cmd_pos.linear.y  = VelY;
  // out_cmd_pos.linear.y  = 0.;
  out_cmd_pos.linear.z  = VelZ;
  // out_cmd_pos.linear.z  = 0.;
  out_cmd_pos.angular.x = 0.;
  out_cmd_pos.angular.y = 0.;
  // out_cmd_pos.angular.z = 0.;
  out_cmd_pos.angular.z = VelWz;
  std::cout << " V= " << VelX << ", " << VelY << ", "  << VelZ << ", " << VelWz <<  "---"  << std::endl << std::flush;
  m_pubTwist.publish( out_cmd_pos );
  // ROS_INFO("CRUISE");
  //  END Openloop
}
// void bebopVSNodelet::timerCounterCallback(const  ros::TimerEvent& event)
// {
//   if (waiting)
//     return;
//   if (timerCounter <TWAIT)
//   {
//     timerCounter++;
//     std::cout << "Counter = " << timerCounter << std::endl << std::flush;
//   }
//     return;
// }

void tanh(cv::Mat & source, cv::Mat & target)
{
  cv::Mat expPos, expNeg, num, div;
  cv::exp(source, expPos);
  cv::exp(-source, expNeg);
  num = expPos-expNeg;
  div = expPos+expNeg;
  target = num/div;
}

void bebopVSNodelet::imageCallback( const sensor_msgs::ImageConstPtr &msg )
{
  // ROS_INFO("IBVS init");
  // if ( !bebop_has_been_setup )
  // {
  //   geometry_msgs::Twist out_cam_ctrl;
  //   out_cam_ctrl.linear.x  = 0;
  //   out_cam_ctrl.linear.y  = 0;
  //   out_cam_ctrl.linear.z  = 0;
  //   out_cam_ctrl.angular.x = 0;
  //   out_cam_ctrl.angular.y = m_cameraTilt;
  //   out_cam_ctrl.angular.z = m_cameraPan;
  //   m_pubTwist.publish( out_cam_ctrl );
  //
  //   bebop_has_been_setup = true;
  //   NODELET_INFO( "Setting desired camera orientation..." );
  // }


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
    // std::cout << img.rows << ", " << img.cols << std::endl << std::flush;

    State.Vx = 0.;
    State.Vy = 0.;
    State.Vz = 0.;
    State.Vroll = 0.;
    State.Vpitch = 0.;
    State.Vyaw = 0.;
    if (vct::preprocessors[2](img, State.params, State.Desired_Configuration ,Matching_result)!= 0)
        return;

    /***************************** Prepare message */
    // if(Matching_result.mean_feature_error < State.params.feature_threshold)
    //   return;

    image_msg = cv_bridge::CvImage(std_msgs::Header(),
                                  sensor_msgs::image_encodings::BGR8,
                                  Matching_result.img_matches).toImageMsg();
    image_msg->header.frame_id = "matching_image";
    image_msg->width = Matching_result.img_matches.cols;
    image_msg->height = Matching_result.img_matches.rows;
    image_msg->is_bigendian = false;
    image_msg->step = sizeof(unsigned char) * Matching_result.img_matches.cols*3;
    image_msg->header.stamp = ros::Time::now();
    // Publish image of the matching
    m_pubCam.publish(image_msg);
    if(Matching_result.mean_feature_error < State.params.feature_threshold)
      return;

    //  Invert camera coordinates
    // Matching_result.p1.col(1) = -1. * Matching_result.p1.col(1);
    // Matching_result.p2.col(1) = -1. * Matching_result.p2.col(1);

    //  Compute control
    if (vct::controllers[2]( State, Matching_result)!= 0)
        return;

    // Transformation
    // ROS_INFO("[BEBOP2 ARUCO] Control computed");
    cv::Mat _V(3,1,CV_64F);
    cv::Mat _W(3,1,CV_64F);

    _V.at<double>(0,0) = State.Vx;
    _V.at<double>(1,0) = State.Vy;
    _V.at<double>(2,0) = State.Vz;
    // _V = 0.1*_V;
    _W.at<double>(0,0) = State.Vroll;
    _W.at<double>(1,0) = State.Vpitch;
    _W.at<double>(2,0) = State.Vyaw;
    // _W = 0.1*_W;
    // std::cout << "CONTROL = " << _V.t() << _W.t() << std::endl << std::flush;
    _V = R * _V + _W.cross(t);
    _W = R * _W;

    // _V.at<double>(2,0) *= 10.; // Corrección por las ganacias de autonomy
    // std::cout << "CONTROL = " << _V.t() << _W.t() << std::endl << std::flush;
    std::cout << "CONTROL = " << _V.at<double>(0,0) << ", "
    << _V.at<double>(1,0) << ", "
    << _V.at<double>(2,0) << ", "
    << _W.at<double>(5,0) << ", "
    << std::endl << std::flush;
    //  Límites de seguridad

    //  TANH
    // tanh(_V,_V);
    // _V = 0.01 * _V;
    // tanh(_W,_W);
    // _W = 0.01 * _W;

    //  SATURATION
    // _V = 0.01 * _V;
    // _V.setTo(0.01, _V > 0.01);
    // _V.setTo(-0.01, _V < -0.01);
    // _W = 0.01 * _W;
    // _W.setTo(0.01, _W > 0.01);
    // _W.setTo(-0.01, _W < -0.01);

    //  CLIPPING
    _V.setTo(0.01, _V > 0.00001);
    _V.setTo(-0.01, _V < -0.00001);
    _W.setTo(0.01, _W > 0.00001);
    _W.setTo(-0.01, _W < -0.00001);

    //  MAXIMAL NORM
    // double Vnorm = cv::norm(_V, cv::NORM_L2);
    // _V = 0.01 * _V / Vnorm;
    // Vnorm = cv::norm(_W, cv::NORM_L2);
    // _W = 0.01 * _W / Vnorm;


    // std::cout << "CONTROL = " << _V.t() << _W.t() << std::endl << std::flush;

    //  Change control
    // VelX  = _V.at<double>(0,0);
    // VelY  = _V.at<double>(1,0);
    // VelZ  = _V.at<double>(2,0);
    // VelWz = -_W.at<double>(5,0);

    // ROS_INFO("IBVS succeded");
    // //  Send message
    if ( timerCounter < TWAIT ) return;
    std::cout << "CONTROL = " << _V.t() << _W.t() << std::endl << std::flush;
    out_cmd_pos.linear.x  = (double) _V.at<double>(0,0);
    out_cmd_pos.linear.y  = (double) _V.at<double>(1,0);
    out_cmd_pos.linear.z  = (double) _V.at<double>(2,0);
    out_cmd_pos.angular.x = 0.;
    out_cmd_pos.angular.y = 0.;
    out_cmd_pos.angular.z = 0.;
    // out_cmd_pos.angular.z = (double)  _W.at<double>(5,0);
    m_pubTwist.publish( out_cmd_pos );
    // // ROS_INFO("IBVS succeded");

  }

//   else
//   {
//
//     std::stringstream sserr;
//     sserr << "Failed to detect an Apriltag, or detected multiple ones";
//
// //     vpDisplay::displayText( I, 10, 10, "Click to exit", vpColor::red );
// //     vpDisplay::flush( I );
//
//     // Stoping drone movement
//     out_cmd_pos.linear.x  = 0;
//     out_cmd_pos.linear.y  = 0;
//     out_cmd_pos.linear.z  = 0;
//     out_cmd_pos.angular.x = 0;
//     out_cmd_pos.angular.y = 0;
//     out_cmd_pos.angular.z = 0;
//     m_pubTwist.publish( out_cmd_pos );
//   }
}

