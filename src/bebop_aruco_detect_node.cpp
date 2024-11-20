/****************************************************************************
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2019 by INRIA. All rights reserved.
 *
 * This software is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * ("GPL") version 2 as published by the Free Software Foundation.
 * See the file LICENSE.txt at the root directory of this source
 * distribution for additional information about the GNU GPL.
 *
 * For using ViSP with software that can not be combined with the GNU
 * GPL, please contact INRIA about acquiring a ViSP Professional
 * Edition License.
 *
 * See http://visp.inria.fr/ for more information.
 *
 * This software was developed at:
 * INRIA Rennes - Bretagne Atlantique
 * Campus Universitaire de Beaulieu
 * 35042 Rennes Cedex
 * France
 * https://team.inria.fr/rainbow/fr/
 *
 * If you have questions regarding the use of this file, please contact
 * INRIA at visp@inria.fr
 *
 * This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
 * WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 * Description:
 * Visual servoing node to control Parrot Bebop 2 drone.
 *
 * Authors:
 * Gatien Gaumerais
 * Fabien Spindler
 *
 *****************************************************************************/

#include <algorithm>
#include <string>
#include <vector>

#include <nodelet/loader.h>
// #include <rclcpp/rclcpp.hpp>
#include <ros/ros.h>


int
main( int argc, char *argv[] )
{
  ros::init( argc, argv, "bebop_aruco_detect", ros::init_options::NoSigintHandler );
  nodelet::Loader nll;

  nodelet::M_string remap( ros::names::getRemappings() );
  nodelet::V_string nargv;
  const std::string nl_name = ros::this_node::getName();
  nll.load( nl_name, "bebop_aruco_detect_nodelet", remap, nargv );

  ROS_INFO("[DEMO BEBOP 2] INIT");
  const std::vector< std::string > &loaded_nodelets = nll.listLoadedNodelets();
  if ( std::find( loaded_nodelets.begin(),
                  loaded_nodelets.end(),
                  nl_name ) == loaded_nodelets.end() )
  {
    // Nodelet OnInit() failed
    ROS_FATAL( "bebop_aruco_detect nodelet failed to load." );
    return 1;
  }

  // It reaches here when OnInit() succeeds
  ROS_INFO( "bebop_aruco_detect nodelet loaded." );
  ros::spin();
  ROS_INFO("[DEMO BEBOP 2] AFTER");
  return 0;
}
