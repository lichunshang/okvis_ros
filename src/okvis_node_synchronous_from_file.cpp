/*********************************************************************************
 *  OKVIS - Open Keyframe-based Visual-Inertial SLAM
 *  Copyright (c) 2015, Autonomous Systems Lab / ETH Zurich
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 * 
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright notice,
 *     this list of conditions and the following disclaimer in the documentation
 *     and/or other materials provided with the distribution.
 *   * Neither the name of Autonomous Systems Lab / ETH Zurich nor the names of
 *     its contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 *  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 *  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 *  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 *  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 *  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 *  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 *  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *  Created on: Jun 26, 2013
 *      Author: Stefan Leutenegger (s.leutenegger@imperial.ac.uk)
 *    Modified: Andreas Forster (an.forster@gmail.com)
 *********************************************************************************/

/**
 * @file okvis_app_synchronous.cpp
 * @brief This file processes a dataset.
 
 This node goes through a dataset in order and waits until all processing is done
 before adding a new message to algorithm

 * @author Stefan Leutenegger
 * @author Andreas Forster
 */

#include <ros/ros.h>
#include <okvis/Publisher.hpp>
#include "okvis_app_synchronous_fn.hpp"

int main(int argc, char **argv){
  ros::init(argc, argv, "okvis_node_synchronous");

  google::InitGoogleLogging(argv[0]);
  FLAGS_stderrthreshold = 0;  // INFO: 0, WARNING: 1, ERROR: 2, FATAL: 3
  FLAGS_colorlogtostderr = 1;

  if (argc != 3 && argc != 4) {
    LOG(ERROR)<<
    "Usage: ./" << argv[0] << " configuration-yaml-file dataset-folder [skip-first-seconds]";
    return -1;
  }

  // set up the node
  ros::NodeHandle nh("okvis_node");

  // publisher
  okvis::Publisher publisher(nh, true);

  okvis::Duration deltaT(0.0);
  if (argc == 4) {
    deltaT = okvis::Duration(atof(argv[3]));
  }

  // read configuration file
  std::string configFilename(argv[1]);
  // the folder path
  std::string path(argv[2]);

  publisher.setCsvFile(path + "/okvis_estimator_output.csv");
  publisher.setLandmarksCsvFile(path + "/okvis_estimator_landmarks.csv");

  auto ros_check = []() {
      if (!ros::ok()) {
        return false;
      }
      return true;
  };

  return app_fn(path, configFilename, deltaT, 
    boost::optional<okvis::VioInterface::StateCallback>(std::bind(&okvis::Publisher::publishStateAsCallback,&publisher,std::placeholders::_1,std::placeholders::_2)), 
    boost::optional<okvis::VioInterface::FullStateCallback>(std::bind(&okvis::Publisher::publishFullStateAsCallback,&publisher,std::placeholders::_1,std::placeholders::_2,std::placeholders::_3,std::placeholders::_4)), 
    boost::optional<okvis::VioInterface::LandmarksCallback>(std::bind(&okvis::Publisher::publishLandmarksAsCallback,&publisher,std::placeholders::_1,std::placeholders::_2,std::placeholders::_3)), 
    boost::optional<std::function<void()>>(), 
    boost::optional<std::function<bool()>>(ros_check));
}
