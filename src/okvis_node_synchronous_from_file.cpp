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

// #include <iostream>
// #include <fstream>
// #include <stdlib.h>
// #include <memory>
// #include <functional>
// #include <atomic>

// #include <Eigen/Core>
// #include <ros/ros.h>

// #pragma GCC diagnostic push
// #pragma GCC diagnostic ignored "-Wnon-virtual-dtor"
// #pragma GCC diagnostic ignored "-Woverloaded-virtual"
// #include <opencv2/opencv.hpp>
// #pragma GCC diagnostic pop
// #include <okvis/VioParametersReader.hpp>
// #include <okvis/ThreadedKFVio.hpp>
// #include <okvis/Publisher.hpp>

// #include <boost/filesystem.hpp>

// #include <ros/ros.h>
// #include <okvis/Publisher.hpp>
#include "okvis_app_synchronous_fn.hpp"

int main(int argc, char **argv){
  google::InitGoogleLogging(argv[0]);
  FLAGS_stderrthreshold = 0;  // INFO: 0, WARNING: 1, ERROR: 2, FATAL: 3
  FLAGS_colorlogtostderr = 1;

  if (argc != 3 && argc != 4) {
    LOG(ERROR)<<
    "Usage: ./" << argv[0] << " configuration-yaml-file dataset-folder [skip-first-seconds]";
    return -1;
  }

  okvis::Duration deltaT(0.0);
  if (argc == 4) {
    deltaT = okvis::Duration(atof(argv[3]));
  }

  // read configuration file
  std::string configFilename(argv[1]);
  // the folder path
  std::string path(argv[2]);

  PoseViewer poseViewer;
  return app_fn(path, configFilename, deltaT, 
    boost::optional<okvis::VioInterface::StateCallback>(), 
    boost::optional<okvis::VioInterface::FullStateCallback>(
                    std::bind(&PoseViewer::publishFullStateAsCallback, &poseViewer,
                    std::placeholders::_1, std::placeholders::_2,
                    std::placeholders::_3, std::placeholders::_4)), 
    boost::optional<okvis::VioInterface::LandmarksCallback>(), 
    boost::optional<std::function<void()>>(std::bind(&PoseViewer::display, &poseViewer)));

}


// int main(int argc, char **argv){
//   // ros::init(argc, argv, "okvis_node_synchronous");

//   google::InitGoogleLogging(argv[0]);
//   FLAGS_stderrthreshold = 0;  // INFO: 0, WARNING: 1, ERROR: 2, FATAL: 3
//   FLAGS_colorlogtostderr = 1;

//   if (argc != 3 && argc != 4) {
//     LOG(ERROR)<<
//     "Usage: ./" << argv[0] << " configuration-yaml-file dataset-folder [skip-first-seconds]";
//     return -1;
//   }

//   // set up the node
//   // ros::NodeHandle nh("okvis_node");

//   // publisher
//   // okvis::Publisher publisher(nh);

//   okvis::Duration deltaT(0.0);
//   if (argc == 4) {
//     deltaT = okvis::Duration(atof(argv[3]));
//   }

//   // read configuration file
//   std::string configFilename(argv[1]);
//   // the folder path
//   std::string path(argv[2]);

//   // return app_fn(path, configFilename, deltaT, 
//   //   boost::optional<okvis::VioInterface::StateCallback>(std::bind(&okvis::Publisher::publishStateAsCallback,&publisher,std::placeholders::_1,std::placeholders::_2)), 
//   //   boost::optional<okvis::VioInterface::FullStateCallback>(std::bind(&okvis::Publisher::publishFullStateAsCallback,&publisher,std::placeholders::_1,std::placeholders::_2,std::placeholders::_3,std::placeholders::_4)), 
//   //   boost::optional<okvis::VioInterface::LandmarksCallback>(std::bind(&okvis::Publisher::publishLandmarksAsCallback,&publisher,std::placeholders::_1,std::placeholders::_2,std::placeholders::_3)), 
//   //   boost::optional<std::function<void()>>());

//     return app_fn(path, configFilename, deltaT, 
//     boost::optional<okvis::VioInterface::StateCallback>(), 
//     boost::optional<okvis::VioInterface::FullStateCallback>(), 
//     boost::optional<okvis::VioInterface::LandmarksCallback>(), 
//     boost::optional<std::function<void()>>());



// }

// // this is just a workbench. most of the stuff here will go into the Frontend class.
// int main(int argc, char **argv)
// {
//   ros::init(argc, argv, "okvis_node_synchronous");

//   google::InitGoogleLogging(argv[0]);
//   FLAGS_stderrthreshold = 0;  // INFO: 0, WARNING: 1, ERROR: 2, FATAL: 3
//   FLAGS_colorlogtostderr = 1;

//   if (argc != 3 && argc != 4) {
//     LOG(ERROR)<<
//     "Usage: ./" << argv[0] << " configuration-yaml-file dataset-folder [skip-first-seconds]";
//     return -1;
//   }
//     // set up the node
//   ros::NodeHandle nh("okvis_node");

//   // publisher
//   okvis::Publisher publisher(nh);

//   okvis::Duration deltaT(0.0);
//   if (argc == 4) {
//     deltaT = okvis::Duration(atof(argv[3]));
//   }

//   // read configuration file
//   std::string configFilename(argv[1]);

//   okvis::VioParametersReader vio_parameters_reader(configFilename);
//   okvis::VioParameters parameters;
//   vio_parameters_reader.getParameters(parameters);

//   okvis::ThreadedKFVio okvis_estimator(parameters);

//   okvis_estimator.setBlocking(true);

//   // okvis_estimator.setFullStateCallback(std::bind(&okvis::Publisher::publishFullStateAsCallback,&publisher,std::placeholders::_1,std::placeholders::_2,std::placeholders::_3,std::placeholders::_4));
//   // okvis_estimator.setLandmarksCallback(std::bind(&okvis::Publisher::publishLandmarksAsCallback,&publisher,std::placeholders::_1,std::placeholders::_2,std::placeholders::_3));
//   // okvis_estimator.setStateCallback(std::bind(&okvis::Publisher::publishStateAsCallback,&publisher,std::placeholders::_1,std::placeholders::_2));

//   // the folder path
//   std::string path(argv[2]);

//   const unsigned int numCameras = parameters.nCameraSystem.numCameras();

//   // open the IMU file
//   std::string line;
//   std::ifstream imu_file(path + "/imu0/data.csv");
//   if (!imu_file.good()) {
//     LOG(ERROR)<< "no imu file found at " << path+"/imu0/data.csv";
//     return -1;
//   }
//   int number_of_lines = 0;
//   while (std::getline(imu_file, line))
//     ++number_of_lines;
//   LOG(INFO)<< "No. IMU measurements: " << number_of_lines-1;
//   if (number_of_lines - 1 <= 0) {
//     LOG(ERROR)<< "no imu messages present in " << path+"/imu0/data.csv";
//     return -1;
//   }
//   // set reading position to second line
//   imu_file.clear();
//   imu_file.seekg(0, std::ios::beg);
//   std::getline(imu_file, line);

//   std::vector<okvis::Time> times;
//   okvis::Time latest(0);
//   int num_camera_images = 0;
//   std::vector < std::vector < std::string >> image_names(numCameras);
//   for (size_t i = 0; i < numCameras; ++i) {
//     num_camera_images = 0;
//     std::string folder(path + "/cam" + std::to_string(i) + "/data");

//     for (auto it = boost::filesystem::directory_iterator(folder);
//         it != boost::filesystem::directory_iterator(); it++) {
//       if (!boost::filesystem::is_directory(it->path())) {  //we eliminate directories
//         num_camera_images++;
//         image_names.at(i).push_back(it->path().filename().string());
//       } else {
//         continue;
//       }
//     }

//     if (num_camera_images == 0) {
//       LOG(ERROR)<< "no images at " << folder;
//       return 1;
//     }

//     LOG(INFO)<< "No. cam " << i << " images: " << num_camera_images;
//     // the filenames are not going to be sorted. So do this here
//     std::sort(image_names.at(i).begin(), image_names.at(i).end());
//   }

//   std::vector < std::vector < std::string > ::iterator
//       > cam_iterators(numCameras);
//   for (size_t i = 0; i < numCameras; ++i) {
//     cam_iterators.at(i) = image_names.at(i).begin();
//   }

//   int counter = 0;
//   okvis::Time start(0.0);
//   while (true) {
//     okvis_estimator.display();

//     // check if at the end
//     for (size_t i = 0; i < numCameras; ++i) {
//       if (cam_iterators[i] == image_names[i].end()) {
//         std::cout << std::endl << "Finished. Press any key to exit." << std::endl << std::flush;
//         cv::waitKey();
//         return 0;
//       }
//     }

//     /// add images
//     okvis::Time t;

//     for (size_t i = 0; i < numCameras; ++i) {
//       cv::Mat filtered = cv::imread(
//           path + "/cam" + std::to_string(i) + "/data/" + *cam_iterators.at(i),
//           cv::IMREAD_GRAYSCALE);
//       std::string nanoseconds = cam_iterators.at(i)->substr(
//           cam_iterators.at(i)->size() - 13, 9);
//       std::string seconds = cam_iterators.at(i)->substr(
//           0, cam_iterators.at(i)->size() - 13);
//       t = okvis::Time(std::stoi(seconds), std::stoi(nanoseconds));
//       if (start == okvis::Time(0.0)) {
//         start = t;
//       }

//       // get all IMU measurements till then
//       okvis::Time t_imu = start;
//       do {
//         if (!std::getline(imu_file, line)) {
//           std::cout << std::endl << "Finished. Press any key to exit." << std::endl << std::flush;
//           cv::waitKey();
//           return 0;
//         }

//         std::stringstream stream(line);
//         std::string s;
//         std::getline(stream, s, ',');
//         std::string nanoseconds = s.substr(s.size() - 9, 9);
//         std::string seconds = s.substr(0, s.size() - 9);

//         Eigen::Vector3d gyr;
//         for (int j = 0; j < 3; ++j) {
//           std::getline(stream, s, ',');
//           gyr[j] = std::stof(s);
//         }

//         Eigen::Vector3d acc;
//         for (int j = 0; j < 3; ++j) {
//           std::getline(stream, s, ',');
//           acc[j] = std::stof(s);
//         }

//         t_imu = okvis::Time(std::stoi(seconds), std::stoi(nanoseconds));

//         // add the IMU measurement for (blocking) processing
//         if (t_imu - start + okvis::Duration(1.0) > deltaT) {
//           okvis_estimator.addImuMeasurement(t_imu, acc, gyr);
//         }

//       } while (t_imu <= t);

//       // add the image to the frontend for (blocking) processing
//       if (t - start > deltaT) {
//         okvis_estimator.addImage(t, i, filtered);
//       }

//       cam_iterators[i]++;
//     }
//     ++counter;

//     // display progress
//     if (counter % 20 == 0) {
//       std::cout << "\rProgress: "
//           << int(double(counter) / double(num_camera_images) * 100) << "%  "
//           << std::flush;
//     }

//   }

//   std::cout << std::endl << std::flush;
//   return 0;
// }