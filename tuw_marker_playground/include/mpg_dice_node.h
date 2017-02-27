/*
 * Copyright (c) 2016, Lukas Pfeifhofer <lukas.pfeifhofer@devlabs.pro>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors
 * may be used to endorse or promote products derived from this software without
 * specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef TUW_MPG_DICE_NODE_H
#define TUW_MPG_DICE_NODE_H

#include "ros/ros.h"
#include <image_transport/image_transport.h>
#include <image_geometry/pinhole_camera_model.h>
#include <tf/transform_broadcaster.h>
#include <marker_msgs/MarkerDetection.h>
#include <marker_msgs/FiducialDetection.h>

#include <dynamic_reconfigure/server.h>
#include <tuw_marker_playground/DiceConfig.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include "opencv2/highgui/highgui.hpp"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>

class MPGDiceMarker {
public:
    MPGDiceMarker(cv::Point2f centerPoint, std::vector<cv::Point2f> borderPoints);

    ~MPGDiceMarker();

    cv::Point2f centerPoint;
    std::vector<cv::Point2f> borderPoints;

    std::vector<cv::Point3f> get3DPoints(float offset);

    friend bool operator==(const MPGDiceMarker &m1, const MPGDiceMarker &m2) {
        // Only compare center Point
        return (m1.centerPoint == m2.centerPoint);
    }

    friend bool operator!=(const MPGDiceMarker &m1, const MPGDiceMarker &m2) {
        return !(m1 == m2);
    }

};

class MPGDiceNode {
public:
    MPGDiceNode(ros::NodeHandle &n);
    ~MPGDiceNode();

    dynamic_reconfigure::Server<tuw_marker_playground::DiceConfig> configServer_;
    dynamic_reconfigure::Server<tuw_marker_playground::DiceConfig>::CallbackType configCallbackFnct_;

private:
    ros::NodeHandle n_;

    image_transport::ImageTransport imageTransport_;
    image_transport::CameraSubscriber cameraSubscriber_;

    ros::Publisher pub_fiducials_;

    tuw_marker_playground::DiceConfig config_;


    void imageCallback(const sensor_msgs::ImageConstPtr &image_msg, const sensor_msgs::CameraInfoConstPtr &camer_info_);

    void publishFiducials(const std_msgs::Header &header, std::vector<MPGDiceMarker> &markers, const sensor_msgs::CameraInfoConstPtr &camer_info_);

    void configCallback(tuw_marker_playground::DiceConfig &config, uint32_t level);
};

#endif // TUW_MPG_DICE_NODE_H