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

#include "mpg_ellipses_node.h"

bool show_debug_image = true;

int main(int argc, char **argv) {
    ros::init(argc, argv, "mpgEllipses");
    ros::NodeHandle n;
    MPGEllipsesNode mpgNode(n);
    ros::spin();
    return 0;
}

MPGEllipsesNode::MPGEllipsesNode(ros::NodeHandle &n) : n_(n), imageTransport_(n) {

    // Advert fiducial publisher
    pub_fiducials_ = n_.advertise<marker_msgs::FiducialDetection>("fiducials", 10);

    // Subscribe to image topic
    cameraSubscriber_ = imageTransport_.subscribeCamera("image", 1, &MPGEllipsesNode::imageCallback, this);
    
    if(show_debug_image){
        cv::namedWindow("mpg_node_debug", CV_WINDOW_NORMAL | CV_GUI_NORMAL);
    }

}

MPGEllipsesNode::~MPGEllipsesNode() {}

void MPGEllipsesNode::publishFiducials(const std_msgs::Header &header, std::vector<cv::KeyPoint> &keypoints, const sensor_msgs::CameraInfoConstPtr &camer_info_) {
    marker_msgs::FiducialDetection msg;
    msg.header = header;
    msg.camera_k = camer_info_->K;
    msg.camera_d = camer_info_->D;

    for (auto &keypoint:keypoints) {
        marker_msgs::Fiducial fiducial;

        geometry_msgs::Point objectPoint;
        objectPoint.x = 0.0f;
        objectPoint.y = 0.0f;
        objectPoint.z = 0.0f;
        fiducial.object_points.push_back(objectPoint);

        geometry_msgs::Point imagePoint;
        imagePoint.x = keypoint.pt.x;
        imagePoint.y = keypoint.pt.y;
        imagePoint.z = 0.0f; // imagePoints are 2d
        fiducial.image_points.push_back(imagePoint);

        fiducial.ids.resize(0);
        fiducial.ids_confidence.resize(0);

        msg.fiducial.push_back(fiducial);
    }

    pub_fiducials_.publish(msg);
}

void MPGEllipsesNode::imageCallback(const sensor_msgs::ImageConstPtr &image_msg, const sensor_msgs::CameraInfoConstPtr &camer_info_) {
    cv_bridge::CvImagePtr imgPtr;
    try {
        // Convert ros image message to cv::Mat
        imgPtr = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::MONO8);

        cv::SimpleBlobDetector::Params params;

        // Filter by Circularity
        params.filterByCircularity = true;
        params.minConvexity = 0.95;

        // Filter by Convexity
        params.filterByConvexity = true;
        params.minConvexity = 0.95;

        cv::Ptr<cv::FeatureDetector> blobsDetector = cv::SimpleBlobDetector::create(params);
        std::vector<cv::KeyPoint> keypoints;
        blobsDetector->detect(imgPtr->image, keypoints);

        // Publish fiducials
        publishFiducials(image_msg->header, keypoints, camer_info_);

        // Draw markers if debug image is enabled
        if(show_debug_image){
            cv::Mat debugImage;
            cvtColor(imgPtr->image, debugImage, cv::COLOR_GRAY2BGR);

            for (size_t i = 0; i < keypoints.size(); ++i){
                double radius = sqrt(keypoints[i].size/M_1_PI);
                cv::circle(debugImage, keypoints[i].pt, radius, cv::Scalar(255, 0, 255), -1);
            }

            cv::imshow("mpg_node_debug", debugImage);
            cv::waitKey(5);
        }
    } catch (cv_bridge::Exception &e) {
        ROS_ERROR ("cv_bridge exception: %s", e.what());
        return;
    }
}
