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

#include "mpg_dice_node.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "mpgDiceMarker");
    ros::NodeHandle n;
    MPGDiceNode mpgNode(n);
    ros::spin();
    return 0;
}

MPGDiceNode::MPGDiceNode(ros::NodeHandle &n) : n_(n), imageTransport_(n) {

    // Register dynamic_reconfigure callback
    configCallbackFnct_ = boost::bind(&MPGDiceNode::configCallback, this ,  _1, _2);
    configServer_.setCallback(configCallbackFnct_);

    // Advert fiducial publisher
    pub_fiducials_ = n_.advertise<marker_msgs::FiducialDetection>("fiducials", 10);

    // Subscribe to image topic
    cameraSubscriber_ = imageTransport_.subscribeCamera("image", 1, &MPGDiceNode::imageCallback, this);
    
    if(config_.show_debug_image){
        cv::namedWindow("mpg_dice_debug", CV_WINDOW_NORMAL | CV_GUI_NORMAL);
    }

}

MPGDiceNode::~MPGDiceNode() {}

void MPGDiceNode::publishFiducials(const std_msgs::Header &header, std::vector<MPGDiceMarker> &markers, const sensor_msgs::CameraInfoConstPtr &camer_info_) {
    marker_msgs::FiducialDetection msg;
    msg.header = header;
    msg.camera_k = camer_info_->K;
    msg.camera_d = camer_info_->D;

    for (auto &marker:markers) {
        marker_msgs::Fiducial fiducial;

        // Add all object points
        for (auto &cvp: marker.get3DPoints(config_.marker_dot_displacement)) {
            geometry_msgs::Point point;
            point.x = cvp.x;
            point.y = cvp.y;
            point.z = cvp.z;
            fiducial.object_points.push_back(point);
        }

        // Add all images points
        for (cv::Point2f &p2d: marker.borderPoints) {
            geometry_msgs::Point point;
            point.x = p2d.x;
            point.y = p2d.y;
            point.z = 0.0f; // imagePoints are 2d
            fiducial.image_points.push_back(point);
        }

        fiducial.ids.resize(1);
        fiducial.ids_confidence.resize(1);
        fiducial.ids[0] = 0; // Id unkown
        fiducial.ids_confidence[0] = 1;

        msg.fiducial.push_back(fiducial);
    }

    pub_fiducials_.publish(msg);
}

static bool intersection(cv::Point2f o1, cv::Point2f p1, cv::Point2f o2, cv::Point2f p2, cv::Point2f &r)
{
    cv::Point2f x = o2 - o1;
    cv::Point2f d1 = p1 - o1;
    cv::Point2f d2 = p2 - o2;

    float cross = d1.x*d2.y - d1.y*d2.x;
    if (abs(cross) < /*EPS*/1e-8)
        return false;

    double t1 = (x.x * d2.y - x.y * d2.x)/cross;
    r = o1 + d1 * t1;
    return true;
}

void MPGDiceNode::imageCallback(const sensor_msgs::ImageConstPtr &image_msg, const sensor_msgs::CameraInfoConstPtr &camer_info_) {
    cv_bridge::CvImagePtr imgPtr;
    try {
        // Convert ros image message to cv::Mat
        imgPtr = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::MONO8);

        // Undistore image using camera info
        image_geometry::PinholeCameraModel cam_model;
        cam_model.fromCameraInfo(camer_info_);

        cv::Mat cameraMatrix(cam_model.intrinsicMatrix()); // intrinsic
        cv::Mat distortionCoeffs = cam_model.distortionCoeffs(); //distortion coefficients
        cv::Mat projectionMatrix(cam_model.projectionMatrix()); // projection

        cameraMatrix.convertTo(cameraMatrix, CV_64F) ;
        distortionCoeffs.convertTo(distortionCoeffs, CV_64F) ;

        /*
        float camera_matrix_data[9];
        for(int i = 0; i < 9; i++)
            camera_matrix_data[i] = camer_info_->K[i];
        cv::Mat camera_matrix = cv::Mat(3, 3, CV_32F, camera_matrix_data);

        float distortion_coefficients_data[5];
        for(int i = 0; i < 5; i++)
            distortion_coefficients_data[i] = camer_info_->D[i];
        cv::Mat distortion_coefficients = cv::Mat(1, 5, CV_32F, distortion_coefficients_data);
        */

        cv::Mat image;
        cv::undistort(imgPtr->image, image, cameraMatrix, distortionCoeffs);



        // Find blobs (circle detection)
        cv::SimpleBlobDetector::Params params;

        // Filter by Circularity
        if(config_.filter_circularity_enabled) {
            params.filterByCircularity = true;
            params.minCircularity = config_.filter_circularity_min;
            params.maxCircularity = config_.filter_circularity_max;
        }

        // Filter by Circularity
        if(config_.filter_convexity_enabled) {
            params.filterByConvexity = true;
            params.minConvexity = config_.filter_convexity_min;
            params.maxConvexity = config_.filter_convexity_max;
        }

        cv::Ptr<cv::FeatureDetector> blobsDetector = cv::SimpleBlobDetector::create(params);
        std::vector<cv::KeyPoint> keypoints;
        blobsDetector->detect(image, keypoints);


        std::vector<MPGDiceMarker> rawMarkers;
        if(keypoints.size() > 0){

            // Create kdTree
            cv::Mat features(keypoints.size(), 2, CV_32F);
            for (size_t i = 0; i < keypoints.size(); i++){
                features.at<float>(i, 0) = keypoints[i].pt.x;
                features.at<float>(i, 1) = keypoints[i].pt.y;
            }

            cv::flann::KDTreeIndexParams kdparams;
            cv::flann::Index kdtree(features, kdparams);

            // Iterate over all center point candidates and try to find border points
            for (size_t i = 0; i < keypoints.size(); i++) {
                cv::Point2f &point = keypoints[i].pt;

                // Query nearest neighbors
                std::vector<float> query;
                query.push_back(point.x);
                query.push_back(point.y);

                std::vector<int> indices;
                std::vector<float> dists;
                kdtree.knnSearch(query, indices, dists, 5);

                // store them as markerPoints
                std::vector<cv::Point2f> markerPoints;
                for (size_t z = 1; z < indices.size(); z++) {
                    if(dists[z] > 0)
                        markerPoints.push_back(keypoints[indices[z]].pt);
                }


                // Try to find only valid patterns
                // FIXME: This is not a very good discriminator, but it works for a first test.
                if(markerPoints.size() == 4) {

                    // Add center point
                    // markerPoints.push_back(point);

                    // Use point as centerPoint for sorting
                    MPGDiceMarker diceMarker(point, markerPoints);

                    cv::Point2f intercPoint;
                    if(intersection(diceMarker.borderPoints[0], diceMarker.borderPoints[2], diceMarker.borderPoints[1], diceMarker.borderPoints[3], intercPoint)) {
                        // Is this the dice center?
                        cv::Point2f diff = intercPoint-point;
                        float epsilon = 10;
                        if(sqrt(pow(diff.x, 2) + pow(diff.y, 2)) < epsilon){
                            // This seems to be a valid marker, use intercPoint as centerPoint
                            rawMarkers.push_back(MPGDiceMarker(intercPoint, markerPoints));
                        }
                    }
                }
            }

        }


        // Remove all markers which share border points/overlap
        // FIXME: Quick and ugly, this is only done to remove the checkerboard...
        std::vector<MPGDiceMarker> markers;
        for (auto &marker:rawMarkers) {
            bool overlap = false;

            // Border points and center point
            std::vector<cv::Point2f> points;
            for(auto &point:marker.borderPoints)
                points.push_back(point);
            points.push_back(marker.centerPoint);

            for(auto &fpoint:points) {
                for (auto &otherMarker:rawMarkers) {
                    if(marker != otherMarker) {
                        // Border points and center point
                        std::vector<cv::Point2f> otherPoints;
                        for(auto &point:otherMarker.borderPoints)
                            otherPoints.push_back(point);
                        otherPoints.push_back(otherMarker.centerPoint);

                        // Iterate trough all points...
                        for(auto &otherPoint:otherPoints) {

                            // Test if points are close -> equal
                            // FIXME: Instead of centerpoint use used blob
                            cv::Point2f diff = fpoint-otherPoint;
                            float epsilon = 10;
                            if(sqrt(pow(diff.x, 2) + pow(diff.y, 2)) < epsilon)
                                overlap = true;
                        }
                    }
                }
            }

            if(!overlap)
                markers.push_back(marker);
        }



        // Publish fiducials
        if(config_.publish_fiducials)
            publishFiducials(image_msg->header, markers, camer_info_);

        // Draw markers if debug image is enabled
        if(config_.show_debug_image){
            cv::Mat debugImage;
            cvtColor(image, debugImage, cv::COLOR_GRAY2BGR);

            for (size_t i = 0; i < keypoints.size(); i++){
                double radius = sqrt(keypoints[i].size/M_1_PI);
                cv::circle(debugImage, keypoints[i].pt, radius, cv::Scalar(255, 255, 255), -1);
            }

            for (auto &marker:markers) {
                for (size_t z = 0; z < marker.borderPoints.size(); z++) {
                    cv::line(debugImage, marker.centerPoint, marker.borderPoints[z], cv::Scalar(100, 100, 100));

                    std::string msg = cv::format( "%d", z);
                    cv::putText(debugImage, msg, marker.borderPoints[z], 1, 3, cv::Scalar(255, 255, 0));
                }

                cv::circle(debugImage, marker.centerPoint, 5, cv::Scalar(0, 0, 255), 1);
            }

            /*
            if(keypoints.size() > 0){


                // Create kdTree
                cv::Mat features(keypoints.size(), 2, CV_32F);
                for (size_t i = 0; i < keypoints.size(); i++){
                    features.at<float>(i, 0) = keypoints[i].pt.x;
                    features.at<float>(i, 1) = keypoints[i].pt.y;
                }

                cv::flann::KDTreeIndexParams kdparams(5);
                cv::flann::Index kdtree(features, kdparams);


                for (size_t i = 0; i < keypoints.size(); i++) {
                    cv::Point2f &point = keypoints[i].pt;

                    // Query nearest neighbors
                    std::vector<float> query;
                    query.push_back(point.x);
                    query.push_back(point.y);

                    std::vector<int> indices;
                    std::vector<float> dists;
                    kdtree.knnSearch(query, indices, dists, 5);

                    // store them as markerPoints
                    std::vector<cv::Point2f> markerPoints;
                    for (size_t z = 1; z < indices.size(); z++) {
                        if(dists[z] > 0)
                            markerPoints.push_back(keypoints[indices[z]].pt);
                    }

                    if(markerPoints.size() == 4){
                        //std::cout << indices.size() << std::endl;
                        //std::cout << cv::Mat(indices) << std::endl;
                        //std::cout << cv::Mat(dists) << std::endl;

                        // Add center point
                        markerPoints.push_back(point);

                        // FIXME: This is not a very good discriminator.
                        // Calculate mean point of borderPoints
                        cv::Point2f zero(0.0f, 0.0f);
                        cv::Point2f sum  = std::accumulate(markerPoints.begin(), markerPoints.end(), zero);
                        cv::Point2f mean(sum.x / markerPoints.size(), sum.y / markerPoints.size());

                        // Is this the dice center?
                        cv::Point2f diff = mean-point;
                        if(sqrt(pow(diff.x, 2) + pow(diff.y, 2)) < 5){

                            for (size_t z = 0; z < indices.size(); z++) {
                                cv::line(debugImage, keypoints[i].pt, keypoints[indices[z]].pt, cv::Scalar(100, 100, 100));
                            }

                            cv::circle(debugImage, mean, 5, cv::Scalar(0, 0, 255), 1);

                        }
                    }
                }

            }
            */

            cv::imshow("mpg_dice_debug", debugImage);
            cv::waitKey(5);
        }
    } catch (cv_bridge::Exception &e) {
        ROS_ERROR ("cv_bridge exception: %s", e.what());
        return;
    }
}

void MPGDiceNode::configCallback(tuw_marker_playground::DiceConfig &config, uint32_t level) {
    config_ = config;
}

struct ComparePoints
{
    MPGDiceMarker &mc;

    ComparePoints(MPGDiceMarker &mc) : mc(mc) {}

    bool operator()(cv::Point2f a, cv::Point2f b)
    {
        if (a.x - mc.centerPoint.x >= 0 && b.x - mc.centerPoint.x < 0)
            return true;
        if (a.x - mc.centerPoint.x < 0 && b.x - mc.centerPoint.x >= 0)
            return false;
        if (a.x - mc.centerPoint.x == 0 && b.x - mc.centerPoint.x == 0) {
            if (a.y - mc.centerPoint.y >= 0 || b.y - mc.centerPoint.y >= 0)
                return a.y > b.y;
            return b.y > a.y;
        }

        // compute the cross product of vectors (center -> a) x (center -> b)
        int det = (a.x - mc.centerPoint.x) * (b.y - mc.centerPoint.y) - (b.x - mc.centerPoint.x) * (a.y - mc.centerPoint.y);
        if (det < 0)
            return true;
        if (det > 0)
            return false;

        // points a and b are on the same line from the center
        // check which point is closer to the center
        int d1 = (a.x - mc.centerPoint.x) * (a.x - mc.centerPoint.x) + (a.y - mc.centerPoint.y) * (a.y - mc.centerPoint.y);
        int d2 = (b.x - mc.centerPoint.x) * (b.x - mc.centerPoint.x) + (b.y - mc.centerPoint.y) * (b.y - mc.centerPoint.y);
        return d1 > d2;
    }
};


MPGDiceMarker::MPGDiceMarker(cv::Point2f centerPoint, std::vector<cv::Point2f> borderPoints) {
    this->centerPoint = centerPoint;
    std::sort(borderPoints.begin(), borderPoints.end(), ComparePoints(*this));
    this->borderPoints = borderPoints;
}

MPGDiceMarker::~MPGDiceMarker() {}

std::vector<cv::Point3f> MPGDiceMarker::get3DPoints(float offset) {
    std::vector<cv::Point3f> objectPoints;
    objectPoints.push_back(cv::Point3f(-offset, offset, 0.0f));
    objectPoints.push_back(cv::Point3f(offset, offset, 0.0f));
    objectPoints.push_back(cv::Point3f(offset, -offset, 0.0f));
    objectPoints.push_back(cv::Point3f(-offset,-offset, 0.0f));
    return objectPoints;
}
