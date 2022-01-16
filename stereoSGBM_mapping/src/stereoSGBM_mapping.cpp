/*  Author: Victor M
*   Email: victorcmitchell@gmail.com
*   Notes:  - 
*/

#include <chrono>
#include <thread>

#include <ros/ros.h>
#include <ros/time.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/distortion_models.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/imgcodecs.hpp>

std::string gstreamerPipelineStr(int cap_width, int cap_height, int frame_rate, int cap_id) {
    return "nvarguscamerasrc sensor-id=" + std::to_string(cap_id)
        + " exposuretimerange='" + std::to_string(13000) + " " + std::to_string(100000000) + "' aelock=false"
        + " gainrange='" + std::to_string(1) + " " + std::to_string(10) + "'"
        + " ispdigitalgainrange='" + std::to_string(1) + " " + std::to_string(1) + "'"
        + " aeantibanding=" + std::to_string(0) + " tnr-mode=" + std::to_string(1)
        + " tnr-strength=" + std::to_string(0.5) + " ee-mode=" + std::to_string(0)
        + " ee-strength=" + std::to_string(0.5)
        + " ! video/x-raw(memory:NVMM), width=" + std::to_string(cap_width)
        + ", height=" + std::to_string(cap_height) + ", format=(string)NV12, "
        + "framerate=(fraction)" + std::to_string(frame_rate) + "/1 ! nvvidconv flip-method=0"
        + " ! video/x-raw, width =" + std::to_string(cap_width) + ", height=" + std::to_string(cap_height) + ", format=(string)BGRx ! "
        + "videoconvert ! video/x-raw, format=(string)BGR ! appsink";
}

void publishStereoMappingData(cv::Mat& depth, cv::Mat& left_img, uint32_t seq, cv::Size size, 
    const ros::Publisher& rgb_pub, const ros::Publisher& meta_pub, const ros::Publisher& depth_pub) {

    sensor_msgs::Image rgb_msg;
    sensor_msgs::CameraInfo meta_msg;
    sensor_msgs::Image depth_msg;

    std_msgs::Header header;
    header.seq = seq;
    header.stamp = ros::Time::now();

    rgb_msg.header = header;
    meta_msg.header = header;
    depth_msg.header = header;

    cv::Mat color;
    cv::cvtColor(left_img, color, cv::COLOR_BGR2RGB);
    // Encode color image data into image msg
    cv_bridge::CvImage rgb_bridge;
    rgb_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::RGB16, color);
    rgb_bridge.toImageMsg(rgb_msg);

    meta_msg.width = size.width;
    meta_msg.height = size.height;

    cv_bridge::CvImage depth_bridge;
    depth_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::TYPE_32FC1, depth);

    // Publish mapping msgs
    rgb_pub.publish(rgb_msg);
    meta_pub.publish(meta_msg);
    depth_pub.publish(depth_msg);
}

// Stores relevent calibration values
class StereoCalibration {
private:
    cv::Size size;
    cv::Mat l_cameraMat, l_distCoeffs;
    cv::Mat r_cameraMat, r_distCoeffs;
    cv::Mat l_rtR, l_rtP;
    cv::Mat r_rtR, r_rtP;
    cv::Mat Q;
    cv::Mat l_map1, l_map2;
    cv::Mat r_map1, r_map2;

public:
    StereoCalibration(std::string& xml_calib) {
        cv::FileStorage fs(xml_calib, cv::FileStorage::READ);
        fs["image_size"] >> size;
        fs["left_camera_matrix"] >> l_cameraMat;
        fs["right_camera_matrix"] >> r_cameraMat;
        fs["left_dist_coeffs"] >> l_distCoeffs;
        fs["right_dist_coeffs"] >> r_distCoeffs;
        fs["left_rectification_rotation_matrix"] >> l_rtR;
        fs["right_rectification_rotation_matrix"] >> r_rtR;
        fs["left_rectification_projection_matrix"] >> l_rtP;
        fs["right_rectification_projection_matrix"] >> r_rtP;
        fs["disparity_to_depth_matrix"] >> Q;
        fs.release();

        cv::initUndistortRectifyMap(l_cameraMat, l_distCoeffs, l_rtR, l_rtP,
                                        size, CV_16SC2, l_map1, l_map2);
        cv::initUndistortRectifyMap(r_cameraMat, r_distCoeffs, r_rtR, r_rtP,
                                        size, CV_16SC2, r_map1, r_map2);
    }

    void applyCalibration(cv::Mat& left_img, cv::Mat& right_img, cv::Mat& left_dst, cv::Mat& right_dst) {
        cv::remap(left_img, left_dst, l_map1, l_map2, cv::INTER_LINEAR);
        cv::remap(right_img, right_dst, r_map1, r_map2, cv::INTER_LINEAR);
    }

    void getQ(cv::Mat& q) {
        q = Q;
    }
};

int main(int argc, char* argv[]) {
    // Post OpenCV build info
    ROS_INFO(cv::getBuildInformation().c_str());

    // Setup ROS node and publishers
    ros::init(argc, argv, "stereoSGBM_mapping");
    ros::NodeHandle nh;
    ros::Publisher rgb_pub = nh.advertise<sensor_msgs::Image>("rgb/image", 1000);               // RGB image of left/image_rect
    ros::Publisher meta_pub = nh.advertise<sensor_msgs::CameraInfo>("rgb/camera_info", 1000);   // RGB camera metadata
    ros::Publisher depth_pub = nh.advertise<sensor_msgs::Image>("depth/image", 1000);           // Depth image

    // Get Camera setup info from node parameters
    int width, height;
    int fps;
    std::string xml_calib;
    nh.getParam("capture_width", width);
    nh.getParam("capture_height", height);
    nh.getParam("capture_fps", fps);
    nh.getParam("calibration_file", xml_calib);
    cv::Size size(width, height);

    // camera feed setup
    std::string pipeline = gstreamerPipelineStr(size.width, size.height, fps, 0);
    ROS_INFO("// Left Cam GStreamer Pipeline //");
    ROS_INFO(pipeline.c_str());
    cv::VideoCapture left_cam(pipeline, cv::CAP_GSTREAMER);
    pipeline = gstreamerPipelineStr(size.width, size.height, fps, 1);
    ROS_INFO("// Right Cam GStreamer Pipeline //");
    ROS_INFO(pipeline.c_str());
    cv::VideoCapture right_cam(pipeline, cv::CAP_GSTREAMER);

    if (!left_cam.isOpened()) {
        ROS_INFO("Couldn't open left camera");
        return -1;
    }
    if (!right_cam.isOpened()) {
        ROS_INFO("Couldn't open right camera");
        return -1;
    }

    // Get stereo calibration data
    StereoCalibration calibrate(xml_calib);
    cv::Mat left_frame, right_frame;

    // Sample Image
    bool left_success = left_cam.read(left_frame);
    bool right_success = right_cam.read(right_frame);
    if (!left_success) {
        ROS_INFO("Left Camera Disconnected - Cannot read frame");
        return -1;
    }
    if (!right_success) {
        ROS_INFO("Right Camera Disconnected - Cannot read frame");
        return -1;
    }

    // Disparity Algorithm Setup
    int block_size = 11;
    int min_disp = 0;
    int max_disp = 64;
    int num_disp = max_disp - min_disp;
    int cn = left_frame.channels();
    
    cv::Ptr<cv::StereoSGBM> stereo = cv::StereoSGBM::create(min_disp, num_disp, block_size);
    stereo->setPreFilterCap(63);
    stereo->setP1(8 * cn * block_size * block_size);
    stereo->setP2(32 * cn * block_size * block_size);
    stereo->setUniquenessRatio(10);
    stereo->setSpeckleWindowSize(100);
    stereo->setSpeckleRange(32);
    stereo->setDisp12MaxDiff(1);
    stereo->setMode(cv::StereoSGBM::MODE_SGBM);
    
    cv::Mat left_gray, right_gray;
    cv::Mat disparity, depth;
    cv::Mat Q;
    calibrate.getQ(Q);
    uint32_t seq;

    while (ros::ok()) {
        left_success = left_cam.read(left_frame);
        right_success = right_cam.read(right_frame);
        if (!left_success) {
            ROS_INFO("Left Camera Disconnected - Cannot read frame");
            return -1;
        }
        if (!right_success) {
            ROS_INFO("Right Camera Disconnected - Cannot read frame");
            return -1;
        }
        
        calibrate.applyCalibration(left_frame, right_frame, left_frame, right_frame);
        cv::cvtColor(left_frame, left_gray, cv::COLOR_BGR2GRAY);
        cv::cvtColor(right_frame, right_gray, cv::COLOR_BGR2GRAY);

        // calculate disparity with StereoSGBM
        stereo->compute(left_gray, right_gray, disparity);
        cv::reprojectImageTo3D(disparity, depth, Q, true); // project disparity map to 3D image

        publishStereoMappingData(depth, left_frame, seq, size, rgb_pub, meta_pub, depth_pub);

        seq++;
    }

    return 0;
}