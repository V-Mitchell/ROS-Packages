/*  Author: Victor M
*   Email: victorcmitchell@gmail.com
*   Notes:  - Reads data from icm20948 IMU, then publishes to ros topic
*/

#include <chrono>
#include <thread>

#include <ros/ros.h>
#include <ros/time.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <geometry_msgs/TwistWithCovariance.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

extern "C" {
    #include "../include/ICM20948/ICM20948.h"
}

void calculateOdometry(geometry_msgs::PoseWithCovariance& pose, geometry_msgs::TwistWithCovariance& twist,
    IMU_ST_ANGLES_DATA *pstAngles, IMU_ST_SENSOR_DATA *pstGyroRawData,
    IMU_ST_SENSOR_DATA *pstAccelRawData, IMU_ST_SENSOR_DATA *pstMagnRawData) {
    
    tf2::Quaternion q;
    q.setRPY(pstAngles->fRoll, pstAngles->fPitch, pstAngles->fYaw);
    q.normalize();
    pose.pose.orientation = tf2::toMsg(q);
    // Accelerometer data is insufficient for position estimation

    // Twist msg encodes linear and angular velocity

}

int main (int argc, char* argv[]) {
    ros::init(argc, argv, "icm20948_odometry");
    ros::NodeHandle nh;

    ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 1000);

    IMU_EN_SENSOR_TYPE enMotionSensorType;
	IMU_ST_ANGLES_DATA stAngles;
	IMU_ST_SENSOR_DATA stGyroRawData;
	IMU_ST_SENSOR_DATA stAccelRawData;
	IMU_ST_SENSOR_DATA stMagnRawData;

    imuInit(&enMotionSensorType);
	if(IMU_EN_SENSOR_TYPE_ICM20948 == enMotionSensorType) {
        ROS_INFO("Motion sensor is ICM-20948");
	} else {
        ROS_INFO("Motion sersor NULL");
        return 0;
	}

    uint32_t seq_id = 0;
    geometry_msgs::PoseWithCovariance pose;
    geometry_msgs::TwistWithCovariance twist;
	while(ros::ok()) {
        
		imuDataGet( &stAngles, &stGyroRawData, &stAccelRawData, &stMagnRawData);
        calculateOdometry(pose, twist, &stAngles, &stGyroRawData, &stAccelRawData, &stMagnRawData);

        nav_msgs::Odometry msg;
        msg.header.seq = seq_id;
        msg.header.stamp = ros::Time::now();
        msg.child_frame_id = "camera_frame";
        msg.pose = pose;
        msg.twist = twist;
        odom_pub.publish(msg);

        seq_id++;
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
	}

    return 0;
}