#ifndef LASERUTILS_H_
#define LASERUTILS_H_

#include "pointmatcher/PointMatcher.h"
#include "point_cloud.h"

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace laserutils {

typedef PointMatcher<float> PM;
typedef PM::DataPoints DP;

class LaserFilters {
public:
    LaserFilters();

    PM::DataPointsFilter *localBoxFilter, *updatedLocalBoxFilter, *randomFilterMore, *surfacenormal_filter,
        *randomFilter, *randomFilterLess, *samplingSurfaceNormalFilter, *samplingSurfaceNormalFilterMore;

    void SetSurfaceNormalRatio(double ratio);

private:
    PointMatcherSupport::Parametrizable::Parameters params;
    std::string name;

};


class LaserTrans {
public:
    LaserTrans();

    ros::Publisher laser_scan_pub_, laser_local_pub_, laser_global_pub_;

    std::map<std::string, std::shared_ptr<ros::Publisher> > pub_maps_;

    tf::TransformBroadcaster br_;

    void AddLaserPub(std::string topic_name, unsigned int buff_num = 1);

    void PubLaserAndBr(const sensor_msgs::PointCloud2 &pointCloudMsg, const Eigen::Affine3d & trans_pose, std::string topic_name, std::string parent_id="/world", std::string frame_id = "/local_frame");

    void PubLaserScan(const sensor_msgs::PointCloud2 &pointCloudMsg, std::string topic_name, std::string frame_id = "/local_frame");

    static void CorrectTransformation(PM::TransformationParameters &T);

    static PM::TransformationParameters InverseTransformation(const PM::TransformationParameters inTransformation);

    static PM::Transformation* rigidTrans;

private:
    ros::NodeHandle nh_;
};

} // laserutils

#endif // LASERUTILS_H_
