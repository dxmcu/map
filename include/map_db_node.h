/*************************************************************************
  > File Name: mapping_load_icp.cpp
  > Author: YHY
  > Mail: hyyezju@gmail.com
  > Created Time: Friday, Feb 27, 2017 14:44:00 HKT
  > 12.23修改
 ************************************************************************/

#ifndef MAP_MAP_DB_NODE_H
#define MAP_MAP_DB_NODE_H

#include <iostream>
#include <fstream>
#include <string>
#include <limits>

#include <geometry_msgs/Quaternion.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseStamped.h>

#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

#include <tf_conversions/tf_eigen.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <unsupported/Eigen/MatrixFunctions>

#include <cmath>
#include <time.h>

#include "pointmatcher/PointMatcher.h"
#include "point_cloud.h"

#include "yaml-cpp/yaml.h"

#include "map/LocalMapRetrieve"
#include "parameters.h"
#include "tic_toc.h"
#include "laserutils.h"

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

using namespace std;
using namespace Eigen;

typedef PointMatcher<float> PM;
typedef PM::DataPoints DP;

static ros::Publisher pubPointCloud;
static ros::Publisher pubRegistedPtc;
static ros::Publisher pubLocalMap;
static ros::Publisher pubTmp;
static ros::Publisher pubOctomap;
static ros::Publisher pubLaserOdometry;

static DP refFrame;
static DP srcFrame;

static PM::TransformationParameters T_srcToRef = PM::TransformationParameters::Identity(4,4);
static PM::TransformationParameters T_refToLocal = PM::TransformationParameters::Identity(4,4);
static PM::TransformationParameters T_init = PM::TransformationParameters::Identity(4,4);
static PM::TransformationParameters T_srcToLocal = PM::TransformationParameters::Identity(4,4);
static PM::TransformationParameters T_srcToGlobal = PM::TransformationParameters::Identity(4,4);
static PM::TransformationParameters T_localToGlobal = PM::TransformationParameters::Identity(4,4);
static PM::DataPointsFilters *DPFilterC, *DPFilterBox; // DPFilter for fast computation
static PM::DataPointsFilters *DPFilter;
static int frame_num = 0;

// TFBroadcaster -- frame
static tf::TransformBroadcaster *brPointer = NULL;
static tf::Transform captureFrame_trans;
static tf::Transform localMap_trans;

static tf::Transform q_iw_trans;
static tf::Transform q_iwTmp_trans;

// Odometry
static nav_msgs::Odometry laserOdometry;

static Eigen::Vector3f Txyz(0, 0, 0), TxyzLast(0, 0, 0);

static int start_num = 0, start_end_count = 0, end_num = std::numeric_limits<int>::max();

// Localization
static Eigen::Vector3f translation;
static Eigen::Quaternionf rotation;

static TicToc tic_toc;

static laserutils::LaserFilters laser_filters;

DP globalMap;

PM::TransformationParameters transformationInverse(const PM::TransformationParameters inTransformation)
{
    PM::TransformationParameters outTransformation = PM::TransformationParameters::Identity(4,4);
    outTransformation.block(0, 0, 3, 3) = (inTransformation.block(0, 0, 3, 3)).transpose().eval();
    outTransformation.block(0, 3, 3, 1) = - (inTransformation.block(0, 0, 3, 3)).transpose().eval() * inTransformation.block(0, 3, 3, 1);
    return outTransformation;
}

bool retrieveCallback(ugv_localizer::LocalMapRetrieve::Request &req,
                      ugv_localizer::LocalMapRetrieve::Response &res)
{
    ROS_INFO_STREAM("wait for retrieval...");
    PM::Transformation* rigidTrans;
    rigidTrans = PM::get().REG(Transformation).create("RigidTransformation");

    geometry_msgs::PoseStamped mapLoc = req.poseWithStamp;

    float x_min, x_max, y_min, y_max, z_min, z_max;
    Txyz.x() = mapLoc.pose.position.x;
    Txyz.y() = mapLoc.pose.position.y;
    Txyz.z() = mapLoc.pose.position.z;
    x_min = Txyz.x() - 50;
    x_max = Txyz.x() + 50;
    y_min = Txyz.y() - 50;
    y_max = Txyz.y() + 50;
    z_min = Txyz.z() - 25;
    z_max = Txyz.z() + 25;

    PointMatcherSupport::Parametrizable::Parameters params;
    std::string name = "BoundingBoxDataPointsFilter";
    params["xMin"] = std::to_string(x_min); params["xMax"] = std::to_string(x_max);
    params["yMin"] = std::to_string(y_min); params["yMax"] = std::to_string(y_max);
    params["zMin"] = std::to_string(z_min); params["zMax"] = std::to_string(z_max);
    params["removeInside"] = "0";
    PM::DataPointsFilter* globalBoxFilter =
            PM::get().DataPointsFilterRegistrar.create(name, params);
    params.clear();

    name = "BoundingBoxDataPointsFilter";
    params["xMin"] = "-50"; params["xMax"] = "50";
    params["yMin"] = "-50"; params["yMax"] = "50";
    params["zMin"] = "-25"; params["zMax"] = "25";
    params["removeInside"] = "0";
    PM::DataPointsFilter* localBoxFilter =
            PM::get().DataPointsFilterRegistrar.create(name, params);
    params.clear();

    // rotate the cropped local related to current frame
    DP localMap = globalBoxFilter->filter(globalMap); // T_srcToGlobal Boundary
    ROS_INFO_STREAM("localMap points: " << localMap.getNbPoints());

    Eigen::Quaternionf q_localToGlobal;
    q_localToGlobal.w() = mapLoc.pose.orientation.w; q_localToGlobal.x() = mapLoc.pose.orientation.x;
    q_localToGlobal.y() = mapLoc.pose.orientation.y; q_localToGlobal.z() = mapLoc.pose.orientation.z;
    T_localToGlobal.block(0, 0, 3, 3) = q_localToGlobal.toRotationMatrix();
    T_localToGlobal.block(0, 3, 3, 1) = Txyz;

    if (!rigidTrans->checkParameters(T_localToGlobal))
    {
        std::cout << "WARNING: T_localToGlobal does not represent a valid rigid transformation\nProjecting onto an orthogonal basis" << std::endl;
        T_localToGlobal = rigidTrans->correctParameters(T_localToGlobal);
    }

    PM::TransformationParameters T_gToNL = PM::TransformationParameters::Identity(4,4); // global to new local
    T_gToNL = transformationInverse(T_localToGlobal);
    if (!rigidTrans->checkParameters(T_gToNL))
    {
        std::cout << "WARNING: T_gToNL does not represent a valid rigid transformation\nProjecting onto an orthogonal basis" << std::endl;
        T_gToNL = rigidTrans->correctParameters(T_gToNL);
    }
    localMap = rigidTrans->compute(localMap, T_gToNL);
    localMap = localBoxFilter->filter(localMap); // history local map

    // Downsampling local points
    double points_ratio = 100000.0/localMap.getNbPoints() > 1?1.0:100000.0/localMap.getNbPoints();

    name = "SamplingSurfaceNormalDataPointsFilter";
    params["ratio"] = std::to_string(points_ratio);
    PM::DataPointsFilter* localSurfaceNormalFilter = PM::get().DataPointsFilterRegistrar.create(name, params);
    params.clear();
    localMap = localSurfaceNormalFilter->filter(localMap);

    ROS_INFO_STREAM("localMap points: " << localMap.getNbPoints());

    sensor_msgs::PointCloud2 registerdLocalMap;
    registerdLocalMap = PointMatcher_ros::pointMatcherCloudToRosMsg<float>(localMap, "/local_frame", mapLoc.header.stamp);
    pubLocalMap.publish(registerdLocalMap);

    res.pointcloud = registerdLocalMap;

    ROS_INFO_STREAM("res.pointcloud " << res.pointcloud.width);

//    ROS_WARN_STREAM("x_min " << x_min << " x_max " << x_max << " y_min " << y_min << " y_max " << y_max << " z_min " << z_min << " z_max " << z_max);
//    ROS_INFO_STREAM("T_localToGlobal: " << std::setprecision (7) << T_localToGlobal(0, 3) << "," << T_localToGlobal(1, 3) << "," << T_localToGlobal(2, 3));
//    ROS_INFO_STREAM("T_srcToLocal: " << std::setprecision (7) << T_srcToLocal(0, 3) << "," << T_srcToLocal(1, 3) << "," << T_srcToLocal(2, 3));

    return true;
}


int main(int argc, char** argv)
{
    DP global_show;

    // ros init
    ros::init(argc, argv, "map_db_node");
    ros::NodeHandle nh("~");

    std::string config_file;
    nh.getParam("config_file", config_file);

    ROS_INFO_STREAM("map_db_node");

    ReadParameters(config_file);

    ros::ServiceServer service = nh.advertiseService("/local_map_retrieve", retrieveCallback);

    pubLocalMap = nh.advertise<sensor_msgs::PointCloud2>
        ("/retrived_map", 1);

    pubPointCloud = nh.advertise<sensor_msgs::PointCloud2>
        ("/rslidar_globalMap", 1);

    tf::TransformBroadcaster br;
    brPointer = &br;

    std::string pcd_name = OUTPUT_MAP_NAME;

    if (argc < 1)
    {
        ROS_ERROR_STREAM("arg[1] = config.yaml");
        return -2;
    }

    pcl::PCDReader pcd_reader;
    pcl::PCLPointCloud2 input_pcl;
    sensor_msgs::PointCloud2 pc_in_msg;

    ROS_WARN_STREAM("pcd name: " << pcd_name);
    std::ifstream infileSwitch(pcd_name);
    if (infileSwitch.good())
    {
        tic_toc.Tic();
        pcd_reader.read(pcd_name, input_pcl);
        pcl_conversions::fromPCL(input_pcl, pc_in_msg);
        globalMap = PointMatcher_ros::rosMsgToPointMatcherCloud<float>(pc_in_msg);

        ROS_INFO_STREAM(">>>>>>> Map loading time: " << tic_toc.Toc()/1000.0 << " s <<<<<<<");
        // Downsampling global points
//        string name = "SamplingSurfaceNormalDataPointsFilter";
//        PointMatcherSupport::Parametrizable::Parameters params;
//        params["ratio"] = "0.1";

        tic_toc.Tic();

        string name = "RandomSamplingDataPointsFilter";
        PointMatcherSupport::Parametrizable::Parameters params;
        params["prob"] = "0.1";
        PM::DataPointsFilter* globalSurfaceNormalFilter = PM::get().DataPointsFilterRegistrar.create(name, params);
        params.clear();
        globalSurfaceNormalFilter->inPlaceFilter(globalMap);

        ROS_INFO_STREAM(">>>>>>> Sampling time: " << tic_toc.Toc()/1000.0 << " s <<<<<<<");

    }
    else
    {
        ROS_ERROR_STREAM("MAP FILE ERROR");
        return -1;
    }

    ROS_INFO_STREAM("File loaded.");
    ROS_INFO_STREAM("globalMap points: " << globalMap.getNbPoints());

    tic_toc.Tic();
    sensor_msgs::PointCloud2 g_map_msg;
    g_map_msg = PointMatcher_ros::pointMatcherCloudToRosMsg<float>(globalMap, "/global_frame", ros::Time::now());
    pubPointCloud.publish(g_map_msg);
    ROS_INFO_STREAM(">>>>>>> Convert and publish time: " << tic_toc.Toc()/1000.0 << " s <<<<<<<");

    ros::spin();

    return 0;

}
#endif //MAP_MAP_DB_NODE_H
