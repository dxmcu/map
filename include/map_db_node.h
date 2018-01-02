/*************************************************************************
  > File Name: mapping_load_icp.cpp
  > Author: YHY
  > Mail: hyyezju@gmail.com
  > Created Time: Friday, Feb 27, 2017 14:44:00 HKT
  > 17.12.23 修改整合
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
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <tf_conversions/tf_eigen.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <unsupported/Eigen/MatrixFunctions>

#include <cmath>
#include <time.h>

#include "pointmatcher/PointMatcher.h"
#include "point_cloud.h"

#include "yaml-cpp/yaml.h"

//#include "map_core/LocalMapRetrieve.h"
//#include "parameters.h"
//#include "laserutils.h"

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>


class MapDbNode
{
private:
    PointMatcher<float>::TransformationParameters T_localToGlobal = PointMatcher<float>::TransformationParameters::Identity(4,4);
    Eigen::Vector3f Txyz = Eigen::Vector3f(0, 0, 0);

    PointMatcher<float>::DataPoints globalMap_;// = PointMatcher<float>::DataPoints();     //全局地图点云

    PointMatcher<float>::TransformationParameters transformationInverse(const PointMatcher<float>::TransformationParameters inTransformation)
    {
        PointMatcher<float>::TransformationParameters outTransformation = PointMatcher<float>::TransformationParameters::Identity(4,4);
        outTransformation.block(0, 0, 3, 3) = (inTransformation.block(0, 0, 3, 3)).transpose().eval();
        outTransformation.block(0, 3, 3, 1) = - (inTransformation.block(0, 0, 3, 3)).transpose().eval() * inTransformation.block(0, 3, 3, 1);
        return outTransformation;
    }

public:

    //double
    void transform(const Eigen::Matrix4d & trans)
    {
        Eigen::Matrix4f T;
        for (int i = 0; i < 3; ++ i)
            for (int j = 0; j < 3; ++ j)
                T(i,j) = trans(i, j);
        transform(T);
    }

    //double
    void invTransform(const Eigen::Matrix4d & trans)
    {
        Eigen::Matrix4f T;
        for (int i = 0; i < 3; ++ i)
            for (int j = 0; j < 3; ++ j)
                T(i,j) = trans(i, j);
        invTransform(T);
    }

    //float
    void transform(const Eigen::Matrix4f & trans)
    {
        PointMatcher<float>::ICP icp;
        icp.setDefault();
        icp.transformations.apply(globalMap_, trans);
    }

    //float
    void invTransform(const Eigen::Matrix4f & trans)
    {
        PointMatcher<float>::ICP icp;
        icp.setDefault();
        icp.transformations.apply(globalMap_, trans.inverse());
    }

    void loadGlobalMapFromPCD(const std::string & pcd_name)  //从pcd文件获取global map
    {
        ROS_INFO_STREAM("pcd name: " << pcd_name);
        std::ifstream infileSwitch(pcd_name);
        static pcl::PCDReader pcd_reader;      //读入一个pcd格式的数据
        static pcl::PCLPointCloud2 input_pcl;  //保存pcl格式
        static sensor_msgs::PointCloud2 pc_in_msg; //要发布的数据 好像没啥用
        //预处理全局地图
        if (infileSwitch.good())
        {
            pcd_reader.read(pcd_name, input_pcl);   //从pcd读入地图,input_pcl保存的是读入的pcl地图
            pcl_conversions::fromPCL(input_pcl, pc_in_msg); //pc_in_msg是ros的cloud2类型
            globalMap_ = PointMatcher_ros::rosMsgToPointMatcherCloud<float>(pc_in_msg);    //然后再转换为pointmatcher类型，保存为globalmap
            // Downsampling global points
            std::string name = "RandomSamplingDataPointsFilter";
            PointMatcherSupport::Parametrizable::Parameters params; //要从地图中滤出一些区域
            params["prob"] = "0.1";                                 //设置参数
            PointMatcher<float>::DataPointsFilter* globalSurfaceNormalFilter = PointMatcher<float>::get().DataPointsFilterRegistrar.create(name, params);
            //global
            params.clear();
            globalSurfaceNormalFilter->inPlaceFilter(globalMap_);   //对globalmap做简单的预处理
            delete globalSurfaceNormalFilter;
        }
        else
        {
            ROS_ERROR_STREAM("MAP FILE ERROR");
            exit(0);
        }
        infileSwitch.close();
    }


    void pos2cloud(const geometry_msgs::PoseWithCovarianceStamped &pose, sensor_msgs::PointCloud2 &cloud)
    {
        geometry_msgs::PoseStamped mapLoc;
        mapLoc.header = pose.header;
        mapLoc.pose = pose.pose.pose;
        retrieve(mapLoc, cloud);
    }

    bool retrieve(const geometry_msgs::PoseStamped &mapLoc,
                                 sensor_msgs::PointCloud2 &res)
    {
#ifdef DEBUG
        ROS_INFO_STREAM("wait for retrieval...");
#endif
        PointMatcher<float>::Transformation* rigidTrans;
        rigidTrans = PointMatcher<float>::get().REG(Transformation).create("RigidTransformation");

        //geometry_msgs::PoseStamped mapLoc = req.poseWithStamp;

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
        PointMatcher<float>::DataPointsFilter* globalBoxFilter =
                PointMatcher<float>::get().DataPointsFilterRegistrar.create(name, params);
        params.clear();

        name = "BoundingBoxDataPointsFilter";
        params["xMin"] = "-50"; params["xMax"] = "50";
        params["yMin"] = "-50"; params["yMax"] = "50";
        params["zMin"] = "-25"; params["zMax"] = "25";
        params["removeInside"] = "0";
        PointMatcher<float>::DataPointsFilter* localBoxFilter =
                PointMatcher<float>::get().DataPointsFilterRegistrar.create(name, params);
        params.clear();

        // rotate the cropped local related to current frame
        PointMatcher<float>::DataPoints localMap = globalBoxFilter->filter(globalMap_); // T_srcToGlobal Boundary
#ifdef DEBUG
        ROS_INFO_STREAM("localMap points: " << localMap.getNbPoints());
#endif

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

        PointMatcher<float>::TransformationParameters T_gToNL = PointMatcher<float>::TransformationParameters::Identity(4,4); // global to new local
        T_gToNL = transformationInverse(T_localToGlobal);
        if (!rigidTrans->checkParameters(T_gToNL))
        {
            std::cout << "WARNING: T_gToNL does not represent a valid rigid transformation\nProjecting onto an orthogonal basis" << std::endl;
            T_gToNL = rigidTrans->correctParameters(T_gToNL);
        }
        localMap = rigidTrans->compute(localMap, T_gToNL);
        localMap = localBoxFilter->filter(localMap); // history local map

        // Downsampling local points
        float points_ratio = 100000.0/localMap.getNbPoints() > 1?1.0:100000.0/localMap.getNbPoints();

        name = "SamplingSurfaceNormalDataPointsFilter";
        params["ratio"] = std::to_string(points_ratio);
        PointMatcher<float>::DataPointsFilter* localSurfaceNormalFilter = PointMatcher<float>::get().DataPointsFilterRegistrar.create(name, params);
        params.clear();
        localMap = localSurfaceNormalFilter->filter(localMap);

#ifdef DEBUG
        ROS_INFO_STREAM("localMap points: " << localMap.getNbPoints());
#endif

        sensor_msgs::PointCloud2 registerdLocalMap;
        registerdLocalMap = PointMatcher_ros::pointMatcherCloudToRosMsg<float>(localMap, "/local_frame", mapLoc.header.stamp);
        //pubLocalMap.publish(registerdLocalMap);

        res = registerdLocalMap;

#ifdef DEBUG
        ROS_INFO_STREAM("res.pointcloud " << registerdLocalMap.width);
#endif

        return true;
    }
};

#endif
