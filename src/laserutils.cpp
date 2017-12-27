#include "laserutils.h"

namespace laserutils {

typedef PointMatcher<float> PM;
typedef PM::DataPoints DP;

LaserFilters::LaserFilters()
{
    name = "BoundingBoxDataPointsFilter";
    params["xMin"] = "-50"; params["xMax"] = "50";
    params["yMin"] = "-50"; params["yMax"] = "50";
    params["zMin"] = "-25"; params["zMax"] = "25";
    params["removeInside"] = "0";
    localBoxFilter =
            PM::get().DataPointsFilterRegistrar.create(name, params);
    params.clear();

    name = "BoundingBoxDataPointsFilter";
    params["xMin"] = "0"; params["xMax"] = "75";
    params["yMin"] = "-50"; params["yMax"] = "50";
    params["zMin"] = "-25"; params["zMax"] = "25";
    params["removeInside"] = "0";
    updatedLocalBoxFilter =
            PM::get().DataPointsFilterRegistrar.create(name, params);
    params.clear();

    name = "RandomSamplingDataPointsFilter";
    params["prob"] = "0.8";
    randomFilterMore =
            PM::get().DataPointsFilterRegistrar.create(name, params);
    params.clear();

    name = "RandomSamplingDataPointsFilter";
    params["prob"] = "0.4";
    randomFilter =
            PM::get().DataPointsFilterRegistrar.create(name, params);
    params.clear();

    name = "RandomSamplingDataPointsFilter";
    params["prob"] = "0.2";
    randomFilterLess =
            PM::get().DataPointsFilterRegistrar.create(name, params);
    params.clear();

    name = "SamplingSurfaceNormalDataPointsFilter";
    params["ratio"] = "0.5";
    samplingSurfaceNormalFilter =
            PM::get().DataPointsFilterRegistrar.create(name, params);
    params.clear();

    name = "SamplingSurfaceNormalDataPointsFilter";
    params["ratio"] = "0.8";
    samplingSurfaceNormalFilterMore =
            PM::get().DataPointsFilterRegistrar.create(name, params);
    params.clear();

    name = "SamplingSurfaceNormalDataPointsFilter";
    params["ratio"] = "0.5";
    surfacenormal_filter =
            PM::get().DataPointsFilterRegistrar.create(name, params);
    params.clear();
}

void LaserFilters::SetSurfaceNormalRatio(double ratio) {
    if (ratio <= 0.0 || ratio > 1.0) {
        params["ratio"] = "1.0";
        surfacenormal_filter =
                PM::get().DataPointsFilterRegistrar.create(name, params);
        params.clear();
        ROS_WARN("ratio should be in (0.0, 1.0] @ SetSurfaceNormalRatio");
    }
    else {
        params["ratio"] = std::to_string(ratio);
        surfacenormal_filter =
                PM::get().DataPointsFilterRegistrar.create(name, params);
        params.clear();
    }
}

LaserTrans::LaserTrans() {
    // initial
    nh_ = ros::NodeHandle("lasertrans");
    laser_scan_pub_ = nh_.advertise<sensor_msgs::PointCloud2>( "current_scan", 1);
    laser_local_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/localMap", 2);
    laser_global_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/globalMap", 1);
}

void LaserTrans::AddLaserPub(std::string topic_name, unsigned int buff_num)
{
    if (pub_maps_.find(topic_name) != pub_maps_.end())
    {
        // topic name not find
        ROS_INFO_STREAM("Topic " << topic_name << " has added before.");
        return;
    }
    auto newPubPtr = std::make_shared<ros::Publisher>(nh_.advertise<sensor_msgs::PointCloud2>(topic_name, buff_num));
    pub_maps_.insert(std::pair<std::string, std::shared_ptr<ros::Publisher> >(topic_name, newPubPtr));
}

PM::Transformation* LaserTrans::rigidTrans = PM::get().REG(Transformation).create("RigidTransformation");

void LaserTrans::CorrectTransformation(PM::TransformationParameters &T)
{
    if (!rigidTrans->checkParameters(T))
    {
        std::cout << "WARNING: T does not represent a valid rigid transformation\nProjecting onto an orthogonal basis" << std::endl;
        T = rigidTrans->correctParameters(T);
    };
}

PM::TransformationParameters LaserTrans::InverseTransformation(const PM::TransformationParameters inTransformation)
{
    PM::TransformationParameters outTransformation = PM::TransformationParameters::Identity(4,4);
    outTransformation.block(0, 0, 3, 3) = (inTransformation.block(0, 0, 3, 3)).transpose().eval();
    outTransformation.block(0, 3, 3, 1) = - (inTransformation.block(0, 0, 3, 3)).transpose().eval() * inTransformation.block(0, 3, 3, 1);
    CorrectTransformation(outTransformation);
    return outTransformation;
}

void LaserTrans::PubLaserAndBr(const sensor_msgs::PointCloud2 &pointCloudMsg, const Eigen::Affine3d & trans_pose, std::string topic_name, std::string parent_id, std::string frame_id)
{
    if (pub_maps_.find(topic_name) == pub_maps_.end())
    {
        // topic name not find
        ROS_ERROR_STREAM("Topic " << topic_name << " not initialized, use addLaserPub to add one.");
        return;
    }

    double timeCurr = pointCloudMsg.header.stamp.toSec();

    tf::Transform transCurr;

    tf::Quaternion quat_tmp;
    tf::Vector3 vec_tmp;

    tf::quaternionEigenToTF(Eigen::Quaterniond(trans_pose.rotation()), quat_tmp);
    tf::vectorEigenToTF(trans_pose.translation(), vec_tmp);

    transCurr.setRotation(quat_tmp);
    transCurr.setOrigin(vec_tmp);

    br_.sendTransform(tf::StampedTransform(transCurr, ros::Time(timeCurr), parent_id, frame_id));

    pub_maps_[topic_name]->publish(pointCloudMsg);
}

void LaserTrans::PubLaserScan(const sensor_msgs::PointCloud2 &pointCloudMsg, std::string topic_name, std::string frame_id)
{
    if (pub_maps_.find(topic_name) == pub_maps_.end())
    {
        // topic name not find
        ROS_ERROR_STREAM("Topic " << topic_name << " not initialized, use addLaserPub to add one.");
        return;
    }

    sensor_msgs::PointCloud2 tmp_Msg = pointCloudMsg;
    tmp_Msg.header.frame_id = frame_id;

    pub_maps_[topic_name]->publish(tmp_Msg);
}

} // laserutils
