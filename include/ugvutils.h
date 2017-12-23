#ifndef UGVUTILS_H_
#define UGVUTILS_H_

#include <iostream>
#include <ros/ros.h>

#include <nav_msgs/Odometry.h>

#include <tf_conversions/tf_eigen.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace ugvutils
{

struct ConvertUtils
{

    template<typename T>
    static void EigenTransformToOdom(const Eigen::Transform<T, 3, Eigen::TransformTraits::Affine> &trans, nav_msgs::Odometry &odom) {
    //    Eigen::Transform<T, 3, Eigen::TransformTraits::Affine> trans;
    //    trans.matrix() = trans_mat;
        Eigen::Quaternion<T> quat(trans.linear());
        Eigen::Matrix<T, 3, 1> pos = trans.translation();
        odom.pose.pose.orientation.x = quat.x();
        odom.pose.pose.orientation.y = quat.y();
        odom.pose.pose.orientation.z = quat.z();
        odom.pose.pose.orientation.w = quat.w();
        odom.pose.pose.position.x = pos.x();
        odom.pose.pose.position.y = pos.y();
        odom.pose.pose.position.z = pos.z();
    }

    //template<> void TransformToOdom(const Eigen::Transform<float, 3, Eigen::TransformTraits::Affine> &trans, nav_msgs::Odometry &odom);

    template<typename T>
    static void PoseToEigenTransform(const geometry_msgs::PoseWithCovariance &pose, Eigen::Transform<T, 3, Eigen::TransformTraits::Affine> &trans)
    {
        Eigen::Quaternion<T> quat;
        Eigen::Matrix<T, 3, 1> pos;
        quat.x() = pose.pose.orientation.x;
        quat.y() = pose.pose.orientation.y;
        quat.z() = pose.pose.orientation.z;
        quat.w() = pose.pose.orientation.w;
        pos.x() = pose.pose.position.x;
        pos.y() = pose.pose.position.y;
        pos.z() = pose.pose.position.z;

        trans.linear() = Eigen::Matrix<T, 3, 3>(quat);
        trans.translation() = pos;
    }

    template<typename T>
    static void EigenTransformToPose(const Eigen::Transform<T, 3, Eigen::TransformTraits::Affine> &trans, geometry_msgs::PoseStamped &pose)
    {
        Eigen::Quaternion<T> quat(trans.linear());
        Eigen::Matrix<T, 3, 1> pos = trans.translation();
        pose.pose.orientation.x = quat.x();
        pose.pose.orientation.y = quat.y();
        pose.pose.orientation.z = quat.z();
        pose.pose.orientation.w = quat.w();
        pose.pose.position.x = pos.x();
        pose.pose.position.y = pos.y();
        pose.pose.position.z = pos.z();
    }

    template<typename T>
    static void TransformEigenToTF(const Eigen::Transform<T, 3, Eigen::TransformTraits::Affine> &trans, tf::Transform &tf_trans)
    {
        Eigen::Affine3d trans_d = trans.template cast<double>();
        tf::transformEigenToTF(trans_d, tf_trans);
    }

    template<typename T>
    static void TransformEigenToTF(const Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> &trans_mat, tf::Transform &tf_trans)
    {
        if (trans_mat.rows() != 4 || trans_mat.cols() != 4)
        {
            ROS_WARN("trans_mat should be 4x4 @ TransformEigenToT");
            return;
        }
        Eigen::Affine3d trans_d;
        trans_d.matrix() = trans_mat.template cast<double>();
        tf::transformEigenToTF(trans_d, tf_trans);
    }


}; // convertutils

struct MathUtils
{

    template<typename T>
    static Eigen::Transform<T, 3, Eigen::TransformTraits::Affine>
    TransformationInverse(const Eigen::Transform<T, 3, Eigen::TransformTraits::Affine>& trans_in) {
        Eigen::Transform<T, 3, Eigen::TransformTraits::Affine> trans_out = Eigen::Transform<T, 3, Eigen::TransformTraits::Affine>::Identity();
        trans_out.linear() = Eigen::Matrix<T, 3, 3>((trans_in.linear()).transpose().eval() );
        trans_out.translation() = - (trans_in.linear()).transpose().eval() * trans_in.translation();
        return trans_out;
    }

};

} // ugvutils

#endif // UGVUTILS_H_
