//
// Created by bohuan on 18-1-2.
//

#ifndef MAP_CORE_EDGEINFO_H
#define MAP_CORE_EDGEINFO_H

#include "NodeInfo.h"

class EdgeInfo
{
private:
    int edge_id_;           //这条边的编号
    Eigen::Matrix4d transformational_matrix_; //= junk_map::returnEigen();    //to到from的变换矩阵
    NodeInfo* from_point_cloud_ptr_;        //保存from指向的node
    NodeInfo* to_point_cloud_ptr_;
    bool is_connected_;                     //是否连接。
    std::vector< std::vector<cv::Point2f> > contours_;  //两个node的公共区域（在from身上画出,5个点，最后一个点和第一个点相同的一个矩形） 公共区域不止一个区域


public:

    std::vector< std::vector<cv::Point2f> > contours()  const;

    EdgeInfo();

    Eigen::Matrix4d transformational_matrix()  const;

    EdgeInfo(NodeInfo * from, NodeInfo * to, int edge_id, const  Eigen::Matrix4d & trans_mat);

    int from()  const ;
    int to()    const ;
    bool is_connceted()  const;

    int cost();      //可以改写,表示这个边的权重。

    Json::Value json();

    void loadFromJson(Json::Value root, std::vector< NodeInfo > &nodes_info);
};


#endif //MAP_CORE_EDGEINFO_H
