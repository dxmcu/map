//
// Created by bohuan on 17-12-20.
//

#ifndef MAP_COMMON_H
#define MAP_COMMON_H

#include <bits/stdc++.h>
#include <json/json.h>
#include <opencv2/opencv.hpp>
#include <geometry_msgs/Point.h>
#include <eigen3/Eigen/Core>

#define pr(x)   std::cout<<#x<<"="<<x<<","
#define prln(x) std::cout<<#x<<"="<<x<<std::endl

//#define DIE
#define CHECK_EQ(x,y,z) if(!(x==y)) pr(x), pr(y)<<z, exit(-1);
#define CHECK_GE(x,y,z) if(!(x>=y)) pr(x), pr(y)<<z, exit(-1);
#define CHECK_LE(x,y,z) if(!(x<=y)) pr(x), pr(y)<<z, exit(-1);
#define CHECK_LT(x,y,z) if(!(x<y))  pr(x), pr(y)<<z, exit(-1);



#define SET0VECTOR(x)   {for(auto &i:x)i=0;}

namespace param
{
    namespace cloud2map
    {
        //包含所有参数，可以json载入后修改
        static std::string frame = "global_frame";
        static double search_radius = 1;
        static double deviation = .78539816339;
        static int buffer = 5;
        static double loop_rate = 10;
        static double cell_resolution = 1;
    }

    namespace function
    {
        template<typename T>
        void fill_vector(std::vector<T> &ref, T value)
        {
            for (auto &it : ref)    it = value;
        }
    }

    namespace ros
    {
        static std::string fream_id = "map";
    }

    static Eigen::Matrix4d from5to0() {
        Eigen::Matrix4d T;
        T<<  0.0511272   ,-0.998678 ,-0.00547231   , -126.835,
                0.998625 ,  0.0511864  ,-0.0113121 ,   -107.797,
                0.0115774 ,-0.00488643   ,  0.99992  ,  -1.92527,
                0          , 0 ,          0         ,  1;

        return T;
    }
    static Eigen::Matrix4d from5to1() {
        Eigen::Matrix4d T;
        T<<-0.296461 , -0.954916, -0.0157252,   -99.4052,
                0.955015  , -0.29628 ,-0.0128589  ,  -98.583,
                0.00762008 ,-0.0188299 ,  0.999794  , -3.10328,
                0      ,    0     ,     0    ,      1;
        return T;
    }
    static Eigen::Matrix4d from5to4() {
        Eigen::Matrix4d T;
        T<<-0.0157734   ,-0.999872 ,-0.00279444     ,52.5816,
                0.999457  ,-0.0158477   ,0.0288946    ,-81.3838,
                -0.0289351 ,-0.00233715    ,0.999578     ,1.19668,
                0           ,0           ,0          , 1;

        return T;
    }
    static Eigen::Matrix4d from1to0() {
        Eigen::Matrix4d T;
        T<< 0.926905 , 0.375084, 0.0125961 , 0.741078,
                -0.374854  ,0.926921, -0.017389 , -50.6464,
                -0.018198 ,0.0113963  ,0.999769, -0.223996,
                0     ,    0    ,     0   ,      1;
        return T;
    }
    static Eigen::Matrix4d from2to1() {
        Eigen::Matrix4d T;
        double pi = 3.1415926535;
        double th = 0;
        T<<  -0.665515 ,   0.746364, -0.00543157,     -51.672,
                -0.746301 ,  -0.665532,  -0.0101054 ,    70.4625,
                -0.0111572 ,-0.00267169 ,   0.999934,    0.311524,
                0        ,   0        ,   0          , 1;
        return T;
    }
    static Eigen::Matrix4d from3to2() {
        Eigen::Matrix4d T;
        double pi = 3.1415926535;
        double th = 0;
        T<< 0.611105   ,-0.791299   ,0.0199067     ,69.1827,
                0.791451    ,0.611233 ,0.000422169   , -152.546,
                -0.0125017   ,0.0154972    ,0.999802    ,-1.22904,
                0           ,0           ,0           ,1;
        return T;
    }
    static Eigen::Matrix4d from4to3() {
        Eigen::Matrix4d T;
        double pi = 3.1415926535;
        double th = 0;
        T<<-0.288368  ,-0.957459  ,0.0107539  ,  171.191,
                0.957344 , -0.288081  ,0.0224025    ,16.8985,
                -0.0183515  ,0.0167553   ,0.999691   ,0.219774,
                0          ,0          ,0          ,1;
        return T;
    }
}


struct CVMat
{
    cv::Mat mat;
    std::string file_name;
    double origin_x;
    double origin_y;
    //其他参数
    //TODO

    Json::Value json()
    {
        Json::Value root;
        root["file"] = file_name;
        root["origin_x"] = origin_x;
        root["origin_y"] = origin_y;
        return std::move(root);
        //TODO
    }

    void loadFromJson(Json::Value root)
    {
        file_name = root["file"].asString();
        mat = cv::imread(file_name);
        origin_x = root["origin_x"].asDouble();
        origin_y = root["origin_y"].asDouble();
        //TODO
    }

    int rows()  const { return mat.rows;    }
    int cols()  const { return mat.cols;    }

    cv::Mat& operator ()()
    {
        return mat;
    }

    uchar& at(const int & x, const int & y)
    {
        return mat.at<uchar>(x, y);
    }
};


/*
class BoundingBox
{
public:
    void setMinX(double x){ min_x_ = x; flag_|=1;}
    void setMaxX(double x){ max_x_ = x; flag_|=2;}
    void setMinY(double y){ min_y_ = y; flag_|=4;}
    void setMaxY(double y){ max_y_ = y; flag_|=8;}
    void setX(int x) { X_ = x; flag_|=16;}
    void setY(int y) { Y_ = y; flag_|=32;}

    void setX(double x) = delete;
    void setX(float x) = delete;
    void setX(unsigned char x) = delete;
    void setX(short x) = delete;

    void setY(double y) = delete;
    void setY(float y) = delete;
    void setY(unsigned char y) = delete;
    void setY(short y) = delete;
    BoundingBox()
    {
        flag_ = 0;
    }

    explicit BoundingBox(int X, int Y)
    {
        flag_ = 0;
        setX(X);
        setY(Y);
    }

    explicit BoundingBox(double min_x, double max_x, double min_y, double max_y)
    {
        flag_ = 0;
        setMinX(min_x);
        setMinX(max_x);
        setMinY(min_y);
        setMaxY(max_y);
    }

    explicit BoundingBox(double min_x, double max_x, double min_y, double max_y, int X, int Y)
    {
        flag_ = 0;
        setX(X);
        setY(Y);
        setMinX(min_x);
        setMinX(max_x);
        setMinY(min_y);
        setMaxY(max_y);
    }

    void loadFromJson(Json::Value root)
    {
        min_x_ = root["min_x"].asDouble();
        max_x_ = root["max_x"].asDouble();
        min_y_ = root["min_y"].asDouble();
        max_y_ = root["max_y"].asDouble();
        X_ = root["X"].asInt();
        Y_ = root["Y"].asInt();
        flag_ = root["flag"].asInt();
    }

    Json::Value json()
    {
        Json::Value root;
        root["min_x"] = min_x_;
        root["max_x"] = max_x_;
        root["min_y"] = min_y_;
        root["max_y"] = max_y_;
        root["X"] = X_;
        root["Y"] = Y_;
        root["flag"] = flag_;
        return std::move( root );
    }

    std::pair<int, int> point2XY(geometry_msgs::Point point)
    {
        CHECK_EQ(flag_, 63, "没有完整的初始化所有信息");
        CHECK_GE(max_x_, min_x_, "x的最大值小于最小值？");
        CHECK_GE(max_y_, min_y_, "y的最大值小于最小值？");

        std::pair<int, int> ret;
        double tot_x = max_x_ - min_x_ + 1;
        double tot_y = max_y_ - min_y_ + 1;
        ret.first  = round( ( point.x - min_x_ ) / tot_x * X_ );
        ret.second = round( ( point.y - min_y_ ) / tot_y * Y_ );

        assert(ret.first >= 0);
        assert(ret.second >= 0);
        assert(ret.first < X_);
        assert(ret.second < Y_);
        return std::move( ret );
    };

    geometry_msgs::Point XY2point(int x, int y)
    {
        CHECK_EQ(flag_, 63, "没有完整的初始化所有信息, common.h");
        CHECK_GE(max_x_, min_x_, "x的最大值小于最小值？ commom.h");
        CHECK_GE(max_y_, min_y_, "y的最大值小于最小值？ common.h");

        geometry_msgs::Point ret;
        ret.x = (max_x_ - min_x_) / X_ * x + min_x_;
        ret.y = (max_y_ - min_y_) / Y_ * y + min_y_;
        return std::move( ret );
    }


    bool isVaild(geometry_msgs::Point point)
    {
        CHECK_EQ(flag_, 63, "没有完整的初始化所有信息 common.h");
        CHECK_GE(max_x_, min_x_, "x的最大值小于最小值？ common.h");
        CHECK_GE(max_y_, min_y_, "y的最大值小于最小值？ common.h");

        if ( point.x > max_x_ )  return false;
        if ( point.x < min_x_ )  return false;
        if ( point.y > max_y_ )  return false;
        if ( point.y < min_y_ )  return false;
        return true;
    }

    int X() const {return X_;}
    int Y() const {return Y_;}



private:
    double min_x_;
    double max_x_;
    double min_y_;
    double max_y_;
    int X_;
    int Y_;
    int flag_ = 0;
};
 */

#endif //MAP_COMMON_H
