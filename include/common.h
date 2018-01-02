//
// Created by bohuan on 17-12-20.
//

#ifndef MAP_CORE_COMMON_H
#define MAP_CORE_COMMON_H

#include <bits/stdc++.h>
#include <json/json.h>
#include <opencv2/opencv.hpp>
#include <geometry_msgs/Point.h>
#include <eigen3/Eigen/Core>

#define pr(x)   std::cout<<#x<<"="<<x<<","
#define prln(x) std::cout<<#x<<"="<<x<<std::endl

#define CHECK_EQ(x,y,z) if(!(x==y)) pr(x), pr(y)<<z, exit(-1);
#define CHECK_GE(x,y,z) if(!(x>=y)) pr(x), pr(y)<<z, exit(-1);
#define CHECK_LE(x,y,z) if(!(x<=y)) pr(x), pr(y)<<z, exit(-1);
#define CHECK_LT(x,y,z) if(!(x<y))  pr(x), pr(y)<<z, exit(-1);


#define DEBUG



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
        static std::string fream_id = "global_frame";
    }

    static int max_markers = 5;
    static int min_contour_area = 200;      //最小面积，低于这个面积的矩形框则舍弃
    //TODO 考虑长宽不符合要求的矩形框。


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
    static Eigen::Matrix4d from6to1() {
        Eigen::Matrix4d T;
        T<< -0.94738, 0.320012  ,-0.0079394, -26.77,
                -0.319978 ,   -0.94741 ,-0.00526192     ,2.88944,
                -0.00920575  ,-0.0024446    ,0.999955   ,-0.166168,
                0           ,0           ,0           ,1;
        return T;
    }
    static Eigen::Matrix4d from7to6() {    //有2个common area
        Eigen::Matrix4d T;
        T<<-0.00837027,    -0.999965 ,-0.000240553       ,122.62,
                0.999384  ,-0.00837361    ,0.0340737      ,-47.365,
                -0.0340745  ,4.48013e-05     ,0.999419    ,-0.364636,
                0,            0 ,           0            ,1;
        return T;
    }
    static Eigen::Matrix4d from8to7() {
        Eigen::Matrix4d T;
        T<<-0.998715 ,-0.0446334 ,-0.0240076 ,   36.0698,
                0.0444052  ,-0.998964 ,0.00995591    ,106.717,
                -0.0244271 ,0.00887706   ,0.999662    ,3.19942,
                0          ,0          ,0          ,1;

        return T;
    }
    static Eigen::Matrix4d from8to4() {
        Eigen::Matrix4d T;
        T<<0.137088 , -0.990182,  0.0273246    ,140.275,
                0.990421   ,0.137476  ,0.0128883    ,1.53724,
                -0.0165182   ,0.025296   ,0.999544  ,-0.160449,
                0          ,0          ,0          ,1;
        return T;
    }
    static Eigen::Matrix4d from8to1() {
        Eigen::Matrix4d T;
        T<<-0.267848  ,-0.963411 ,0.00980218   ,-47.5639,
                0.963338 , -0.267963 ,-0.0132726    ,10.1437,
                0.0154136 ,0.00588778   ,0.999864   ,0.127897,
                0         , 0,          0          ,1;
        return T;
    }
    static Eigen::Matrix4d from6to3() {
        Eigen::Matrix4d T;
        T<<0.192757    ,0.981223 ,-0.00674641    , 141.562,
                -0.981023    ,0.192855   ,0.0200288     ,167.649,
                0.0209538  ,0.00275769     ,0.999777    ,-2.68266,
                0          , 0           ,0           ,1;
        return T;
    }
    static Eigen::Matrix4d from7to1() {
        Eigen::Matrix4d T;
        T<<0.281166     ,0.959653  ,-0.00340603     ,-157.965,
                -0.959659     ,0.281167 ,-0.000206412      ,12.6948,
                0.000759578   ,0.00332666     ,0.999994     ,-3.34021,
                0            ,0            ,0            ,1;
        return T;
    }
    static Eigen::Matrix4d from7to3() {
        Eigen::Matrix4d T;
        T<< 0.970569,  -0.240251  ,0.0165998     ,119.76,
                0.239805   ,0.970497  ,0.0250727    ,37.6125,
                -0.0221339 ,-0.0203541   ,0.999548   ,-0.64511,
                0          ,0,          0          ,1;
        return T;
    }
    static Eigen::Matrix4d from9to8() {
        Eigen::Matrix4d T;
        T<<-0.986617    ,0.162443  ,-0.0141145    ,-46.6636,
                -0.16277   ,-0.986311   ,0.0264023     ,124.779,
                -0.00963239   ,0.0283463    ,0.999552    ,-3.62838,
                0        ,   0           ,0           ,1;
        return T;
    }
    static Eigen::Matrix4d from9to4() {
        Eigen::Matrix4d T;
        T<<0.0251325    ,0.999678  ,0.00351628     ,10.2227,
                -0.999681   ,0.0251231  ,0.00270647    ,-27.6293,
                0.00261726 ,-0.00358318     ,0.99999   ,0.0957625,
                0    ,       0,           0,           1;
        return T;

    }
    static Eigen::Matrix4d from9to5() {
        Eigen::Matrix4d T;
        T<<-0.99975   ,0.0145718   ,0.0169735     ,54.0589,
                -0.0146847   ,-0.999871 ,-0.00654555     ,42.2783,
                0.016876 ,-0.00679316    ,0.999835    ,-1.24848,
                0         ,  0           ,0           ,1;
        return T;
    }
    static Eigen::Matrix4d from10to9(){
        Eigen::Matrix4d T;
        T<<0.98497  ,-0.171361  ,0.0216812   ,-67.9666,
                0.171446   ,0.985191 ,-0.0020889    ,83.5592,
                -0.0210022 ,0.00577465   ,0.999763    ,2.46844,
                0          ,0 ,         0          ,1;
        return T;
    }
    static Eigen::Matrix4d from10to6(){
        Eigen::Matrix4d T;
        T<<-0.244588  ,-0.969622 ,0.00314414 ,   63.4026,
                0.969568  ,-0.244535  ,0.0118562    ,-47.568,
                -0.0107272 ,0.00594834   ,0.999925   ,0.490676,
                0          ,0          ,0          ,1;
        return T;
    }
    static Eigen::Matrix4d from10to7(){
        Eigen::Matrix4d T;
        T<<-0.250948  ,-0.967999 ,0.00153728    ,63.5254,
                0.967883  ,-0.250892  ,0.0159982   ,-47.5605,
                -0.0151005 ,0.00550261   ,0.999871   ,0.483078,
                0          ,0,          0          ,1;
        return T;
    }
    static Eigen::Matrix4d from10to8(){
        Eigen::Matrix4d T;
        T<< -0.95877    ,0.283783   ,-0.015079     ,31.3666,
                -0.284041   ,-0.958619   ,0.0192542     ,48.8551,
                -0.00899103   ,0.0227434    ,0.999701    ,-1.06879,
                0,           0    ,       0,           1;
        return T;
    }
    static Eigen::Matrix4d from10to5(){
        Eigen::Matrix4d T;
        T<<-0.984902   ,0.169741  ,-0.034011    ,123.261,
                -0.169316  ,-0.985447 ,-0.0150169   ,-42.4103,
                -0.036065 ,-0.0090316   ,0.999309    ,2.55084,
                0          ,0,          0          ,1;
        return T;
    }
    static Eigen::Matrix4d from10to4(){
        Eigen::Matrix4d T;
        T<<0.192596     ,0.981177    ,0.0140721      ,92.1292,
                -0.981278     ,0.192586   ,0.00211145   ,   42.6654,
                -0.000638375  , -0.0142153     ,0.999899    ,-0.594045,
                0            ,0            ,0            ,1;

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
#endif
