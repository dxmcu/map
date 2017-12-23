#ifndef PARAMETERS_H
#define PARAMETERS_H

#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include <ros/ros.h>

extern std::string DATA_PATH;
extern std::string OUTPUT_PATH;
extern std::string INPUT_BAG_FILE_NAME, OUTPUT_MAP_NAME;

extern std::string POINTMATCHER_YAML;

extern std::string LASER_TOPIC_NAME, ODOM_TOPIC_NAME;

extern int INTERVAL_OF_SCANS;

extern int LOOP_ENABLE;

extern double SPEED_OF_CAR;

extern double MAX_TURNING_SPEED;

extern int SECOND_STATION;
extern int STATION_ENABLE;

extern double STOP_WAIT_TIME;

extern int ODOM_RAW;

void ReadParameters(std::string config_file);


#endif // PARAMETERS_H
