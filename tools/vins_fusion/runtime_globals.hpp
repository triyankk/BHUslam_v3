#pragma once
// One definition per executable; shared offline/live estimator defaults.
#include "estimator.h"
double INIT_DEPTH=5,MIN_PARALLAX=10./460.,ACC_N,ACC_W,GYR_N,GYR_W;
std::vector<Eigen::Matrix3d> RIC;std::vector<Eigen::Vector3d>TIC;
Eigen::Vector3d G(0,0,9.81);
double BIAS_ACC_THRESHOLD=.1,BIAS_GYR_THRESHOLD=.1,SOLVER_TIME=.04,TD=0;
int ROW,COL,NUM_OF_CAM=2,STEREO=1,USE_IMU=1,MULTIPLE_THREAD=0,MAX_CNT=150,MIN_DIST=30,SHOW_TRACK=0,FLOW_BACK=1;
double F_THRESHOLD=1.;
std::string OUTPUT_FOLDER,IMAGE0_TOPIC,IMAGE1_TOPIC,FISHEYE_MASK;
std::vector<std::string> CAM_NAMES;
std::map<int,Eigen::Vector3d> pts_gt;
int NUM_ITERATIONS=8,ESTIMATE_EXTRINSIC=0,ESTIMATE_TD=0,ROLLING_SHUTTER=0;
std::string EX_CALIB_RESULT_PATH,VINS_RESULT_PATH,IMU_TOPIC;
