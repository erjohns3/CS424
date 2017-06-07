#ifndef ROBOVISION_H
#define ROBOVISION_H

#include <chrono>
#include <iostream>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <string>
#include <SerialStream.h>
#include "irobot-create.hh"
#include <ctime>
#include <chrono>
#include <thread>
#include <string>
#include <stdio.h>
#include <stdlib.h>
#include <raspicam/raspicam_cv.h>
#include <opencv2/imgproc/imgproc.hpp>

using namespace cv;
using namespace cv::xfeatures2d;
using namespace iRobot;
using namespace LibSerial;
using namespace std;

using std::chrono::duration;
using std::chrono::steady_clock;
using std::cout;
using std::endl;
using std::string;
using std::vector;

bool alignPerspective(vector<Point2f>& query, vector<Point2f>& scene,
    Mat& img_query, Mat& img_scene, vector<Point2f>& scene_corners);
void cropBottom(Mat& img_scene_full, Mat& img_scene, float crop_fraction);
void drawProjection(Mat& img_matches, Mat& img_query,
    vector<Point2f>& scene_corners);
string type2str(int type);

int robotcamera(Create*, pthread_mutex_t*, pthread_mutex_t*, bool*);

void after_mission_task(void);
int img_proc(Create* robot, pthread_mutex_t* mutex_robot, pthread_mutex_t* mutex_images, bool* stop, int x);
int search_magic_lamp(Create* robot, pthread_mutex_t* mutex_robot, pthread_mutex_t* mutex_images, bool* stop, int x);


extern vector<Mat> images;

#endif
