#ifndef ROBOMOTION_H
#define ROBOMOTION_H

#include <SerialStream.h>
#include <ctime>
#include <iostream>
#include <chrono>
#include <thread>
#include <raspicam/raspicam_cv.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <time.h>
#include <algorithm>
#include "irobot-create.hh"
#include "robocontour.h"

using namespace iRobot;
using namespace LibSerial;
using namespace std::chrono;
using namespace cv;
using namespace std;

void robotmotion(Create*, pthread_mutex_t*, bool*, bool*);


#endif
