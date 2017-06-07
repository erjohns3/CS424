#ifndef ROBOSAFETY_H
#define ROBOSAFETY_H

#include <SerialStream.h>
#include "irobot-create.hh"
#include <ctime>
#include <iostream>
#include <chrono>
#include <thread>
#include <raspicam/raspicam_cv.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <time.h>
#include <algorithm>

using namespace iRobot;
using namespace LibSerial;
using namespace std::chrono;
using namespace std;

void robotsafety(Create*, pthread_mutex_t*, bool*, bool*);

#endif
