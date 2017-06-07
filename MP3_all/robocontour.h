#ifndef ROBOCONTOUR_H
#define ROBOCONTOUR_H

#include <stdio.h>
#include <ctime>
#include <iostream>
#include <time.h>
#include <chrono>
#include <thread>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <vector>
#include <math.h>
#include <stdlib.h>
#include <fstream>
#include <string>
#include <unistd.h>

using namespace cv;
using namespace std;

// trasform the world coordinate to image coordinate
Point2f world_2_image(Point2f point);

Point2f move_straight(Point2f point, float dir);

int robotcontour(vector<float>, vector<float>);

#endif
