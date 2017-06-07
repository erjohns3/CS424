#include <SerialStream.h>
#include <iostream>
#include <chrono>
#include <thread>
#include <pthread.h>
#include <stdio.h>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <raspicam/raspicam_cv.h>
#include <opencv2/imgproc/imgproc.hpp>
#include "irobot-create.hh"
#include "robosafety.h"
#include "robomotion.h"
#include "robovision.h"

using namespace iRobot;
using namespace LibSerial;
using namespace std;
using namespace cv;
using namespace std::chrono;

SerialStream stream ("/dev/ttyUSB0", LibSerial::SerialStreamBuf::BAUD_57600);
Create robot(stream);
bool drive;
bool stop;

pthread_mutex_t mutex_robot;
pthread_mutex_t mutex_images;

void *RoboSafety(void *x){
	robotsafety(&robot, &mutex_robot, &drive, &stop);
	cout << endl << "^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^Safety Done^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^" << endl << endl;
	pthread_exit(NULL);
}

void *RoboMotion(void *x){
	robotmotion(&robot, &mutex_robot, &drive, &stop);
	cout << endl << "^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^Motion Done^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^" << endl << endl;
	pthread_exit(NULL);
}

void *RoboCamera(void *x){
	robotcamera(&robot, &mutex_robot, &mutex_images, &stop);
	cout << endl << "^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^Camera Done^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^" << endl << endl;
	pthread_exit(NULL);
}

void *RoboVision(void *x){
	cout << "Vision " << " Started" << endl;
	search_magic_lamp(&robot, &mutex_robot, &mutex_images, &stop, int (x)) ;
	cout << "Target disarmed! Start analizing scenes ... "<<endl<<endl;
	after_mission_task();
	img_proc(&robot, &mutex_robot, &mutex_images, &stop, int (x));
	cout << endl << "^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^Vision Done^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^" << endl << endl;
	pthread_exit(NULL);
}

int main ()
{
	srand(time(NULL));
	try
	{
		cout << "Created iRobot Object" << endl;
		robot.sendFullCommand();
		cout << "Setting iRobot to Full Mode" << endl;
		this_thread::sleep_for(chrono::milliseconds(1000));
		cout << "Robot is ready" << endl;

	 	Create::sensorPackets_t sensors;
		sensors.push_back(Create::SENSOR_BUMPS_WHEELS_DROPS);
		sensors.push_back(Create::SENSOR_WALL_SIGNAL);
		sensors.push_back(Create::SENSOR_BUTTONS);
		sensors.push_back(Create::SENSOR_CLIFF_LEFT_SIGNAL);
		sensors.push_back(Create::SENSOR_CLIFF_FRONT_LEFT_SIGNAL);
		sensors.push_back(Create::SENSOR_CLIFF_RIGHT_SIGNAL);
		sensors.push_back(Create::SENSOR_CLIFF_FRONT_RIGHT_SIGNAL);
		sensors.push_back(Create::SENSOR_OVERCURRENTS);
		sensors.push_back(Create::SENSOR_REQUESTED_LEFT_VELOCITY);
		sensors.push_back(Create::SENSOR_REQUESTED_RIGHT_VELOCITY);
		robot.sendStreamCommand (sensors);
		cout << "Sent Stream Command" << endl;

		int ledColor = Create::LED_COLOR_GREEN;
		robot.sendLedCommand (Create::LED_PLAY, 0, 0);

		Create::song_t song0;
		song0.push_back(Create::note_t(100, 10));
		song0.push_back(Create::note_t(90, 10));
		song0.push_back(Create::note_t(80, 10));
		robot.sendSongCommand(0,song0);

		drive = true;
		stop = false;

		//pid_t pid = getpid();
		//cout<< "process ID: "<<  pid <<endl;
		//int ret = sched_setscheduler(pid, SCHED_RR, const struct sched_param *param);
		//cout << "Set Schedule: " << ret<< endl;
		
		pthread_attr_t attrSafety;
		sched_param paramSafety;
		pthread_attr_init (&attrSafety);
		pthread_attr_getschedparam (&attrSafety, &paramSafety);
		paramSafety.sched_priority = 4;
		pthread_attr_setschedparam (&attrSafety, &paramSafety);

		pthread_attr_t attrMotion;
		sched_param paramMotion;
		pthread_attr_init (&attrMotion);
		pthread_attr_getschedparam (&attrMotion, &paramMotion);
		paramMotion.sched_priority = 3;
		pthread_attr_setschedparam (&attrMotion, &paramMotion);

		pthread_attr_t attrCamera;
		sched_param paramCamera;
		pthread_attr_init (&attrCamera);
		pthread_attr_getschedparam (&attrCamera, &paramCamera);
		paramCamera.sched_priority = 2;
		pthread_attr_setschedparam (&attrCamera, &paramCamera);

		pthread_attr_t attrVision;
		sched_param paramVision;
		pthread_attr_init (&attrVision);
		pthread_attr_getschedparam (&attrVision, &paramVision);
		paramVision.sched_priority = 1;
		pthread_attr_setschedparam (&attrVision, &paramVision);

		//after_mission_task();

		pthread_t thread_camera;
		pthread_create(&thread_camera, &attrCamera, RoboCamera, (void *)0);

		//extract_target_features();

		pthread_t thread_safety;
		pthread_create(&thread_safety, &attrSafety, RoboSafety, (void *)0);
		
		pthread_t thread_motion;
		pthread_create(&thread_motion, &attrMotion, RoboMotion, (void *)0);
		
		pthread_t thread_vision;
		pthread_create(&thread_vision, &attrVision, RoboVision, (void *)0);

		pthread_join(thread_safety, NULL);
		pthread_join(thread_motion, NULL);
		pthread_join(thread_camera, NULL);
		pthread_join(thread_vision, NULL);
		
		robot.sendDriveDirectCommand(0, 0);
  	}
  	catch (InvalidArgument& e){
    	cerr << e.what () << endl;
    	return 3;
  	}
  	catch (CommandNotAvailable& e){
    	cerr << e.what () << endl;
    	return 4;
  	}
}
