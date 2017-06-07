#include "robosafety.h"

void robotsafety(Create* robot, pthread_mutex_t* mutex_robot, bool* drive, bool* stop){
	short velLeft = 0;
	short velRight = 0;
	bool driveTest = true;
	int overcurrent = 0;
	pthread_mutex_lock (mutex_robot);
	while(!*stop){
		if(robot->cliffLeftSignal() < 10 || robot->cliffRightSignal() < 10 ||
		robot->cliffFrontLeftSignal() < 10 || robot->cliffFrontRightSignal() < 10 ||
		robot->wheeldropLeft() || robot->wheeldropRight() || robot->wheeldropCaster() ||
		overcurrent >= 5){
			if(*drive){
				velLeft = robot->requestedLeftVelocity();
				velRight = robot->requestedRightVelocity();
				robot->sendDriveDirectCommand(0, 0);
				robot->sendPlaySongCommand(0);
				*drive = false;
				driveTest = false;
				this_thread::sleep_for(chrono::milliseconds(2000));
			}
		}else if(!*drive){
			*drive = true;
			driveTest = true;
			robot->sendDriveDirectCommand(velLeft, velRight);
		}
		if(robot->leftWheelOvercurrent() || robot->rightWheelOvercurrent()){
			overcurrent++;
		}else{
			overcurrent = 0;
		}
		if(robot->playButton()){
			*stop = true;
			driveTest = true;
		}
		if(driveTest){
			pthread_mutex_unlock (mutex_robot);
			this_thread::sleep_for(chrono::milliseconds(100));
			pthread_mutex_lock (mutex_robot);
		}
	}
	robot->sendDriveDirectCommand(0, 0);
	pthread_mutex_unlock (mutex_robot);
	return;
}
