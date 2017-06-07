#include "robomotion.h"

void robotmotion(Create* robot, pthread_mutex_t* mutex_robot, bool* drive, bool* stop){

    // add by xiao
	vector<float> distance;
	vector<float> angle;

	const short sleep_time = 5;
	const short forward_speed = 200;
	const short rotate_speed = 200;
	bool foundWall = false;
	bool shortWall = false;
	bool skip = false;
	short targetWall = 80;
	short wallSignal;
	short diff = 0;
	short step = 0;
	short prevWallSignal = 0;
	short maxWallDiff = 0;
	short bumpCount = 0;

	steady_clock::time_point clock0;
	steady_clock::time_point bumpClock;
	steady_clock::time_point distClock;

	pthread_mutex_lock (mutex_robot);

	distClock = steady_clock::now();
	bumpClock = steady_clock::now();
	robot->sendDriveDirectCommand (forward_speed, forward_speed);

	while (!*stop) {

		if(robot->bumpLeft() || robot->bumpRight()){
			robot->sendDriveDirectCommand (0, 0);
			skip = false;
			if(duration_cast<milliseconds>(steady_clock::now() - bumpClock).count() < 300){
				bumpCount++;
			}else{
				bumpCount = 0;
			}
			if(foundWall){
				distance.push_back(double(duration_cast<milliseconds>(steady_clock::now() - distClock).count()) * 0.02);
			}
			cout << "Bump Count: "<< bumpCount << endl;
			if(bumpCount >= 1){
				robot->sendDriveDirectCommand (-rotate_speed, rotate_speed);
				clock0 = steady_clock::now();
				while(!*stop && duration_cast<milliseconds>(steady_clock::now() - clock0).count() < 1000){
					pthread_mutex_unlock (mutex_robot);
					this_thread::sleep_for(chrono::milliseconds(sleep_time));
					pthread_mutex_lock (mutex_robot);
				}
				skip = true;
				robot->sendDriveDirectCommand (0, 0);
				angle.push_back(-1.5708);
			}
			cout << "Left Turn" << endl;
			robot->sendDriveDirectCommand (-rotate_speed, rotate_speed);
			clock0 = steady_clock::now();
			shortWall = false;
			targetWall = 10;
			maxWallDiff = 0;
			prevWallSignal = robot->wallSignal();
			while(!*stop && !skip && duration_cast<milliseconds>(steady_clock::now() - clock0).count() < 2000){
				wallSignal = robot->wallSignal();
				if(wallSignal > targetWall){
					targetWall = wallSignal;
				}
				pthread_mutex_unlock (mutex_robot);
				this_thread::sleep_for(chrono::milliseconds(sleep_time));
				pthread_mutex_lock (mutex_robot);
			}
			if(!skip){
				targetWall = targetWall / 2;
			}
			if(targetWall > 120){
				targetWall = 120;
			}
			robot->sendDriveDirectCommand (rotate_speed, -rotate_speed);
			clock0 = steady_clock::now();
			while(!*stop && !skip && robot->wallSignal() < targetWall*3/4 && duration_cast<milliseconds>(steady_clock::now() - clock0).count() < 2000){
				pthread_mutex_unlock (mutex_robot);
				this_thread::sleep_for(chrono::milliseconds(sleep_time));
				pthread_mutex_lock (mutex_robot);
			}
			if(foundWall && !skip){
				angle.push_back((double(duration_cast<milliseconds>(steady_clock::now() - clock0).count()) / 645.0) - 3.1001);
			}
			robot->sendDriveDirectCommand (0, 0);
			bumpClock = steady_clock::now();
			distClock = steady_clock::now();
			foundWall = true;
		}

		//cout << "Wall Signal: " << robot->wallSignal() << endl;

		if(foundWall){
			diff = (targetWall - robot->wallSignal()) / 3;
			if(diff > 75){
				diff = 75;
			}else if(diff < -75){
				diff = -75;
			}
			robot->sendDriveDirectCommand (forward_speed+diff, forward_speed-diff);

			if(robot->wallSignal() < 5){
				robot->sendDriveDirectCommand (forward_speed, forward_speed);
				clock0 = steady_clock::now();
				cout << "Right Turn" << endl;
				step = 0;
				while(!*stop){
					if((robot->wallSignal() > 5 && step != 1) || robot->bumpRight() || robot->bumpLeft()){
						break;
					}
					if(step == 0 && duration_cast<milliseconds>(steady_clock::now() - clock0).count() > 1400){
						robot->sendDriveDirectCommand (rotate_speed, -rotate_speed);
						clock0 = steady_clock::now();
						angle.push_back(1.5708);
						distance.push_back(double(duration_cast<milliseconds>(steady_clock::now() - distClock).count()) * 0.02);
						step = 1;
					} else if(step == 1 && duration_cast<milliseconds>(steady_clock::now() - clock0).count() >1000){
						robot->sendDriveDirectCommand (forward_speed, forward_speed);
						clock0 = steady_clock::now();
						distClock = steady_clock::now();
						step = 2;
					} else if(step == 2 && duration_cast<milliseconds>(steady_clock::now() - clock0).count() > 1500){
						break;
					}
					pthread_mutex_unlock (mutex_robot);
					this_thread::sleep_for(chrono::milliseconds(sleep_time));
					pthread_mutex_lock (mutex_robot);
				}
			}
		}
		pthread_mutex_unlock (mutex_robot);
		this_thread::sleep_for(chrono::milliseconds(sleep_time));
		pthread_mutex_lock (mutex_robot);
	}

	distance.push_back(double(duration_cast<milliseconds>(steady_clock::now() - distClock).count()) * 0.02);

	robot->sendDriveDirectCommand (0,0);
	cout<< "Distance"<<endl;
	for(int i = 0; i< distance.size(); i++){
		cout<< distance[i] <<endl;
	}
	cout<< "Angle"<<endl;
	for(int i = 0; i< angle.size(); i++){
		cout<< angle[i] <<endl;
	}

	robotcontour(distance, angle);

	pthread_mutex_unlock (mutex_robot);
	return;
}

