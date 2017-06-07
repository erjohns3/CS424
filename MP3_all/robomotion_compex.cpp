#include "robomotion.h"

void robotmotion(Create* robot, pthread_mutex_t* mutex_robot, bool* drive, bool* stop){

    // add by xiao
	vector<float> distance;
	vector<float> angle;

	const short sleep_time = 5;
	const short forward_speed = 150;
	const short rotate_speed = 80;
	const short backward_speed = 80;
	bool foundWall = false;
	short targetWall = 80;
	short wallSignal;
	short diff = 0;
	short step = 0;

	steady_clock::time_point clock0;
	steady_clock::time_point distClock;

	pthread_mutex_lock (mutex_robot);

	distClock = steady_clock::now();

	robot->sendDriveDirectCommand (forward_speed, forward_speed);

	while(!robot->bumpLeft() && !robot->bumpRight()){
		pthread_mutex_unlock (mutex_robot);
			this_thread::sleep_for(chrono::milliseconds(sleep_time));
			pthread_mutex_lock (mutex_robot);
	}
	robot->sendDriveDirectCommand (-rotate_speed, rotate_speed);
	while(!*stop && robot->wallSignal() > 10){
		pthread_mutex_unlock (mutex_robot);
		this_thread::sleep_for(chrono::milliseconds(sleep_time));
		pthread_mutex_lock (mutex_robot);
	}
	while(!*stop && robot->wallSignal() < 20
	){
		pthread_mutex_unlock (mutex_robot);
		this_thread::sleep_for(chrono::milliseconds(sleep_time));
		pthread_mutex_lock (mutex_robot);
	}
	while(!*stop){
		wallSignal = robot->wallSignal();
		if(wallSignal > targetWall){
			targetWall = wallSignal;
		} else if(wallSignal < 5){
			robot->sendDriveDirectCommand (rotate_speed, -rotate_speed);
			targetWall = targetWall / 2;
			cout << "Target Wall Found: "<<targetWall << endl;
			break;
		}
		pthread_mutex_unlock (mutex_robot);
		this_thread::sleep_for(chrono::milliseconds(sleep_time));
		pthread_mutex_lock (mutex_robot);
	}
	while(!*stop){
		if(robot->wallSignal() > targetWall){
			robot->sendDriveDirectCommand (0, 0);
			break;
		}
	}

	while (!*stop) {

		if(robot->bumpLeft() || robot->bumpRight()){
			robot->sendDriveDirectCommand (0, 0);
			angle.push_back(-1.5707);
			distance.push_back(duration_cast<milliseconds>(steady_clock::now() - distClock).count() * forward_speed / 8000);
			cout << "Left Turn" << endl;
			robot->sendDriveDirectCommand (-rotate_speed, rotate_speed);
			clock0 = steady_clock::now();
			while(!*stop){
				if(robot->wallSignal() > targetWall*3/2){
					robot->sendDriveDirectCommand (0,0);
					break;
				}
				cout << "Wall Signal: "<< robot->wallSignal() << endl;
				if(duration_cast<milliseconds>(steady_clock::now() - clock0).count() > 5000){
					robot->sendDriveDirectCommand (rotate_speed, -rotate_speed);
					clock0 = steady_clock::now();
					while(!*stop && duration_cast<milliseconds>(steady_clock::now() - clock0).count() < 2500){
						pthread_mutex_unlock (mutex_robot);
						this_thread::sleep_for(chrono::milliseconds(sleep_time));
						pthread_mutex_lock (mutex_robot);
					}
					robot->sendDriveDirectCommand (0,0);
					break;
				}
				pthread_mutex_unlock (mutex_robot);
				this_thread::sleep_for(chrono::milliseconds(sleep_time));
				pthread_mutex_lock (mutex_robot);
			}
			distClock = steady_clock::now();
		}

		diff = (targetWall - robot->wallSignal()) / 3;
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
				if(step == 0 && duration_cast<milliseconds>(steady_clock::now() - clock0).count() > 2000){
					robot->sendDriveDirectCommand (rotate_speed, -rotate_speed);
					clock0 = steady_clock::now();
					angle.push_back(1.5707);
					distance.push_back(duration_cast<milliseconds>(steady_clock::now() - distClock).count() * forward_speed / 8000);
					step = 1;
				} else if(step == 1 && duration_cast<milliseconds>(steady_clock::now() - clock0).count() >2500){
					robot->sendDriveDirectCommand (forward_speed, forward_speed);
					clock0 = steady_clock::now();
					distClock = steady_clock::now();
					step = 2;
				} else if(step == 2 && duration_cast<milliseconds>(steady_clock::now() - clock0).count() > 2200){
					break;
				}
				pthread_mutex_unlock (mutex_robot);
				this_thread::sleep_for(chrono::milliseconds(sleep_time));
				pthread_mutex_lock (mutex_robot);
			}
		}

		pthread_mutex_unlock (mutex_robot);
		this_thread::sleep_for(chrono::milliseconds(sleep_time));
		pthread_mutex_lock (mutex_robot);
	}

	distance.push_back(duration_cast<milliseconds>(steady_clock::now() - distClock).count() * forward_speed / 8000);

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

