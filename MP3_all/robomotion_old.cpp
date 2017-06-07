#include "robomotion.h"

void robotmotion(Create* robot, pthread_mutex_t* mutex_robot, bool* drive, bool* stop){
	vector<float> distance;
	vector<float> angle;

	const short forward_speed = 180;
	const short rotate_speed = 80;
	const short backward_speed = 80;
	bool foundWall = false;
	bool bumpLeft0 = false;
	bool bumpLeft1 = false;
	bool bumpLeft2 = false;
	bool bumpRight0 = false;
	bool bumpRight1 = false;
	bool bumpRight2 = false;
	short wallSignal = 0;
	short wall0 = 0;
	short wall1 = 0;
	short wall2 = 0;
	short targetWall = 80;  //check xiao
	short diff = 0;
	bool skip = false;

	steady_clock::time_point clock0;
	steady_clock::time_point distClock;
	steady_clock::time_point bumpClock;
	steady_clock::time_point wallClock;

	pthread_mutex_lock (mutex_robot);

	distClock = steady_clock::now();
	bumpClock = steady_clock::now();
	wallClock = steady_clock::now();

	robot->sendDriveDirectCommand (forward_speed, forward_speed);
	while (!*stop) {
	
		if(*drive){
			wallSignal = robot->wallSignal();
			
			if(duration_cast<milliseconds>(steady_clock::now() - wallClock).count() > 20){
				wallClock = steady_clock::now();
				if(wallSignal != wall0 || wallSignal < 5){
					wall2 = wall1;
					wall1 = wall0;
					wall0 = wallSignal;
				}
			}

			if(duration_cast<milliseconds>(steady_clock::now() - bumpClock).count() > 20){
				bumpClock = steady_clock::now();
				bumpRight2 = bumpRight1;
				bumpRight1 = bumpRight0;
				bumpRight0 = robot->bumpRight();
				bumpLeft2 = bumpLeft1;
				bumpLeft1 = bumpLeft0;
				bumpLeft0 = robot->bumpLeft();
			}

			// rotate counter-clockwise to allign with the wall
			// check right bump condition
			if(bumpRight1 && !bumpLeft0){
				cout << "Alligning" << endl;
				robot->sendDriveDirectCommand (-backward_speed, -backward_speed);
				clock0 = steady_clock::now();
				// move backward
				while(!*stop && duration_cast<milliseconds>(steady_clock::now() - clock0).count() < 30000/backward_speed){
					pthread_mutex_unlock (mutex_robot);
					this_thread::sleep_for(chrono::milliseconds(5));
					pthread_mutex_lock (mutex_robot);
				}
				// rotate counter-clockwise
				robot->sendDriveDirectCommand (-rotate_speed, rotate_speed);
				clock0 = steady_clock::now();
		
		
				cout << "passed mini peak" << endl;
				// until wall signal is strong // check xiao second condition
				while(!*stop && robot->wallSignal() < targetWall*3/4 && duration_cast<milliseconds>(steady_clock::now() - clock0).count() < 300000/rotate_speed){
					pthread_mutex_unlock (mutex_robot);
					this_thread::sleep_for(chrono::milliseconds(5));
					pthread_mutex_lock (mutex_robot);
				}
				
				
				while(!*stop){
					wallSignal = robot->wallSignal();
					if(duration_cast<milliseconds>(steady_clock::now() - wallClock).count() > 20){
						wallClock = steady_clock::now();
						if(wallSignal != wall0 || wallSignal < 5){
							wall2 = wall1;
							wall1 = wall0;
							wall0 = wallSignal;
						}
					}
					

					if(wall0 < wall1 && wall1 < wall2){
						robot->sendDriveDirectCommand (0, 0);
						targetWall = wall0 / 2;
						cout << "Target Wall Found: "<<targetWall << endl;
						break;
					}

					if(duration_cast<milliseconds>(steady_clock::now() - clock0).count() > 400000/rotate_speed){
						robot->sendDriveDirectCommand (rotate_speed, -rotate_speed);
						clock0 = steady_clock::now();
						while(!*stop && duration_cast<milliseconds>(steady_clock::now() - clock0).count() < 200000/rotate_speed){
							pthread_mutex_unlock (mutex_robot);
							this_thread::sleep_for(chrono::milliseconds(5));
							pthread_mutex_lock (mutex_robot);
						}
						robot->sendDriveDirectCommand (0, 0);
						break;
					}
					pthread_mutex_unlock (mutex_robot);
					this_thread::sleep_for(chrono::milliseconds(5));
					pthread_mutex_lock (mutex_robot);
				}
				
				bumpLeft2 = false;
				bumpLeft1 = false;
				bumpLeft0 = false;
				bumpRight2 = false;
				bumpRight1 = false;
				bumpRight0 = false;
				foundWall = true;
				//distClock = steady_clock::now();
			}


			if(robot->bumpLeft()){
				if(foundWall){
					angle.push_back(-1.5707);
					distance.push_back(duration_cast<milliseconds>(steady_clock::now() - distClock).count() * forward_speed / 8000);
				}
				cout << "Left Turn" << endl;

				robot->sendDriveDirectCommand (-backward_speed, -backward_speed);
				clock0 = steady_clock::now();
				while(!*stop && duration_cast<milliseconds>(steady_clock::now() - clock0).count() < 10000/backward_speed){
					pthread_mutex_unlock (mutex_robot);
					this_thread::sleep_for(chrono::milliseconds(5));
					pthread_mutex_lock (mutex_robot);
				}

				robot->sendDriveDirectCommand (-rotate_speed, rotate_speed);
				clock0 = steady_clock::now();
				while(!*stop && robot->wallSignal() > targetWall*3/8 && duration_cast<milliseconds>(steady_clock::now() - clock0).count() < 300000/rotate_speed){
					pthread_mutex_unlock (mutex_robot);
					this_thread::sleep_for(chrono::milliseconds(5));
					pthread_mutex_lock (mutex_robot);
				}
				cout << "passed mini peak" << endl;
				while(!*stop && robot->wallSignal() < targetWall*7/8 && duration_cast<milliseconds>(steady_clock::now() - clock0).count() < 300000/rotate_speed){
					pthread_mutex_unlock (mutex_robot);
					this_thread::sleep_for(chrono::milliseconds(5));
					pthread_mutex_lock (mutex_robot);
				}
				while(!*stop){
					wallSignal = robot->wallSignal();
					if(duration_cast<milliseconds>(steady_clock::now() - wallClock).count() > 20){
						wallClock = steady_clock::now();
						if(wallSignal != wall0 || wallSignal < 5){
							wall2 = wall1;
							wall1 = wall0;
							wall0 = wallSignal;
						}
					}

					if(wall0 < wall1 && wall1 < wall2){
						robot->sendDriveDirectCommand (0, 0);
						targetWall = wall0 / 2;
						cout << "Target Wall Found: "<<targetWall << endl;
						break;
					}

					if(duration_cast<milliseconds>(steady_clock::now() - clock0).count() > 400000/rotate_speed){
						robot->sendDriveDirectCommand (rotate_speed, -rotate_speed);
						clock0 = steady_clock::now();
						while(!*stop && duration_cast<milliseconds>(steady_clock::now() - clock0).count() < 200000/rotate_speed){
							pthread_mutex_unlock (mutex_robot);
							this_thread::sleep_for(chrono::milliseconds(5));
							pthread_mutex_lock (mutex_robot);
						}
						robot->sendDriveDirectCommand (0, 0);
						break;
					}
					pthread_mutex_unlock (mutex_robot);
					this_thread::sleep_for(chrono::milliseconds(5));
					pthread_mutex_lock (mutex_robot);
				}
				bumpLeft2 = false;
				bumpLeft1 = false;
				bumpLeft0 = false;
				bumpRight2 = false;
				bumpRight1 = false;
				bumpRight0 = false;
				foundWall = true;
				distClock = steady_clock::now();
			}

			if(foundWall){
				diff = (targetWall - wallSignal) / 3;
				robot->sendDriveDirectCommand (forward_speed+diff, forward_speed-diff);

				if(wall0 < 5 && wall1 < 5 && wall2 < 5){
					robot->sendDriveDirectCommand (forward_speed, forward_speed);
					clock0 = steady_clock::now();
					while(!*stop && !skip && duration_cast<milliseconds>(steady_clock::now() - clock0).count() < 300000/forward_speed){
						wallSignal = robot->wallSignal();
						if(duration_cast<milliseconds>(steady_clock::now() - wallClock).count() > 20){
							wallClock = steady_clock::now();
							wall2 = wall1;
							wall1 = wall0;
							wall0 = wallSignal;
						}
						if(wall0 > 5 && wall1 > 5 && wall2 > 5){
							skip = true;
							break;
						}
						if(robot->bumpRight() || robot->bumpLeft()){
							skip = true;
							break;
						}
						pthread_mutex_unlock (mutex_robot);
						this_thread::sleep_for(chrono::milliseconds(5));
						pthread_mutex_lock (mutex_robot);
					}
					
					if(!*stop && !skip){
						cout << "Right Turn" << endl;
						angle.push_back(1.5707);
						distance.push_back(duration_cast<milliseconds>(steady_clock::now() - distClock).count() * forward_speed / 8000);
					}
					
					robot->sendDriveDirectCommand (rotate_speed, -rotate_speed);
					clock0 = steady_clock::now();
					
					while(!*stop && !skip && duration_cast<milliseconds>(steady_clock::now() - clock0).count() < 200000/rotate_speed){
						pthread_mutex_unlock (mutex_robot);
						this_thread::sleep_for(chrono::milliseconds(5));
						pthread_mutex_lock (mutex_robot);
					}
					
					distClock = steady_clock::now();
					robot->sendDriveDirectCommand (forward_speed, forward_speed);
					clock0 = steady_clock::now();
					
					while(!*stop && !skip && duration_cast<milliseconds>(steady_clock::now() - clock0).count() < 350000/forward_speed){
						wallSignal = robot->wallSignal();
						if(duration_cast<milliseconds>(steady_clock::now() - wallClock).count() > 20){
							wallClock = steady_clock::now();
							wall2 = wall1;
							wall1 = wall0;
							wall0 = wallSignal;
						}
						if(wall0 > 5 && wall1 > 5 && wall2 > 5){
							skip = true;
							break;
						}
						if(robot->bumpRight() || robot->bumpLeft()){
							skip = true;
							break;
						}
						pthread_mutex_unlock (mutex_robot);
						this_thread::sleep_for(chrono::milliseconds(5));
						pthread_mutex_lock (mutex_robot);
					}
					skip = false;
				}
			}
		}
		
		pthread_mutex_unlock (mutex_robot);
		this_thread::sleep_for(chrono::milliseconds(5));
		pthread_mutex_lock (mutex_robot);
	}
	
	cout<<"-------------------------------Pressed the Play Button. Stop the mission!-------------------------------"<<endl<<endl;
	
	cout << duration_cast<milliseconds>(steady_clock::now() - distClock).count() << endl;
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
}
