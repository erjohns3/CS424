#include <SerialStream.h>
#include "irobot-create.hh"
#include <ctime>
#include <iostream>
#include <chrono>
#include <thread>
#include <raspicam/raspicam_cv.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <time.h>

using namespace iRobot;
using namespace LibSerial;
using namespace std;
using namespace std::chrono;

int main ()
{
	char serial_loc[] = "/dev/ttyUSB0";
	srand(time(NULL));

	try
	{
		raspicam::RaspiCam_Cv Camera;
		cv::Mat rgb_image, bgr_image;
		if (!Camera.open()) {
			cerr << "Error opening the camera" << endl;
			return -1;
		}
		cout << "Opened Camera" << endl;

		SerialStream stream (serial_loc, LibSerial::SerialStreamBuf::BAUD_57600);
		cout << "Opened Serial Stream to" << serial_loc << endl;
		this_thread::sleep_for(chrono::milliseconds(1000));

		Create robot(stream);
		cout << "Created iRobot Object" << endl;

		robot.sendFullCommand();
		cout << "Setting iRobot to Full Mode" << endl;
		this_thread::sleep_for(chrono::milliseconds(1000));
		cout << "Robot is ready" << endl;

		// stream some sensors.
	 	Create::sensorPackets_t sensors;
		sensors.push_back(Create::SENSOR_BUMPS_WHEELS_DROPS);
		sensors.push_back(Create::SENSOR_WALL_SIGNAL);
	    sensors.push_back (Create::SENSOR_BUTTONS);
		robot.sendStreamCommand (sensors);
		cout << "Sent Stream Command" << endl;

	    int ledColor = Create::LED_COLOR_GREEN;
	    robot.sendDriveCommand (speed, Create::DRIVE_STRAIGHT);
	    robot.sendLedCommand (Create::LED_PLAY, 0, 0);
	    cout << "Sent Drive Command" << endl;

		iRobot::Create::note_t note1(100, 4);
	   	iRobot::Create::song_t song1;
        song1.push_back(note1);
	   	robot.sendSongCommand(0,song1);

		int rot_ang = 0;
		const int speed = 287; // forward speed
		const int speed_backward = 165; // backward speed

		bool backup = false;
		bool rotate = false;
		short wallSignal = 0;
		int led_mode = 0;

		steady_clock::time_point backup_clock;
		steady_clock::time_point led_clock = steady_clock::now();
		steady_clock::time_point wall_clock = steady_clock::now();
		steady_clock::time_point sensor_clock = steady_clock::now();

		while (!robot.playButton()){

			// check for bumps and wall proximity
			if(duration_cast<milliseconds>(steady_clock::now() - sensor_clock).count() > 100){
				wallSignal = robot.wallSignal();
				if (!backup && (robot.bumpLeft() || robot.bumpRight())) {
					robot.sendDriveCommand(0, Create::DRIVE_STRAIGHT);
        			cout << "Bump !" << endl;
					backup = true;
					backup_clock = steady_clock::now();
					robot.sendDriveCommand(-speed_backward, Create::DRIVE_STRAIGHT);
      			}
				sensor_clock = steady_clock::now();
			}

			// backup when bumper goes up
			if(backup && duration_cast<milliseconds>(steady_clock::now() - backup_clock).count() > 2309){
				robot.sendDriveCommand(0, Create::DRIVE_STRAIGHT);		// stop driving
				Camera.grab();											// initiate camera
				Camera.retrieve (bgr_image);							// take photo
				cv::cvtColor(bgr_image, rgb_image, CV_RGB2BGR);
				cv::imwrite("irobot_image.jpg", rgb_image);
				cout << "Taking photo" << endl;
				// compute the desired rotation angle and direction
				rot_ang = rand()%120+120;
				backup = false;
				rotate = true;
				backup_clock = steady_clock::now();
				if(rand()%2 == 0){
				 	robot.sendDriveCommand(107, Create::DRIVE_INPLACE_COUNTERCLOCKWISE);
			    }
			 	else{
				 	robot.sendDriveCommand(107, Create::DRIVE_INPLACE_CLOCKWISE);
			 	}
			}

			// rotate after backing up
			if(rotate && duration_cast<milliseconds>(steady_clock::now() - backup_clock).count() > 3.0*rot_ang){
			 	robot.sendDriveCommand(0, Create::DRIVE_INPLACE_COUNTERCLOCKWISE);
			 	robot.sendDriveCommand(0, Create::DRIVE_INPLACE_CLOCKWISE);
				rotate = false;
	      	}

			// flash the LEDs
			if(backup && duration_cast<milliseconds>(steady_clock::now() - led_clock).count() > 200){
				if (led_mode==0){
					robot.sendLedCommand(Create::LED_PLAY, Create::LED_COLOR_GREEN, Create::LED_INTENSITY_FULL);
				}
				else if (led_mode==1){
					robot.sendLedCommand(Create::LED_ALL, Create::LED_COLOR_GREEN, Create::LED_INTENSITY_OFF);
				}
				else if (led_mode==2){
					robot.sendLedCommand(Create::LED_ADVANCE, Create::LED_COLOR_RED, Create::LED_INTENSITY_FULL);
				}
				else if (led_mode==3){
					robot.sendLedCommand(Create::LED_PLAY, Create::LED_COLOR_RED, Create::LED_INTENSITY_FULL);
				}
				else if (led_mode==4){
					robot.sendLedCommand(Create::LED_ALL, Create::LED_COLOR_GREEN, Create::LED_INTENSITY_OFF);
				}
				else if (led_mode==5){
					robot.sendLedCommand(Create::LED_ADVANCE, Create::LED_COLOR_GREEN, Create::LED_INTENSITY_FULL);
				}
				cout << "LED mode set to " << led_mode << endl;
				led_mode = (led_mode+1)%6;
				led_clock = steady_clock::now();
			}

			// play wall signal sound
			if (wallSignal >= 3 && duration_cast<milliseconds>(steady_clock::now() - wall_clock).count() > 15000 / (wallSignal + 7)) {
  		        cout << "Wall signal " << robot.wallSignal() << endl;
  				robot.sendPlaySongCommand(0);
				wall_clock = steady_clock::now();
			}
		}

		// stop the robot
  		cout << "Play button pressed, stopping Robot" << endl;
	    robot.sendDriveCommand(0, Create::DRIVE_STRAIGHT);
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
