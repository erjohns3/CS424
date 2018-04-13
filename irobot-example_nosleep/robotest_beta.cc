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

int rot_dir = 0;
bool take_photo_done = false;

int speed = 287; // forward speed
int speed_backward =165; // backward speed
int x=0;

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

    		short wallSignal, prevWallSignal = 0;

			iRobot::Create::note_t note1(64, 4);
		   	iRobot::Create::song_t song1;
	        song1.push_back(note1);
		   	robot.sendSongCommand(0,song1);

    		bool backup = false;
			bool rotate = false;
    		int rot_ang=0;
			short wallSignal;

			steady_clock::time_point sensor_clock = steady_clock::now();
			steady_clock::time_point backup_clock;
			steady_clock::time_point rotate_clock;

    		while (!robot.playButton ())
    		{
				if (!backup && (robot.bumpLeft () || robot.bumpRight ())) {
					cout << "Bump !" << endl;
					backup = true;
					backup_clock = steady_clock::now();
					robot.sendDriveCommand(-speed_backward, Create::DRIVE_STRAIGHT);

					while(backup && duration_cast<milliseconds>(steady_clock::now() - backup_clock).count() < 2309){

					}
					robot.sendDriveCommand(0, Create::DRIVE_STRAIGHT);

					// this_thread::sleep_for(chrono::milliseconds(1500));
					Camera.grab();
					Camera.retrieve (bgr_image);
					cv::cvtColor(bgr_image, rgb_image, CV_RGB2BGR);
					cv::imwrite("irobot_image.jpg", rgb_image);
					cout << "Taking photo" << endl;

					// compute the desired rotation angle and direction
					take_photo_done = true;
					rot_ang = rand()%120+120;
					rot_dir = rand()%2;
					backup = false;
					rotate = true;

				}

				if (robot.advanceButton ())
				{
					cout << "Advance button pressed" << endl;
					speed = -1 * speed;
					ledColor += 10;
					if (ledColor > 255){
						ledColor = 0;
					}

					robot.sendDriveCommand (speed, Create::DRIVE_INPLACE_CLOCKWISE);
					if (speed < 0){
						robot.sendLedCommand (Create::LED_PLAY,  ledColor, Create::LED_INTENSITY_FULL);
					}
					else{
						robot.sendLedCommand (Create::LED_ADVANCE, ledColor, Create::LED_INTENSITY_FULL);
					}
				}
				wallSignal = robot.wallSignal();

				sensor_clock = steady_clock::now();
			}



				if(back_count==10 && take_photo_done==true && rot_count<10){
					if(rot_dir == 0){
					 	robot.sendDriveCommand(107, Create::DRIVE_INPLACE_COUNTERCLOCKWISE);
					 	this_thread::sleep_for(chrono::milliseconds(rot_ang*100/107));
				    }
				 	else{
					 	robot.sendDriveCommand(107, Create::DRIVE_INPLACE_CLOCKWISE);
					  	this_thread::sleep_for(chrono::milliseconds(rot_ang*100/107));
				 	}
				 	rot_count++;
		      	}

				// flashing LEDs
				if(backup==true && take_photo_done==false){
					if (x==0){
						robot.sendLedCommand(Create::LED_PLAY, Create::LED_COLOR_GREEN, Create::LED_INTENSITY_FULL);
					}
					else if (x==1){
						robot.sendLedCommand(Create::LED_ALL, Create::LED_COLOR_GREEN, Create::LED_INTENSITY_OFF);
					}
					else if (x==2){
						robot.sendLedCommand(Create::LED_ADVANCE, Create::LED_COLOR_RED, Create::LED_INTENSITY_FULL);
					}
					else if (x==3){
						robot.sendLedCommand(Create::LED_PLAY, Create::LED_COLOR_RED, Create::LED_INTENSITY_FULL);
					}
					else if (x==4){
						robot.sendLedCommand(Create::LED_ALL, Create::LED_COLOR_GREEN, Create::LED_INTENSITY_OFF);
					}
					else if (x==5){
						robot.sendLedCommand(Create::LED_ADVANCE, Create::LED_COLOR_GREEN, Create::LED_INTENSITY_FULL);
					}
					x=(x+1)%6;
				}


			   if(backup==false){
			  	 	robot.sendDriveCommand (speed, Create::DRIVE_STRAIGHT);
					this_thread::sleep_for(chrono::milliseconds(100));
			   }


		      if (wallSignal > 3) {
		        cout << "Wall signal " << robot.wallSignal() << endl;
				// create sound

				robot.sendPlaySongCommand(0);
				this_thread::sleep_for(chrono::milliseconds(100/wallSignal));
		      }

		      prevWallSignal = wallSignal;


		    // You can add more commands here.
    	}

  		cout << "Play button pressed, stopping Robot" << endl;
	    robot.sendDriveCommand (0, Create::DRIVE_STRAIGHT);
  	}

  	catch (InvalidArgument& e)
  	{
    	cerr << e.what () << endl;
    	return 3;
  	}
  	catch (CommandNotAvailable& e)
  	{
    	cerr << e.what () << endl;
    	return 4;
  	}
}
