#include "robocontour.h"

int image_height = 1200;
int image_width = 1600;


// trasform the world coordinate to image coordinate
Point2f world_2_image(Point2f point){
  int x = point.x+image_width/2;
  int y = point.y+image_height/2;
  Point2f new_point(x,y);
  return new_point;
}

Point2f move_straight(Point2f point, float dir, float dist){
  float x = dist*cos(dir)+point.x;
  float y = dist*sin(dir)+point.y;
  Point2f new_point(x,y);
  return new_point;
}

int robotcontour(vector<float> straight, vector<float> angle)
{
	string line_;

	std::vector<Point2f> waypoints;
	Point2f apoint(0,0);
	waypoints.push_back(apoint);

	Mat img_output(image_height, image_width, CV_8UC3, Scalar(255, 255, 255));
	Scalar lineColor(255, 0, 0);
	int lineWidth = 1;
	int radius = 3;

	bool drawing_done = false;
	float dir = 0;
	int num = 0;

	Point2f prev_pos(0,0);
	Point2f curr_pos;

	for (int i = 0; i < straight.size(); i++){
		Point2f curr_pos = move_straight(prev_pos, dir, straight[i]);
		line(img_output,  world_2_image(prev_pos),  world_2_image(curr_pos), lineColor, lineWidth, CV_AA);
      	circle(img_output,  world_2_image(curr_pos), radius, lineColor, CV_FILLED, CV_AA);
		if(i<straight.size()-1){
			dir = dir+angle[i];
		}
		prev_pos = curr_pos;
    }

  imwrite("irobot_contour.png", img_output);

  return 0;
}
