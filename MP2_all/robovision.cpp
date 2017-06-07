#include "robovision.h"

void cropBottom(Mat& img_scene_full, Mat& img_scene, float crop_fraction) {
	// Crop the lower part of the scene
	cv::Rect crop;
	crop.x = 0;
	crop.y = 0;
	crop.width = img_scene_full.size().width;
	crop.height = img_scene_full.size().height * crop_fraction;
	img_scene = img_scene_full(crop);
}


bool alignPerspective(vector<Point2f>& query, vector<Point2f>& scene,
		Mat& img_query, Mat& img_scene, vector<Point2f>& scene_corners) {
	Mat H = findHomography(query, scene, RANSAC);
	if (H.rows == 0 && H.cols == 0) {
		//cout << "Failed rule0: Empty homography" << endl;
		return false;
	}

	vector<Point2f> query_corners(4);
	query_corners[0] = cvPoint(0,0);
	query_corners[1] = cvPoint(img_query.cols, 0);
	query_corners[2] = cvPoint(img_query.cols, img_query.rows);
	query_corners[3] = cvPoint(0, img_query.rows );

	perspectiveTransform(query_corners, scene_corners, H);

	float min_area = 32.0 * 32.0;
	double max_area = img_scene.rows * img_scene.cols;
	float ratio_inside = 0.75;
	float min_angle_sin =	0.173; // Minimum 10 degree angle required

	// Geometric verification heuristics
	// Rule 1: Must be a convex hull.
	// Rule 2: Area can’t be less than 32x32
	// Rule 3: The detected projection can’t have more than 100% area
	// Rule 4: Projection can't contain very small angle < 10 degree
	// Rule 5: More than 75% of the area of the detected projection should have
	// to be within image bounds

	// Rule 1: Must be a convex hull.
	vector<Point2f> sc_vec(4);
	// Generate 4 vectors from the 4 scene corners
	for(int i = 0; i < 4; i++) {
		sc_vec[i] = scene_corners[(i + 1) % 4] - scene_corners[i];
	}
	vector<float> sc_cross(4);
	// Calculate cross product of pairwise vectors
	for(int i = 0; i < 4; i++) {
		sc_cross[i] = sc_vec[i].cross(sc_vec[(i+1) % 4]);
	}

	// Check for convex hull
	if (!(sc_cross[0] < 0 && sc_cross[1] < 0 && sc_cross[2] < 0 && sc_cross[3] < 0)
			&& !(sc_cross[0] > 0 && sc_cross[1] > 0 && sc_cross[2] > 0 && sc_cross[3] > 0)) {
		//cout << "Failed rule1: Not a convex hull" << endl;
		return false;
	}

	// Rule 2: Area can’t be less than 32x32
	// Rule 3: The detected projection can’t have more than 100% area
	float area = (sc_cross[0] + sc_cross[2]) / 2.0;
	if (fabs(area) < min_area) {
		//cout << "Failed rule2: Projection too small" << endl;
		return false;
	} else if (fabs(area) > max_area) {
		//cout << "Failed rule3: Projection too large" << endl;
		return false;
	}

	// Rule 4: Can't contain very small angle < 10 degree inside projection.
	// Check for angles
	vector<float> sc_norm(4);
	for (int i = 0; i < 4; i++) {
		sc_norm[i] = norm(sc_vec[i]);
	}
	for (int i = 0; i < 4; i++) {
		float sint = sc_cross[i] / (sc_norm[i] * sc_norm[(i + 1) % 4]);
		if (fabs(sint) < min_angle_sin) {
			//cout << "Failed rule4: Contains very small angle" << endl;
			return false;
		}
	}

	// Rule 5: More than 75% of the area of the detected projection should
	// have to be within image bounds.
	// Approximate mechanism by determining the bounding rectangle.
	cv::Rect bound = boundingRect(scene_corners);
	cv::Rect scene_rect(0.0, 0.0, img_scene.cols, img_scene.rows);
	cv::Rect isect = bound & scene_rect;
	if (isect.width * isect.height <	ratio_inside * bound.width * bound.height ) {
		//cout << "Failed rule5: Large proportion outside scene" << endl;
		return false;
	}
	return true;
}

// Show the projection
void drawProjection(Mat& img_matches, Mat& img_query,
		vector<Point2f>& scene_corners) {
	line(img_matches, scene_corners[0] + Point2f(img_query.cols, 0),
			scene_corners[1] + Point2f(img_query.cols, 0), Scalar(0, 255, 0), 4);
	line(img_matches, scene_corners[1] + Point2f(img_query.cols, 0),
			scene_corners[2] + Point2f(img_query.cols, 0), Scalar(0, 255, 0), 4);
	line(img_matches, scene_corners[2] + Point2f(img_query.cols, 0),
			scene_corners[3] + Point2f(img_query.cols, 0), Scalar(0, 255, 0), 4);
	line(img_matches, scene_corners[3] + Point2f(img_query.cols, 0),
			scene_corners[0] + Point2f(img_query.cols, 0), Scalar(0, 255, 0), 4);
}

string type2str(int type) {
	std::string r;
	uchar depth = type & CV_MAT_DEPTH_MASK;
	uchar chans = 1 + (type >> CV_CN_SHIFT);

	switch (depth) {
		case CV_8U:	r = "8U"; break;
		case CV_8S:	r = "8S"; break;
		case CV_16U: r = "16U"; break;
		case CV_16S: r = "16S"; break;
		case CV_32S: r = "32S"; break;
		case CV_32F: r = "32F"; break;
		case CV_64F: r = "64F"; break;
		default:		 r = "User"; break;
	}

	r += "C";
	r += (chans + '0');
	return r;
}

vector<Mat> images;
vector<String> result(11);
vector<Mat> img_query(10);
vector<vector<KeyPoint>> keypoints_query(10);
vector<Mat> descriptors_query(10);

void extract_target_features(void){
	auto sttime = steady_clock::now();
	
	vector<String> objs(10);
	objs[0] = "query-image/low-resolution/ancient-lamp-600.jpg";
  	objs[1] = "query-image/low-resolution/audio-cassette-600.jpg";
  	objs[2] = "query-image/low-resolution/magic-lamp-600.jpg";
  	objs[3] = "query-image/low-resolution/mammoth-600.jpg";
  	objs[4] = "query-image/low-resolution/mayan-calendar-600.jpg";
  	objs[5] = "query-image/low-resolution/mjolnir-hammer-600.jpg";
  	objs[6] = "query-image/low-resolution/one-ring-600.jpg";
  	objs[7] = "query-image/low-resolution/pueblo-pot-600.jpg";
  	objs[8] = "query-image/low-resolution/roman-glass-600.jpg";
  	objs[9] = "query-image/low-resolution/willow-plate-600.jpg";

  	result[0] = "****** find ancient lamp in the scene !";
  	result[1] = "****** find audio cassette in the scene !";
  	result[2] = "****** find magic lamp in the scene !";
  	result[3] = "****** find mammoth in the scene !";
  	result[4] = "****** find mayan calendar in the scene !";
  	result[5] = "****** find mjolnir hammer in the scene !";
  	result[6] = "****** find one ring in the scene !";
  	result[7] = "****** find pueblo pot in the scene !";
  	result[8] = "****** find roman glass in the scene !";
  	result[9] = "****** find willow plate in the scene !";
  	result[10] = "###### nothing is found in the scene ... keep searching ...";
	
	int num = 10;
  	for(int i = 0; i < num; i++){
    	img_query[i] = imread(objs[i], IMREAD_GRAYSCALE);
  	}

  	// Detect the keypoints and extract descriptors using SURF
  	// Surf keypoint detector and descriptor.
  	int minHessian = 100;
  	int nOctaves = 4;
  	int nOctaveLayers = 3;
  	Ptr<SURF> detector = SURF::create(minHessian, nOctaves, nOctaveLayers, true);

  	for (int i = 0; i<num; i++){
    	detector->detectAndCompute(img_query[i], Mat(), keypoints_query[i], descriptors_query[i]);
  	}
  	
  	cout << "Feature extraction for "<<num<<" query images take " << (duration <double>(steady_clock::now() - sttime)).count() << " sec" << endl;
}


int img_proc(Create* robot, pthread_mutex_t* mutex_robot, pthread_mutex_t* mutex_images, bool* stop, int x) {
	Mat img_scene;
	vector<KeyPoint> keypoints_scene;
	Mat descriptors_scene;

	int found_obj = false;
	int num = 0;
	bool res = false;
	
	int minHessian = 100;
  	int nOctaves = 4;
  	int nOctaveLayers = 3;
  	Ptr<SURF> detector = SURF::create(minHessian, nOctaves, nOctaveLayers, true);
	
	//vector<vector<DMatch>> matches;
	//vector<DMatch> good_matches;
	//vector<Point2f> query;
	//vector<Point2f> scene;
	//vector<Point2f> scene_corners(4);
	//BFMatcher matcher(NORM_L2);
	
	while(!*stop  || num != images.size() ){
    	auto sttime = steady_clock::now();
    	
		pthread_mutex_lock (mutex_images);
		img_scene = images[num];
		pthread_mutex_unlock (mutex_images);
		num++;
		
   		if(!img_scene.data) {
        	cout<< "Error reading images" << endl;
        	return -1;
    	}

    	detector->detectAndCompute(img_scene, Mat(), keypoints_scene, descriptors_scene);
	    found_obj = false;
		
		for(int num_obj = 0; num_obj < 10; num_obj++){
	    	// Matching descriptor vectors using Brute Force matcher
      		BFMatcher matcher(NORM_L2);
      		vector<vector<DMatch>> matches;
      		//matches.clear();
      		matcher.knnMatch(descriptors_query[num_obj], descriptors_scene, matches, 2);
			
			//good_matches.clear();
      		vector<DMatch> good_matches;
      		for(int i = 0; i < descriptors_query[num_obj].rows; i++) {
        		if (matches[i][0].distance < 0.75 * matches[i][1].distance)
        		  good_matches.push_back(matches[i][0]);
      		}

      		// Find the location of the query in the scene
      		vector<Point2f> query;
      		vector<Point2f> scene;
      		//query.clear();
      		//scene.clear();
      		
      		for(size_t i = 0; i < good_matches.size(); i++) {
        		query.push_back(keypoints_query[num_obj][good_matches[i].queryIdx].pt);
        		scene.push_back(keypoints_scene[good_matches[i].trainIdx].pt);
      		}

      		vector<Point2f> scene_corners(4);
      		bool res = alignPerspective(query, scene, img_query[num_obj], img_scene, scene_corners);
      		
      		if (res) {
        		cout << result[num_obj] << endl;
        		found_obj = true;
      		}
		}

    	if (found_obj == false){
      		cout << result[10] << endl;
    	}
    	cout << "Image processing loop takes "<< (duration <double>(steady_clock::now() - sttime)).count()<< " sec." << "  Current process: "<<num<<" / "<<images.size()<<endl << endl;
	}
	
	return 0;
}




int robotcamera(Create* robot, pthread_mutex_t* mutex_robot, pthread_mutex_t* mutex_images, bool* stop){
	raspicam::RaspiCam_Cv Camera;
	
	if (!Camera.open()) {
		cerr << "Error opening the camera" << endl;
		return -1;
	}
	cout<<"Camera Opened"<<endl;
	Mat img_scene, bgr_image, img_scene_full;
	
	float keep_top_fraction = 0.85;
	
	pthread_mutex_lock (mutex_robot);
	
	int counter = 0;
	
	while(!*stop){
		pthread_mutex_unlock (mutex_robot);
	
		Camera.grab();
		Camera.retrieve(bgr_image);
		if (counter>0){
			cvtColor(bgr_image, img_scene_full, CV_BGR2GRAY);
		
			if(!img_scene_full.data) {
				cout<< "Error reading images" << endl;
				return -1;
			}
		
			cropBottom(img_scene_full, img_scene, keep_top_fraction);
			resize(img_scene, img_scene, Size(), 0.4, 0.4);
		
			string a( "/home/pi/MP2_all/camera_images/image");
			string c(".jpg");
			char b[20];
			sprintf(b, "%d", images.size()+1);
			string d = a+b+c;
		
			imwrite(d, img_scene);
		
			pthread_mutex_lock (mutex_images);
			images.push_back(img_scene);
			pthread_mutex_unlock (mutex_images);
		
			this_thread::sleep_for(chrono::milliseconds(3000));
		}
		pthread_mutex_lock (mutex_robot);
		counter++;
		//cout<<"how many images? "<<images.size()<<endl;
	}                                                                               
	pthread_mutex_unlock (mutex_images);
	pthread_mutex_unlock (mutex_robot);
	return 0;
}







int robotvision(Create* robot, pthread_mutex_t* mutex_robot, pthread_mutex_t* mutex_images, bool* stop, int x){
//preprocessing of query images
	vector<String> objs(10);
	objs[0] = "query-image/low-resolution/ancient-lamp-600.jpg";
	objs[1] = "query-image/low-resolution/audio-cassette-600.jpg";
	objs[2] = "query-image/low-resolution/magic-lamp-600.jpg";
	objs[3] = "query-image/low-resolution/mammoth-600.jpg";
	objs[4] = "query-image/low-resolution/mayan-calendar-600.jpg";
	objs[5] = "query-image/low-resolution/mjolnir-hammer-600.jpg";
	objs[6] = "query-image/low-resolution/one-ring-600.jpg";
	objs[7] = "query-image/low-resolution/pueblo-pot-600.jpg";
	objs[8] = "query-image/low-resolution/roman-glass-600.jpg";
	objs[9] = "query-image/low-resolution/willow-plate-600.jpg";

	vector<String> result(11);
	result[0] = "find ancient lamp in the scene !";
	result[1] = "find audio cassette in the scene !";
	result[2] = "find magic lamp in the scene !";
	result[3] = "find mammoth in the scene !";
	result[4] = "find mayan calendar in the scene !";
	result[5] = "find mjolnir hammer in the scene !";
	result[6] = "find one ring lamp in the scene !";
	result[7] = "find pueblo pot lamp in the scene !";
	result[8] = "find roman glass in the scene !";
	result[9] = "find willow plate in the scene !";
	result[10] = "nothing is found in the scene ... keep searching ...";

	Mat img_query = imread(objs[x], IMREAD_GRAYSCALE);

	// Detect the keypoints and extract descriptors using SURF
	// Surf keypoint detector and descriptor.
	int minHessian = 100;
	int nOctaves = 4;
	int nOctaveLayers = 3;
	Ptr<SURF> detector = SURF::create(minHessian, nOctaves, nOctaveLayers, true);

	vector<KeyPoint> keypoints_query;
	Mat descriptors_query;

	auto sttime = steady_clock::now();
	detector->detectAndCompute(img_query, Mat(), keypoints_query, descriptors_query);

	Mat descriptors_scene;
	vector<KeyPoint> keypoints_scene;

	vector<vector<DMatch>> matches;
	vector<DMatch> good_matches;
	vector<Point2f> query;
	vector<Point2f> scene;
	Mat img_matches;
	vector<Point2f> scene_corners(4);
	Mat img_scene;
	bool image_found = false;
	int num = 0;
	//cout<< "vision 0" << endl;
	pthread_mutex_lock (mutex_robot);
	pthread_mutex_lock (mutex_images);
	
	
	while((!*stop || num != images.size()) && !image_found){
		
		pthread_mutex_unlock (mutex_robot);
		sttime = steady_clock::now();
		
		if(num < images.size()){
			img_scene = images[num];
			num++;
			cout<<"vision thread "<<x<<": processing "<<num<<"  /  "<<images.size()<<endl;
			pthread_mutex_unlock (mutex_images);
			descriptors_scene.release();
			detector->detectAndCompute(img_scene, Mat(), keypoints_scene, descriptors_scene);
			
			if(!descriptors_scene.empty()){
				
				// Matching descriptor vectors using Brute Force matcher
				BFMatcher matcher(NORM_L2);
				matches.clear();
				matcher.knnMatch(descriptors_query, descriptors_scene, matches, 2);
				good_matches.clear();
				for(int i = 0; i < descriptors_query.rows; i++) {
					if (matches[i][0].distance < 0.75 * matches[i][1].distance)
						good_matches.push_back(matches[i][0]);
				}
		
				// Find the location of the query in the scene
				query.clear();
				scene.clear();
				for(size_t i = 0; i < good_matches.size(); i++) {
					query.push_back(keypoints_query[good_matches[i].queryIdx].pt);
					scene.push_back(keypoints_scene[good_matches[i].trainIdx].pt);
				}

				scene_corners.clear();
				scene_corners.resize(4);
				bool res = false; 
				//if(query.size()>5 && scene.size()>5){
					//cout<<"query.size() = "<<query.size()<<" scene.size() = "<<scene.size()<<endl;
				 	res = alignPerspective(query, scene, img_query, img_scene, scene_corners);
				//}

				// Write output to file

				//Mat img_matches;
				/*
				drawMatches(img_query, keypoints_query, img_scene, keypoints_scene,
						good_matches, img_matches, Scalar::all(-1), Scalar::all(-1),
						vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
				*/

				// Fill the extra area in almost white (Saves ink when printing)
				/*
				if (img_query.rows < img_scene.rows) {
					rectangle(img_matches, Point2f(0, img_query.rows),
							Point2f(img_query.cols - 1, img_scene.rows - 1),
							Scalar(255, 240, 240), CV_FILLED);
				} else if (img_scene.rows < img_query.rows) {
					rectangle(img_matches, Point2f(img_query.cols, img_scene.rows),
							Point2f(img_query.cols + img_scene.cols - 1, img_query.rows - 1),
							Scalar(255, 240, 240), CV_FILLED);
				}
				*/
				if (res) {
					cout << result[x] << endl;
					image_found = true;
					//drawProjection(img_matches, img_query, scene_corners);
				}
			
				// Write result to a file
				//cv::imwrite(output_file, img_matches);
			}
		}else{
			pthread_mutex_unlock (mutex_images);
		}

		//cout << "image " << x << " preprocessing time " << (duration <double>(steady_clock::now() - sttime)).count() << " sec" << endl << endl;

		//this_thread::sleep_for(chrono::milliseconds(100));
		
		pthread_mutex_lock (mutex_robot);
		pthread_mutex_lock (mutex_images);
		//imshow("ShowImage1",img_scene_full);
		
		//}
		//else{
			//pthread_mutex_unlock (mutex_images);
		//}
	}
	pthread_mutex_unlock (mutex_images);
	pthread_mutex_unlock (mutex_robot);
	return 0;
}
