all:
	g++ -std=c++11 `pkg-config --cflags opencv` -o robomain robomain.cc robosafety.cpp robomotion.cpp robovision.cpp robocontour.cpp irobot-create.cc -pthread -L/opt/vc/lib -lserial -lraspicam -lraspicam_cv -lmmal -lmmal_core -lmmal_util `pkg-config --libs opencv` -lopencv_core -lopencv_imgcodecs -lopencv_highgui -lopencv_imgproc


clean:
	rm robomain
