// Brief Sample of using OpenCV dnn module in real time with device capture, video and image.
// VIDEO DEMO: https://www.youtube.com/watch?v=NHtRlndE2cg
#include <opencv2/dnn.hpp>
#include <opencv2/dnn/shape_utils.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <fstream>
#include <iostream>
#include <algorithm>
#include <cstdlib>
#include "SerialPort.h"
#include <windows.h>

using namespace std;
using namespace cv;

static const char* about =
"This program uses You only look once (YOLO)-Detector (https://arxiv.org/abs/1612.08242) to detect objects on camera/video/image.\n"
"Models can be downloaded here: https://pjreddie.com/darknet/yolo/\n"
"Default network is 416x416.\n"
"Class names can be downloaded here: https://github.com/pjreddie/darknet/tree/master/data\n";

static const char* params =
"{ help           | false | print usage         }"
"{ cfg            |       | model configuration }"
"{ model          |       | model weights       }"
"{ source         |       | video or image for detection}"
"{ min_confidence | 0.24  | min confidence      }"
"{ class_names    |       | File with class names, [PATH-TO-DARKNET]/data/coco.names }";



SerialPort serial;

struct DetectedObject {
	DetectedObject(std::string& typeClass, float w, float d, float confidence)
		: typeClass(typeClass)
		, w(w)
		, d(d)
		, confidence(confidence)
	{}

	std::string typeClass;
	float w;  // w from the center
	float d;  // distance
	float confidence;
};

void setThrottle(int16_t l, int16_t r) {
	int16_t v[2] = { l, r };
	serial.WriteData((char*)v, 4);
}

void setValues(int16_t l, int16_t r) {
	int16_t v[2] = { l, r };
	serial.WriteData((char*)v, 4);
}

int robotAction = 0; // 0: stop, 1: forward, 2: rotate
void robotStop() { setValues(0, 0); robotAction = 0;  } // stop the motors
void robotForward(float distance) { setValues(1, distance * 32767 / 100.0); robotAction = 1;  } // 32767 means 100 meters [0, 100]
void robotRotate(float angle) { setValues(2, angle * 32767 / 180.0); robotAction = 2;  } // 32767 means 180 degrees [=180, 180]

// Convert screen (x,y) coordinates in pixels to (w, d) in meters
void calculateDistance(float xc, float y, float W, float H, float &w, float &d) {
	float h = 67.9; // [cm] camera height
	float d0 = 59.5; // [cm] minimum distance 
	float d1 = 213.5; // [cm] distance to center of the image
	float th0 = atan2(d0, h);
	float th1 = atan2(d1, h);
	float x = H / (2 * tan(th1 - th0));
	float th = th1 - atan2(H / 2 - y, x);
	d = h * tan(th);
	// I'm not sure if the horizontal distance is ok
	float x1 = H / 4;
	float w1 = 81.5; // distance to the center left quarter
	float f = (x1) * d1 / w1;
	w = (xc - W / 2) / f * d;
	d /= 100; // convert to meters
	w /= 100; // convert to meters
}

bool isPersonInDanger(std::vector<DetectedObject>& objects, float wa, float da) {
	for (int i = 0; i < objects.size(); ++i) {
		if (objects[i].typeClass == "person" && objects[i].d < da && abs(objects[i].w) < wa)
			return true;
	}
	return false;
}

void drawObjects3D(Mat objectMap, std::vector<DetectedObject>& objects) {
	float W = objectMap.cols;
	float H = objectMap.rows;
	float Wmeters = 2.5;
	float Hmeters = 5.0;
	// Radar in polar form
	for (int i = 0; i < 12; ++i) {
		if (i % 2 == 0) 
		  cv::circle(objectMap, Size(W / 2, H), i /2.0 * H / Hmeters, Scalar(0, 255, 0), 1);
		else
		  cv::circle(objectMap, Size(W / 2, H), i /2.0* H / Hmeters, Scalar(0, 100, 0), 1);
	}
	float R = 2 * W;
	int NAngles = 13;
	for (int i = 0; i < NAngles; ++i) {
		float rx = W / 2 + R * cos(i * 3.1415 / (NAngles -1));
		float ry = H - R * sin(i * 3.1415 / (NAngles - 1));
		if (i % 2 ==0)
		  cv::line(objectMap, Size(W / 2, H), Size(rx, ry), Scalar(0, 255, 0));
		else
		  cv::line(objectMap, Size(W / 2, H), Size(rx, ry), Scalar(0, 100, 0));
	}
	// Draw the objects, people are more important than balls
	for (int i = 0; i < objects.size(); ++i) {
		DetectedObject obj = objects[i];
		if (obj.typeClass == "sports ball" || obj.typeClass == "orange" || obj.typeClass == "apple")
			cv::circle(objectMap, Size(W/2 + obj.w * W/2 / Wmeters, H - obj.d * H / Hmeters), 0.125 * H / Hmeters, Scalar(0, 255, 0), -1);
		if (obj.typeClass == "person")
			cv::circle(objectMap, Size(W/2 + obj.w * W/2 / Wmeters, H - obj.d * H / Hmeters), 0.25 * H / Hmeters, Scalar(0, 0, 255), -1);
	}
}

int main(int argc, char** argv)
{
	//serial.connect("\\\\.\\COM7");
	//if (!serial.IsConnected()) {
	//	std::cout << "Error Connecting bluethoot" << std::endl;
	//	return -1;
	//}
    CommandLineParser parser(argc, argv, params);
    
	if (parser.get<bool>("help")) {
        cout << about << endl;
        parser.printMessage();
        return 0;
    }

    String modelConfiguration = parser.get<String>("cfg");
    String modelBinary = parser.get<String>("model");
    dnn::Net net = dnn::readNetFromDarknet(modelConfiguration, modelBinary);
    
	if (net.empty()) {
        cerr << "Can't load network by using the following files: " << endl;
        cerr << "cfg-file:     " << modelConfiguration << endl;
        cerr << "weights-file: " << modelBinary << endl;
        cerr << "Models can be downloaded here:" << endl;
        cerr << "https://pjreddie.com/darknet/yolo/" << endl;
        exit(-1);
    }


    VideoCapture cap(0);			//gets video and creates the file to record it
    if(!cap.isOpened()) {
        cout << "Couldn't find camera: " << 1 << endl;
        return -1;
    }
	int frame_width = cap.get(CV_CAP_PROP_FRAME_WIDTH);
	int frame_height = 2 * cap.get(CV_CAP_PROP_FRAME_HEIGHT);
	VideoWriter video("out.avi", CV_FOURCC('M', 'J', 'P', 'G'), 10, Size(frame_height, frame_width), true);


    vector<string> classNamesVec;
    ifstream classNamesFile(parser.get<String>("class_names").c_str());
    
	if (classNamesFile.is_open()) {
        string className = "";
        while (std::getline(classNamesFile, className))
            classNamesVec.push_back(className);
    }
    
	
	//System.Threading.Thread.Sleep(2000);

	while(true) {
		
		Mat frame;
        cap >> frame; // get a new frame from camera/video or read image
		cv::rotate(frame, frame, ROTATE_90_CLOCKWISE);
		Mat objMap(Size(700, 700), CV_8UC3, Scalar(0, 0, 0));

		Mat inputBlob = dnn::blobFromImage(frame, 1 / 255.F, Size(416, 416), Scalar(), true, false); //Convert Mat to batch of images
		net.setPreferableTarget(dnn::DNN_TARGET_OPENCL);
		net.setInput(inputBlob, "data");                   //set the network input
		Mat detectionMat = net.forward("detection_out");   //compute output
        
		vector<double> layersTimings;					//performance stuff 
        double freq = getTickFrequency() / 1000;		
        double time = net.getPerfProfile(layersTimings) / freq;
		ostringstream ss;
        ss << "FPS: " << 1000/time << " ; time: " << time << " ms";
        putText(frame, ss.str(), Point(20,20), 0, 0.5, Scalar(0,0,255));
        
		float confidenceThreshold = parser.get<float>("min_confidence");
		
		//creats list where all detected objects will be recorded
		std::vector<DetectedObject> objects;

		// I got parts following code from the OpenCV yolo demo
		// It calculates the detection confidence
		for (int i = 0; i < detectionMat.rows; i++) {
            const int probability_index = 5;
            const int probability_size = detectionMat.cols - probability_index;
            float *prob_array_ptr = &detectionMat.at<float>(i, probability_index);
            size_t objectClass = max_element(prob_array_ptr, prob_array_ptr + probability_size) - prob_array_ptr;
            float confidence = detectionMat.at<float>(i, (int)objectClass + probability_index);
			// By default confidenceThreshold is 24%, but it can be set by command line
            if (confidence > confidenceThreshold)
            {
                float x = detectionMat.at<float>(i, 0);
                float y = detectionMat.at<float>(i, 1);
                float width = detectionMat.at<float>(i, 2);
                float height = detectionMat.at<float>(i, 3);
                int xLeftBottom = static_cast<int>((x - width / 2) * frame.cols);
                int yLeftBottom = static_cast<int>((y - height / 2) * frame.rows);
                int xRightTop = static_cast<int>((x + width / 2) * frame.cols);
                int yRightTop = static_cast<int>((y + height / 2) * frame.rows);

                Rect object(xLeftBottom, yLeftBottom,
                            xRightTop - xLeftBottom,
                            yRightTop - yLeftBottom);
                rectangle(frame, object, Scalar(0, 255, 0));
				
                if (objectClass < classNamesVec.size())
                {
                    ss.str("");
                    ss << confidence;
                    String conf(ss.str());
                    String label = String(classNamesVec[objectClass]) + ": " + conf;

                    int baseLine = 0;
                    Size labelSize = getTextSize(label, FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseLine);
                    rectangle(frame, Rect(Point(xLeftBottom, yLeftBottom ), Size(labelSize.width, labelSize.height + baseLine)), Scalar(255, 255, 255), CV_FILLED);
                    putText(frame, label, Point(xLeftBottom, yLeftBottom+labelSize.height), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0,0,0));

					//for the given object the distance and with are calculated and the algorithm is called
					float d, w;
					calculateDistance((xLeftBottom + xRightTop) / 2.0, frame.rows - yRightTop, frame.cols, frame.rows, w, d);
					DetectedObject obj(classNamesVec[objectClass], w, d, confidence);
					objects.push_back(obj);
                }
            }
        }

		Mat videoMap(Size(frame.cols, frame.rows), CV_8UC3, Scalar(0, 0, 0));
		drawObjects3D(videoMap, objects);
		drawObjects3D(objMap, objects);

		Mat merge(Size(2 * frame.cols, frame.rows), CV_8UC3, Scalar(0, 0, 0));
		Mat dst_roi = merge(Rect(0, 0, frame.cols, frame.rows));
		frame.copyTo(dst_roi);
		dst_roi = merge(Rect(videoMap.cols, 0, videoMap.cols, videoMap.rows));
		videoMap.copyTo(dst_roi);

		// Navigation algorithm
		// Get the nearest ball
		float minDistance = 100000;
		int imin = -1;
		for (int i = 0; i < objects.size(); ++i) {
			if (objects[i].typeClass == "sports ball") {
				float d = objects[i].d;
				float w = objects[i].w;
				float distance = sqrt(d * d + w * w);
				if (distance < minDistance) {
					minDistance = distance;
					imin = i;
				}
			}
		}

		// do something only if there is a ball
		if (imin > 0) {
			DetectedObject ball = objects[imin];
			float angle = atan2(ball.w, ball.d) * 180 / 3.1415;
			if (fabs(angle) > 10 && robotAction != 2) {
				robotRotate(angle);
			}
			else if(robotAction != 2){
				if (isPersonInDanger(objects, 1, 1)) {
					robotStop();
				}
				else {
					if(robotAction != 1)
						robotForward(ball.d);
				}
			}
		}
		else {
			if (robotAction != 2 && robotAction != 1) {
				robotRotate (45);
			}
		}

		//robotForward(2);
		 


		cv::line(frame, Size(0, frame.rows / 2), Size(frame.cols, frame.rows / 2), Scalar(0, 0, 255));
		cv::line(frame, Size(frame.cols / 2, 0), Size(frame.cols / 2, frame.rows), Scalar(0, 0, 255));
		cv::line(frame, Size(3*frame.cols / 4, 0), Size(3* frame.cols / 4, frame.rows), Scalar(0, 255, 0));


		//imshow("BEV", objMap);
		imshow("Object Tracking", merge);
		video.write(merge);
		//imshow("Video Map", videoMap);
		//imshow("YOLO: Detections", frame);
        if (waitKey(1) >= 0) break;
    
	}
    return 0;
} // main
