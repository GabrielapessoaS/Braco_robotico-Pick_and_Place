#include <stdio.h>
#include <iostream>
#include <opencv2/imgproc.hpp>
#include <opencv2/video.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/videoio.hpp>

using namespace std;

int main(int argc, char* argv[]) {
	if(argc < 2) {
		cerr << "Missing threshold parameter\n";
		return -1;
	}
	vector<vector<cv::Point>> contours;
	vector<cv::Vec4i> hierarchy;
	
	cv::Ptr<cv::BackgroundSubtractorMOG2> pBackSub;
	pBackSub = cv::createBackgroundSubtractorMOG2(100, 16, false);

	cv::VideoCapture capture(0);

	if(!capture.isOpened()) {
		cerr << "unable to open camera\n";
		return -1;
	}

	cv::Mat frame, fgMask;
	while(1) {
		capture >> frame;
		if(frame.empty()) {
			cerr << "empty frame\n";
			break;
		}
		pBackSub->apply(frame, fgMask);
		cv::imshow("FG Mask", fgMask);

		cv::RNG rng(12345);
		cv::findContours(fgMask, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE, cv::Point(0,0));

		vector<cv::Rect> boundRect(contours.size());
		vector<vector<cv::Point>> contours_poly(contours.size());
		vector<cv::Moments> m(contours.size());
		vector<cv::Point> centers(contours.size());


		for(int i=0; i<contours.size(); i++) {
			if(cv::contourArea(contours[i]) < atoi(argv[1]))
				continue;
			cv::putText(frame, "Motion detected", cv::Point(10, 20), cv::FONT_HERSHEY_SIMPLEX, 0.75, cv::Scalar(0, 0, 255), 2);
			cv::approxPolyDP(contours[i], contours_poly[i], 3, true);
			boundRect[i] = cv::boundingRect(contours_poly[i]);
			m[i] = cv::moments(contours[i], true);
			centers[i] = cv::Point(m[i].m10/m[i].m00, m[i].m01/m[i].m00);
			cv::circle(frame, centers[i], 5, cv::Scalar(128,0,0), -1);
			cv::Scalar color = cv::Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));
			cv::rectangle(frame, boundRect[i].tl(), boundRect[i].br(), color, 2);
		}
		cv::imshow("Frame", frame);
		int keyboard = cv::waitKey(30);
		if(keyboard == 'q' || keyboard == 27)
			break;
	}

	return 0;
}
