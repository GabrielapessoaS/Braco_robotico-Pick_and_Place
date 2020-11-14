#include <stdio.h>
#include <iostream>
#include <opencv2/imgproc.hpp>
#include <opencv2/video.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/videoio.hpp>

using namespace std;

int main(int argc, char* argv[]) {
	if(argc < 4) {
		cerr << "Missing parameters\n";
		return -1;
	}
	vector<vector<cv::Point>> contours;
	vector<cv::Vec4i> hierarchy;
	
	cv::Ptr<cv::BackgroundSubtractorMOG2> pBackSub;
	pBackSub = cv::createBackgroundSubtractorMOG2(100, 16, false);

	cv::VideoCapture capture(atoi(argv[1]));

	if(!capture.isOpened()) {
		cerr << "unable to open camera\n";
		return -1;
	}
	cv::Mat frame, fgMask;
	cout << "Reconhecendo plano de fundo...\n";
	for(int i=0; i<atoi(argv[3]); i++) {
		capture >> frame;
		if(frame.empty()) {
			cerr << "empty frame\n";
			return 0;	
		}
		pBackSub->apply(frame, fgMask);
	}
	cout << "Reconhecimento finalizado. Posicione os objetos e pressione ENTER.\n";
	cv::imshow("Plano de fundo", fgMask);
	int keyboard = cv::waitKey(0);
	if(keyboard == 'q')
		return -1;
	cv::destroyWindow("Plano de fundo");
	//while(1) {
		capture >> frame;
		if(frame.empty()) {
			cerr << "empty frame\n";
			return -1; //break;
		}
		pBackSub->apply(frame, fgMask);
		cv::imshow("Objetos", fgMask);

		cv::RNG rng(12345);
		cv::findContours(fgMask, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE, cv::Point(0,0));

		vector<cv::Rect> boundRect(contours.size());
		vector<vector<cv::Point>> contours_poly(contours.size());
		vector<cv::Moments> m(contours.size());
		vector<cv::Point> centers(contours.size());

		char name[20];
		int valid_contours=0;
		for(int i=0; i<contours.size(); i++) {
			if(cv::contourArea(contours[i]) < atoi(argv[2]))
				continue;
			valid_contours++;
			cv::putText(frame, "Objetos reconhecidos", cv::Point(10, 20), cv::FONT_HERSHEY_SIMPLEX, 0.75, cv::Scalar(0, 0, 255), 2);
			cv::approxPolyDP(contours[i], contours_poly[i], 3, true);

			boundRect[i] = cv::boundingRect(contours_poly[i]);
			cv::Scalar color = cv::Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));
			cv::rectangle(frame, boundRect[i].tl(), boundRect[i].br(), color, 2);

			m[i] = cv::moments(contours[i], true);
			centers[i] = cv::Point(m[i].m10/m[i].m00, m[i].m01/m[i].m00);

			cv::circle(frame, centers[i], 3, cv::Scalar(128,0,0), -1);

			sprintf(name, "%d:(%d,%d)", valid_contours, centers[i].x, centers[i].y);
			cv::putText(frame, string(name), centers[i], cv::FONT_HERSHEY_SIMPLEX, 0.50, cv::Scalar(10, 0, 255), 2);

		}
		cout << valid_contours << " objetos reconhecidos.\n";

		cv::imshow("Stream", frame);
		keyboard = cv::waitKey(0);
		if(keyboard == 'q' || keyboard == 27)
			return -1; //break;
	//}

	return 0;
}
