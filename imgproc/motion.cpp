#include <stdio.h>
#include <iostream>
#include <numeric>
#include <opencv2/imgproc.hpp>
#include <opencv2/video.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/videoio.hpp>

using namespace std;

vector<cv::Point> findObjects(cv::Mat & frame, vector<vector<cv::Point>> & contours, cv::Point ref, int minarea) {
	vector<cv::Rect> boundRect(contours.size());
	vector<vector<cv::Point>> contours_poly(contours.size());
	vector<cv::Moments> m(contours.size());
	vector<cv::Point> centers(contours.size());

	cv::RNG rng(12345);
	char name[20];
	int valid_contours=0;

	for(int i=0; i<contours.size(); i++) {
		if(cv::contourArea(contours[i]) < minarea)
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
		cv::line(frame, ref, centers[i], cv::Scalar(0,200,0));
		cout << i << ": r = " << sqrt(pow(centers[i].x-ref.x, 2) + pow(centers[i].y-ref.y, 2));
	        cout << " theta = " << 180*atan(((double) centers[i].y)/centers[i].x)/3.14159265<< endl;

	}
	return centers;
}


int main(int argc, char* argv[]) {
	if(argc < 4) {
		cerr << "Uso: " << argv[0] << " id_camera sensibilidade iteracoes1 iteracoes2\n";
		cerr << "\n\tPara informacoes no significado dos parametros, consulte o README.md\n";
		return -1;
	}
	vector<vector<cv::Point>> contours;
	vector<cv::Vec4i> hierarchy;
	
	cv::Ptr<cv::BackgroundSubtractorMOG2> pBackSub;
	pBackSub = cv::createBackgroundSubtractorMOG2(100, 16, false);

	cv::VideoCapture capture(atoi(argv[1]));

	cv::Size s;

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
		cv::imshow("Plano de fundo", fgMask);
		if(cv::waitKey(5) >= 0)
			break;
	}
	s = frame.size();
	cv::Point ref((s.width-1)/2, s.height-1);
	cout << "Dimensoes: " << ref.x << " " << ref.y;
	cout << "Reconhecimento finalizado. Posicione os objetos e pressione ENTER.\n";
	cv::imshow("Plano de fundo", fgMask);
	int keyboard = cv::waitKey(0);
	if(keyboard == 'q')
		return -1;
	cv::destroyWindow("Plano de fundo");
	for(int i=0; i<atoi(argv[4]); i++) {
		capture >> frame;
		cv::imshow("what", frame);
		if(cv::waitKey(5) >= 0) {
			break;
		}

		if(frame.empty()) {
			cerr << "empty frame\n";
			return -1; //break;
		}
	}
	//cv::destroyWindow("what");
	pBackSub->apply(frame, fgMask);
	cv::namedWindow("Objetos", cv::WINDOW_NORMAL);
	cv::imshow("Objetos", fgMask);

	cv::findContours(fgMask, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE, cv::Point(0,0));

	vector<cv::Point> centers = findObjects(frame, contours, ref, atoi(argv[2]));

	cout << centers.size() << " objetos reconhecidos.\n";

	cv::imshow("Stream", frame);
	keyboard = cv::waitKey(0);
	if(keyboard == 'q' || keyboard == 27)
		return -1;

	return 0;
}
