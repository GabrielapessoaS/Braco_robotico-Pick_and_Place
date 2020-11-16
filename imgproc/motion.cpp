#include <stdio.h>
#include <iostream>
#include <numeric>
#include <thread>
#include <opencv2/imgproc.hpp>
#include <opencv2/video.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/videoio.hpp>

using namespace std;

vector<cv::Point> centers;


void findObjects(int cam, int minarea, int bgIter, int objIter) {
	vector<vector<cv::Point>> contours;
	vector<cv::Vec4i> hierarchy;
	cv::VideoCapture capture(cam);
	//cv::Point ref(refx, refy);
	
	cv::Ptr<cv::BackgroundSubtractorMOG2> pBackSub;
	pBackSub = cv::createBackgroundSubtractorMOG2(100, 16, false);
	
	cv::Mat frame, fgMask;

	cv::RNG rng(12345);
	char name[20];
	int valid_contours=0;

	if(!capture.isOpened()) {
		cerr << "unable to open camera\n";
		return;
	}
	capture >> frame;

	cv::Size s = frame.size();;
	cv::Point ref((s.width-1)/2, s.height-1);
	cout << "Dimensoes: " << ref.x << " " << ref.y << endl;
	
	cout << "Reconhecendo plano de fundo...\n";
	for(int i=0; i<bgIter; i++) {
		capture >> frame;
		if(frame.empty()) {
			cerr << "empty frame\n";
			return;
		}
		pBackSub->apply(frame, fgMask);
		cv::imshow("Plano de fundo", fgMask);
		if(cv::waitKey(5) >= 0)
			break;
	}
	
	cout << "Reconhecimento finalizado. Posicione os objetos e pressione ENTER.\n";
	cv::imshow("Plano de fundo", fgMask);
	int keyboard = cv::waitKey(0);
	if(keyboard == 'q')
		return;
	cv::destroyWindow("Plano de fundo");
	for(int i=0; i<objIter; i++) {
		capture >> frame;
		cv::imshow("what", frame);
		if(cv::waitKey(5) >= 0) {
			break;
		}

		if(frame.empty()) {
			cerr << "empty frame\n";
			return; //break;
		}
	}
	cv::destroyWindow("what");
	pBackSub->apply(frame, fgMask);
	cv::namedWindow("Objetos", cv::WINDOW_NORMAL);
	cv::imshow("Objetos", fgMask);
	if(cv::waitKey(0) == 'q')
		return;

	cv::findContours(fgMask, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE, cv::Point(0,0));

	vector<cv::Rect> boundRect(contours.size());
	vector<vector<cv::Point>> contours_poly(contours.size());
	vector<cv::Moments> m(contours.size());
	centers.resize(contours.size(), cv::Point(0,0));


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
	cv::imshow("Reconhecidos", frame);
	if(cv::waitKey(0) == 'q')
		return;
	cout << centers.size() << " objetos reconhecidos.\n";
}


int main(int argc, char* argv[]) {
	if(argc < 4) {
		cerr << "Uso: " << argv[0] << " id_camera sensibilidade iteracoes1 iteracoes2\n";
		cerr << "\n\tPara informacoes no significado dos parametros, consulte o README.md\n";
		cerr << "\tid_camera: numero do arquivo da camera no /dev (e.g. /dev/video0 --> id_camera = 0)\n";
		return -1;
	}


	thread objectRecognition(findObjects, atoi(argv[1]), atoi(argv[2]), atoi(argv[3]), atoi(argv[4]));

	objectRecognition.join();

	return 0;
}
