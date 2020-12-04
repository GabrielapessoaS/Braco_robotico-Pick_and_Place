#include <stdio.h>
#include <pigpio.h>
#include <stdlib.h>
#include <signal.h>
#include <iostream>
#include <numeric>
#include <thread>
#include <opencv2/imgproc.hpp>
#include <opencv2/video.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/videoio.hpp>

// Constantes de controle dos servos e cinematica inversa

//#define BUT_SEL	17
//#define BUT_INC	27
//#define BUT_DEC	22
//#define BOBINA	4

#define SERVO_BASE	27
#define SERVO_Y		17
#define SERVO_X		4
//#define BOBINA	6 

#define MIN_BASE	500
#define MAX_BASE	2500

#define MIN_X	1000
#define MAX_X	1530

#define MIN_Y	1000
#define MAX_Y	2500

#define SPEED 1

const double a1 = 8.0;
const double a2 = 8.0;

#define MAX_LEN	a1+a2
#define MIN_LEN 1

// Constantes de processamento de imagens
#define PERIM_CALIB 20.0 // Definir com ponto flutuante

using namespace std;

vector<cv::Point2d> centerscm;
double proportion = -1;

int sv=0;
int run=1;
int usbase = 0, usx = 0, usy = 0;

void stop(int signum){
  run = 0;
}

int gpioServoBound(int servo, int us) {
	switch(servo) {
		case SERVO_BASE:
			if( us < MIN_BASE ) us = MIN_BASE;
			if( us > MAX_BASE ) us = MAX_BASE;
			break;
		case SERVO_Y:
			if( us < MIN_Y ) us = MIN_Y;
			if( us > MAX_Y ) us = MAX_Y;
			break;
		case SERVO_X:
			if( us < MIN_X ) us = MIN_X;
			if( us > MAX_X ) us = MAX_X;
			break;
	}
	//printf("bound: %u us\n", us);
	return gpioServo(servo, us);
}
			


int degree_to_us(double degree, int servo){
	switch(servo){
		case SERVO_BASE:
			//k=((int)(-15.55*(dg_k- *degree) + 1900.0));
			//return ((int)(-11.11*(*degree) + 1500.0));
			return (int)(9.14*degree );
		case SERVO_X:
			//return ((int)(11.11*(*degree)+ 500.0));
			return ((int)(9.14*(180-degree) + 547.4));
		case SERVO_Y:
			//return ((int)(-15.55*(*degree - dg_j) + 1900.0));
			return ((int)(9.14*degree + 1950.0));
	}
	return 0;
}

void ease_func(){
  int pulse_base= gpioGetServoPulsewidth(SERVO_BASE);
  int pulse_x= gpioGetServoPulsewidth(SERVO_X);
  int pulse_y= gpioGetServoPulsewidth(SERVO_Y);

  //printf("base: %d -> %d\n", pulse_base, usbase);
  //printf("x: %d -> %d\n", pulse_x, usx);
  //printf("y: %d -> %d\n", pulse_y, usy);

  if( pulse_base < usbase)
	  gpioServoBound(SERVO_BASE, pulse_base + SPEED);
  else if( pulse_base > usbase)
	  gpioServoBound(SERVO_BASE, pulse_base - SPEED);

  if( pulse_x < usx)
	  gpioServoBound(SERVO_X, pulse_x + SPEED);
  else if( pulse_x > usx)
	  gpioServoBound(SERVO_X, pulse_x - SPEED);

  if( pulse_y < usy)
	  gpioServoBound(SERVO_Y, pulse_y + SPEED);
  else if( pulse_y > usy)
	  gpioServoBound(SERVO_Y, pulse_y - SPEED);
}


void inverse_kinematics(double x, double y, double theta1, double theta2){

  printf("Cinematica inversa para (%lf, %lf)...\n", x, y);

  if((sqrt(x*x + y*y) > MAX_LEN) || (sqrt(x*x + y*y) < MIN_LEN)) {
	  printf("Ponto alem do alcance.\n");
	  return;
  }
  theta1 = atan(y/x) + acos((x*x + y*y + a1*a1 - a2*a2)/(2*a1*sqrt(x*x + y*y)));
  theta2 = theta1 - acos((x*x + y*y - a1*a1 - a2*a2) / (2.0*a1*a2));

  theta1 = 180.0*theta1/M_PI;
  theta2 = 180.0*theta2/M_PI;

  printf("theta1 = %lf\n", theta1);
  printf("theta2 = %lf\n", theta2);


  usx = degree_to_us(theta1, SERVO_X);
  usy = degree_to_us(theta2, SERVO_Y);
}

int main(int argc, char **argv){
}



void findObjects(bool calibrate, int cam, int minarea, int bgIter, int objIter) {
	centerscm.clear();
	vector<cv::Point> centers;
	vector<vector<cv::Point>> contours;
	vector<cv::Vec4i> hierarchy;
	cv::VideoCapture capture(cam);
	//cv::Point ref(refx, refy);
	
	cv::Ptr<cv::BackgroundSubtractorMOG2> pBackSub;
	pBackSub = cv::createBackgroundSubtractorMOG2(100, 16, false);
	
	cv::Mat bg, frame, fgMask;

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
	//cout << "Dimensoes: " << ref.x << " " << ref.y << endl;
	
	cout << "Reconhecendo plano de fundo...\n";
	for(int i=0; i<bgIter; i++) {
		capture >> bg;
		if(frame.empty()) {
			cerr << "empty frame\n";
			return;
		}
		pBackSub->apply(bg, fgMask);
		cv::imshow("Plano de fundo", fgMask);
		if(cv::waitKey(5) >= 0)
			break;
	}
	
	if(!calibrate)
		cout << "Reconhecimento finalizado. Posicione os objetos e pressione ENTER.\n";
	else
		cout << "Posicione objetos de calibragem, de cores distintas do plano de fundo e\ncom perimetro de " << PERIM_CALIB << " cm.\n";
	cv::imshow("Plano de fundo", fgMask);
	int keyboard = cv::waitKey(0);
	if(keyboard == 'q')
		return;
	cv::destroyWindow("Plano de fundo");
	do {
		for(int i=0; i<objIter; i++) {
			capture >> frame;
			pBackSub->apply(bg, fgMask);
			if(frame.empty()) {
				cerr << "empty frame\n";
				return; //break;
			}
			cv::imshow("what", frame);
			if(cv::waitKey(5) >= 0) {
				break;
			}

		}
		cv::destroyWindow("what");
		pBackSub->apply(frame, fgMask);
		cv::namedWindow("Objetos", cv::WINDOW_NORMAL);
		cv::imshow("Objetos", fgMask);
		if(cv::waitKey(0) == 'q')
			return;

		cv::findContours(fgMask, contours, hierarchy,
				cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE,
				cv::Point(0,0));

		vector<cv::Rect> boundRect(contours.size());
		vector<vector<cv::Point>> contours_poly(contours.size());
		vector<cv::Moments> m(contours.size());
		centers.resize(contours.size(), cv::Point(0,0));
		centerscm.resize(contours.size(), cv::Point(0,0));

		double psum = 0;
		cout << "** Posicao dos centro dos objetos eh dado em centimetros,\nsendo a origem (0,0) o meio inferior da imagem (comprimento/2, largura). **\n";
		for(int i=0, perim = 0; i<contours.size(); i++) {
			if(cv::contourArea(contours[i]) < minarea)
				continue;
			valid_contours++;
			cout << endl;
			cv::putText(frame, "Objetos reconhecidos", cv::Point(10, 20), cv::FONT_HERSHEY_SIMPLEX, 0.75, cv::Scalar(0, 0, 255), 2);
			cv::approxPolyDP(contours[i], contours_poly[i], 3, true);
			if(calibrate) {
				perim = cv::arcLength(contours[i], true);
				psum = psum + perim/PERIM_CALIB;
				if(proportion > 0) {
					proportion = psum/(valid_contours);
				}
				else
					proportion = perim/PERIM_CALIB;
				cout << "Constante proporcional calculada: " << proportion << " pixels/cm\n";
			}

			boundRect[i] = cv::boundingRect(contours_poly[i]);
			cv::Scalar color = cv::Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));
			cv::rectangle(frame, boundRect[i].tl(), boundRect[i].br(), color, 2);

			m[i] = cv::moments(contours[i], true);
			centers[i] = cv::Point(m[i].m10/m[i].m00, m[i].m01/m[i].m00);
			centerscm[i] = cv::Point2d((centers[i].x-ref.x)/proportion, (ref.y-centers[i].y)/proportion);

			cv::circle(frame, centers[i], 3, cv::Scalar(128,0,0), -1);

			cout << "Objeto " << valid_contours << ": (" << centerscm[i].x << ", " << centerscm[i].y << ")\n";
			sprintf(name, "%d:(%.2lf,%.2lf)", valid_contours, centerscm[i].x, centerscm[i].y);
			cv::putText(frame, string(name), centers[i], cv::FONT_HERSHEY_SIMPLEX, 0.50, cv::Scalar(10, 0, 255), 2);
			cv::line(frame, ref, centers[i], cv::Scalar(0,200,0));
			cout << i << "\tr = " << sqrt(pow(centerscm[i].x, 2) + pow(centerscm[i].y, 2));
			cout << "\ttheta = " << 180*atan(((double) centerscm[i].y)/centerscm[i].x)/3.14159265<< endl;

		}
		cv::imshow("Reconhecidos", frame);
		if(cv::waitKey(0) == 'q') {
			if(!calibrate) return;
		}
		cout << centers.size() << " objetos reconhecidos, sendo " << valid_contours << " validos\n";
		cv::destroyWindow("Reconhecidos");
		cv::destroyWindow("Objetos");
		centers.clear();
		centerscm.clear();
		valid_contours = 0;
		cout << endl << endl;
	} while(!calibrate);
}


int main(int argc, char* argv[]) {
	if(argc < 4) {
		cerr << "Uso: " << argv[0] << " id_camera sensibilidade iteracoes1 iteracoes2\n";
		cerr << "\n\tPara informacoes no significado dos parametros, consulte o README.md\n";
		cerr << "\tid_camera: numero do arquivo da camera no /dev (e.g. /dev/video0 --> id_camera = 0)\n";
		return -1;
	}

	double dg_x=90, dg_y=0, dg_base=90;	// Posicao inicial dos servos (graus)
	double X, Y;	// Posicao instantanea do end effector

	usbase = degree_to_us(dg_base, SERVO_BASE);
	usx = degree_to_us(dg_x, SERVO_X);
	usy = degree_to_us(dg_y, SERVO_Y);

	printf("
	if(gpioInitialise() < 0)
		exit(1);

	//gpioSetSignalFunc(SIGINT, stop); // Signal handler de CTRL-C iria conflitar com o sinal de CTRL-C do OpenCV

	//gpioSetMode(BOBINA, PI_OUTPUT);
	//gpioWrite(BOBINA, 1);

	//gpioSetMode(BUT_SEL, PI_INPUT);
	//gpioSetMode(BUT_INC, PI_INPUT);
	//gpioSetMode(BUT_DEC, PI_INPUT);

	//gpioSetPWMfrequency(SERVO_BASE, 50);
	//gpioSetPWMfrequency(SERVO_X, 50);
	//gpioSetPWMfrequency(SERVO_Y, 50);
	//
	gpioServoBound(SERVO_BASE, degree_to_us(dg_base, 0));
	gpioServoBound(SERVO_X, degree_to_us(dg_x, 1));
	gpioServoBound(SERVO_Y, degree_to_us(dg_y, 2));

	gpioSetTimerFunc(0, 10, ease_func);

	while(run){
		fprintf(stdout, "Insira os valores X e Y\n");
		fprintf(stdout, "X: ");
		scanf("%lf", &X);
		fprintf(stdout, "Y: ");
		scanf("%lf", &Y);

		inverse_kinematics(X, Y, dg_x, dg_y); 
	}
	gpioTerminate();

	cout << "\t** Iniciando etapa de calibragem **\n\n";
	findObjects(true, atoi(argv[1]), atoi(argv[2]), atoi(argv[3]), atoi(argv[4]));
	cout << "\n\n\t** Calibragem finalizada. Iniciando etapa principal de reconhecimento de objetos. **\n\n";
	thread objectRecognition(findObjects, false, atoi(argv[1]), atoi(argv[2]), atoi(argv[3]), atoi(argv[4]));

	objectRecognition.join();

	return 0;
}
