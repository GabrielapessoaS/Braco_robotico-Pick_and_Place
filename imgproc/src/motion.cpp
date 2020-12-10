#include <stdio.h>
extern "C"{
	#include "inv_kinematics.h"

}
#include <unistd.h>
#include <iostream>
#include <numeric>
#include <thread>
#include <opencv2/imgproc.hpp>
#include <opencv2/video.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/videoio.hpp>

// Constantes de processamento de imagens
#define PERIM_CALIB 20.0 // Definir com ponto flutuante

using namespace std;

vector<cv::Point2d> centerscm;	// Vetor que contem os centros dos objetos
bool centers_available = false;	// Flag para indicar diponibilidade de centros no vetor
double proportion = -1;		// Coeficiente de proporcao de centimetros/pixel

int usbase = 0, usx = 0, usy = 0;
int lock_motion = 0;

void findObjects(bool calibrate, int cam, int minarea, int bgIter, int objIter);
void smoothMove();
int servoControl();

int main(int argc, char* argv[]) {
	if(argc < 4) {
		cerr << "Uso: " << argv[0] << " id_camera sensibilidade iteracoes1 iteracoes2\n";
		cerr << "\n\tPara informacoes no significado dos parametros, consulte o README.md\n";
		cerr << "\tid_camera: numero do arquivo da camera no /dev (e.g. /dev/video0 --> id_camera = 0)\n";
		return -1;
	}

	double dg_x=90, dg_y=0, dg_base=90;	// Posicao inicial dos servos (graus)
	double X, Y;				// Posicao instantanea do end effector

	usbase = degree_to_us(dg_base, SERVO_BASE);
	usx = degree_to_us(dg_x, SERVO_X);
	usy = degree_to_us(dg_y, SERVO_Z);

	fprintf(stderr, "valor retornado por base: %d\n", degree_to_us(dg_base, SERVO_BASE));
	fprintf(stderr, "valor retornado por X: %d\n", degree_to_us(dg_x, SERVO_X));
	fprintf(stderr, "valor retornado por Y: %d\n", degree_to_us(dg_y, SERVO_Z));


	printf("Inicializando GPIOs dos servos...\n");
	if(gpioInitialise() < 0) {
		cout << "Erro ao inicializar os GPIOs.\n";
		exit(1);
	}

	//gpioSetMode(BOBINA, PI_OUTPUT);
	//gpioWrite(BOBINA, 1);

	gpioServoBound(SERVO_BASE, degree_to_us(dg_base, SERVO_BASE));
	gpioServoBound(SERVO_X, degree_to_us(dg_x, SERVO_X));
	gpioServoBound(SERVO_Z, degree_to_us(dg_y, SERVO_Z));

	cout << "Servos inicializados.\n";

	cout << "\t** Iniciando etapa de calibragem **\n\nSiga as orientações\n\n";

	// Calibragem
	findObjects(true, atoi(argv[1]), atoi(argv[2]), atoi(argv[3]), atoi(argv[4]));
	cout << "\n\n\t** Calibragem finalizada. Iniciando etapa \
		 principal de reconhecimento de objetos. **\n\n";
	// Thread de suavizacao de movimentos dos servomotores
	thread armServos(smoothMove);
	// Thread de processamento de imagens
	thread objectRecognition(findObjects, false, atoi(argv[1]), atoi(argv[2]), atoi(argv[3]), atoi(argv[4]));


	objectRecognition.join();

	gpioTerminate();

	return 0;
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
	
	// RECONHECIMENTO DE PLANO DE FUNDO //
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
	// FIM DO RECONHECIMENTO DE PLANO DE FUNDO //
	
	// Loop de pick-n-place
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
		cout << "** Posicao dos centro dos objetos eh dada em centimetros,\nsendo a origem (0,0) o meio inferior da imagem (comprimento/2, largura). **\n";
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
		// Reconhecimento finalizado
		// Etapa de movimentacao dos objetos
		if(!calibrate) {
			cout << "Iniciando etapa de movimentação de objetos...\n";

			if(servoControl() > 0) {
				cout << "Todos objetos movimentados com sucesso.\n";
			}
		}

		cv::destroyWindow("Objetos");
		centers.clear();
		centerscm.clear();
		valid_contours = 0;
		cout << endl << endl;
	} while(!calibrate);
}

void smoothMove() {
  int pulse_base;
  int pulse_x;
  int pulse_y;

  while(1) {
	  usleep(10);
	  pulse_base = gpioGetServoPulsewidth(SERVO_BASE);
	  pulse_x = gpioGetServoPulsewidth(SERVO_X);  
	  pulse_y = gpioGetServoPulsewidth(SERVO_Z);

	  if( (pulse_base == usbase) && (pulse_x == usx) && (pulse_y == usy) ) {
		  cout << "Atingido ponto do objeto.\n";
		  usleep(100000);
		  lock_motion = 0;
	  }

	  if( pulse_base < usbase )
		  gpioServoBound(SERVO_BASE, pulse_base + SPEED);
	  else if( pulse_base > usbase)
		  gpioServoBound(SERVO_BASE, pulse_base - SPEED);

	  if( pulse_x < usx )
		  gpioServoBound(SERVO_X, pulse_x + SPEED);
	  else if( pulse_x > usx)
		  gpioServoBound(SERVO_X, pulse_x - SPEED);

	  if( pulse_y < usy )
		  gpioServoBound(SERVO_Z, pulse_y + SPEED);
	  else if( pulse_y > usy)
		  gpioServoBound(SERVO_Z, pulse_y - SPEED);
  }
}

int servoControl() {
	if(!centers_available) return -1;
	for(int i=0; i<centerscm.size(); i++) {
		inverse_kinematics(centerscm[i].x, centerscm[i].y, &usbase, &usx, &usy);
		lock_motion = 1;
		cout << "Resgatando objeto...\n";
		while(lock_motion) {};
		cout << "Objeto " << i << " resgatado.\n";
	}
	return 0;
}