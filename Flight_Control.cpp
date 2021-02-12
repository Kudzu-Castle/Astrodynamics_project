#include <iostream>
#include <stdio.h>
#include <Core>  
#include "ekfilter.hpp"
#include <iostream>
#include <fstream>
#include <cmath>
#include <ctime>
#include <cstdlib>
#include <vector>
#include <chrono>
#include <random>
#include "OPENCV.HPP"
#include "Common/MPU9250.h"
#include "Navio2/LSM9DS1.h"
#include "Common/MS5611.h"
#include "Common/Util.h"
#include <Dense>
#include "Ricatti_solver.h"
#include "Navio2/LSM9DS1.h"
#include "Common/InertialSensor.h"
#include <chrono>
#include "PWM.h"
#include <Navio2/PWM.h> 
#include <Navio2/RCInput_Navio2.h>
#include <Navio2/RCOutput_Navio2.h>
#include <thread>
#include <wiringPi.h>
#include <wiringSerial.h>
#include <cstring>
#include <unistd.h>				//Needed for I2C port
#include <fcntl.h>				//Needed for I2C port
#include <sys/ioctl.h>			//Needed for I2C port
#include <linux/i2c-dev.h>		//Needed for I2C port
#include <wiringPiI2C.h>


#define PWM_MIN 1060   /* mS */
#define PWM_MAX 1860   /* mS */
#define LIMIT(x,xl,xu) ((x)>=(xu)?(xu):((x)<(xl)?(xl):(x)))
#define IMU_MPU9250 1

using namespace Kalman;
using namespace std;
using namespace Eigen;
using namespace cv;
using namespace std::this_thread; // sleep_for, sleep_until
using namespace std::chrono; // nanoseconds, system_clock, seconds


double m = .2, M = .5, arm = 0.015, H_1=0.0254, h_2=.00127, Radius = 0.0381;	//small outer mass, big inner mass, arm length


const double Ixx = m*arm*arm/4+m*h_2*h_2/6+2*m*arm*arm+M* Radius * Radius /4+M*H_1*H_1/12;	//inertia calculations based on spherical assumptions
const double Iyy = m*arm*arm/4+m*h_2*h_2/6+2*m*arm*arm+M* Radius * Radius /4+M*H_1*H_1/12;
const double Izz = 4*m*arm*arm+M* Radius * Radius /12;

const double  l = .115;	//arm length  in meters 
const double g = 9.81;	//gravity constant

void readsensors(MatrixXd& sensor, double& gxtotal, double& gytotal, double& gztotal, double& accxtotal, double& accytotal, double& accztotal, int fd);
void KFilter(double h, MatrixXd U, MatrixXd A, MatrixXd B, MatrixXd sensor_readings, MatrixXd& X, double dt, MatrixXd, MatrixXd, MatrixXd&);
void RungeKutta(double h, MatrixXd A, MatrixXd& X, MatrixXd B, MatrixXd U);
double serialread(int pin, char type, int);
double Xcameratransform(double serialdata, double z);
double Ycameratransform(double serialdata, double z);
void inputcalc(MatrixXd& U, MatrixXd x);
//void LQR(MatrixXd A, MatrixXd R, MatrixXd Q, MatrixXd& u,  MatrixXd x, MatrixXd B);


int main()
{
	


		double  h=0.05;	//time step

		VectorXd input(4);
		MatrixXd X(12,1), U(4,1), Y(12,1);
		Y.setZero();
		X << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;	//state vector initialization
		U << 1, 0, 0, 0;	//initial input
		MatrixXd A(12, 12), Q(12,12), R(12,12), P(12,12);
		A.setZero();
		A(0, 6) = 1;
		A(1, 7) = 1;
		A(2, 8) = 1;
		A(3, 9) = 1;
		A(4, 10) = 1;
		A(5, 11) = 1;
		A(6, 4) = -g;
		A(7, 3) = g;

		Q.setIdentity();
		Q = Q * 0.001;
		R.setIdentity();
		R = R * 0.08;	/// small variance
		P.setIdentity();

		MatrixXd B(12, 4), S(12,12), Q_Ricatti(12,12), R_Ricatti(4,4);
		B.setZero();
		B(3, 1) = 1 / Ixx;
		B(4, 2) = 1 / Iyy;
		B(5, 3) = 1 / Izz;
		B(8, 0) = 1 / m;

		//matrices for LQR controller
		Q_Ricatti.setIdentity();
		R_Ricatti(0) = 0.004;
		R_Ricatti.setIdentity();


		ofstream xposition, yposition, pitchangle, rollangle, yawangle, zposition;	//opens files for writing data to plot later
		xposition.open("x.txt");
		yposition.open("y.txt");
		rollangle.open("roll.txt");
		pitchangle.open("pitch.txt");
		yawangle.open("yaw.txt");
		zposition.open("z.txt");

		//creates PID gains and values for calculations
		MatrixXd Kp_rot(3, 1), Kd_rot(3, 1), Ki_rot(3, 1), Integral_0(3,1),Integral_1(3,1), Derivative(3,1), Proportional(3,1), Ki_prior(3,1);
		Kp_rot.setIdentity();	//[roll, pitch, yaw]
		Kp_rot = Kp_rot * 0.0675;
		Kd_rot.setIdentity();
		Kd_rot = Kd_rot * 0.0018;
		Ki_rot.setIdentity();
		Ki_rot = Ki_rot * .09;

		Integral_0.setZero();
		Derivative.setZero();
		Proportional.setZero();

		MatrixXd error_0(3,1), error_1(3,1); 

		MatrixXd Xdesired(12, 1);	//desired at 1 meter alitiutde and zero angle of rotation
		Xdesired << 0, 0, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0;


	double	Kp_z, Ki_z, Kd_z,Integral_0z=0, Integral_1z, Derivative_z, error_0z=0, error_1z, F_control;
	Kp_z = 0.8;
	Ki_z = 0.8;
	Kd_z = 0.90;
	
	MatrixXd tau(3, 1);
	wiringPiSetup();		// Setup the library
	pinMode(11, OUTPUT);	// Configure GPIO0 as an output	TX
	pinMode(12, INPUT);		// Configure GPIO1 as an input	RX
	wiringPiSetupGpio();  // For GPIO pin numbering
	int devID=0x10;	//default id
	int i2c= wiringPiI2CSetup(devID);
	int fd;
	char* filename = (char*)"/dev/ttyS0";

	fd = serialOpen(filename, 115200);

		error_0.setZero();
		error_1.setZero();

		double dt = 0;	
		double gxtotal = 0, gytotal = 0, gztotal = 0, accxtotal = 0, accytotal = 0, accztotal = 0;
		int IMUsample = 0;
		bool start = false;
		RCInput_Navio2 rcin;
		rcin.initialize();	//initialize channels for PWM
		rcin.read(0);		//read channel 1 input

		PWM signal1, signal2, signal3, signal4;	//iniitalize PWM channels
		signal1.enable(0);
		signal1.init(0);
		signal2.enable(1);
		signal2.init(1);
		signal3.enable(2);
		signal3.init(2);
		signal4.enable(3);
		signal4.init(3);

		while (!start)	//begin main loop once RC input is turned on
		{
			if (rcin.read(0) > 0)
				start = true;
			else 
				rcin.read(0);

		}

		auto start = std::chrono::high_resolution_clock::now();
		inputcalc(U, X);	//calculates initial input of PWM to motors based on ref input


		//************** MAIN LOOP**********************
		for (double t = 0; t <= 30; t = t + h)
		{
			
			readsensors(Y, gxtotal, gytotal, gztotal, accxtotal, accytotal, accztotal, fd );	//read sensor data
			IMUsample++;

			KFilter(h, U,A,B ,Y, X, t, Q, R, P);	//filter sensor data with simulated predicted reuslts

			//PID for roll pitch yaw
			error_1(0) = Xdesired(3) - X(3);
			error_1(1) = Xdesired(4) - X(4);
			error_1(2) = Xdesired(5) - X(5);
			Integral_1 = Integral_0 + error_1;
			Derivative = (error_1 - error_0) / h;

			tau = Kp_rot * error_1 + Ki_rot * Integral_1 + Kd_rot * Derivative;

			error_0 = error_1;
			Integral_0 = Integral_1;

			//PID  for altitude
			error_1z = Xdesired(2) - X(2);	//find error bw desired and measured altitude
			Integral_1z = Integral_0z + error_1z * h;
			Derivative_z = (error_1z - error_0z)/h;

			F_control = Kp_z * error_1z + Ki_z * Integral_1z + Kd_z * Derivative_z;

			error_0z = error_1z;
			Integral_0z = Integral_1z;
			

			//updates the input variable based on PID feedback		U = [Thrust_force, Tau_roll, Tau_pitch, Tau_yaw]
			U(0) = F_control;
			U(1) = tau(0);
			U(2) = tau(1);
			U(3) = tau(2);

			inputcalc(U, X);	//converts the U vector to PWM signals

			// output data to files for plotting
			xposition << X(0) << endl;
			yposition << X(1) << endl;
			zposition << X(2) << endl;
			pitchangle << X(3) * 180 / 3.14 << endl;
			rollangle << X(4) << endl;
			yawangle << X(5) << endl;
		
			//if arm switch is turned off then cut power and fall to ground
			if (rcin.read(0) == 0)
			{
				signal1.set_duty_cycle(0,0);
				signal1.set_period(0, 0);
				signal2.set_duty_cycle(0, 0);
				signal2.set_period(0, 0);
				signal3.set_duty_cycle(0, 0);
				signal3.set_period(0, 0);
				signal4.set_duty_cycle(0, 0);
				signal4.set_period(0, 0);
				break;
			}

			sleep_for(milliseconds(50));
		}


		auto end = chrono::steady_clock::now();
		cout << steady_clock << endl;

		//close files
		xposition.close();
		yposition.close();
		pitchangle.close();
		zposition.close();
		pitchangle.close();
		rollangle.close();
		yawangle.close();
		serialClose(fd);

	return 0;

}



void readsensors(MatrixXd& sensor, double& gxtotal, double& gytotal, double& gztotal, double& accxtotal, double& accytotal, double& accztotal, int fd)
{
	//sensor = {x y z phi theta psi xdot ydot zdot phidot thetadot psidot}

	LSM9DS1 sensor_object;		//create sensor object
	sensor_object.probe();
	sensor_object.initialize();
	sensor_object.update();
	float* ax, * ay, * az, * gx, * gy, * gz;
	double alt, x, y;
	sensor_object.read_accelerometer(ax, ay, az);
	sensor_object.read_gyroscope(gx, gy, gz);
	gxtotal += *gx;
	gytotal += *gy;
	gztotal += *gz;
	accxtotal += *ax;
	accxtotal += *ay;
	accxtotal += *az;

	sensor(0) = *ax;
	sensor(1) = *ay;
	sensor(2) = *az;
	sensor(3) = *gx;	//angle rates of rotation
	sensor(4) = *gx;
	sensor(5) = *gx;

	
	//************read serial data from camera and LiDAR***************

	alt = serialread(16, 'z', fd); // read LiDAR pins

	x = Xcameratransform(serialread(17, 'x', fd), alt); //camera pins or via usb

	y = Ycameratransform(serialread(17, 'y',fd), alt);

		//put x, y and z from camera into sensor vector
		sensor(6) = x;
		sensor(7) = y;
		sensor(9) = alt;

}



double serialread(int pin, char type, int fd)
{

	if (pin == 17 && type == 'x')		//camera x coord		 serial data from camera looks like ->  T2 XXXX YYY
		if (digitalRead(pin) == 1)		//if receiving data
		{
			
			serialOPen(fd);
			string serialdata;
			getline(getChar(fd), serialdata);
			char* c;
			c = &serialdata[0];
			int i = 3;	//beginning of x data in serial string
			std::stringstream ss;
			while (c[i] != ' ')		//ends when it reaches a space
			{
				ss << c[i];
				i++;
			}
			int result;
			ss >> result;	//the is the x pixel coordinate				


			double x_coord = result;

			

			return x_coord;
		}

		else if (pin == 17 && type == 'y')
			if (digitalRead(pin) == 1)
			{
				

				std::string serialdata;
				getline(getChar(fd), serialdata);
				char* c;
				c = &serialdata[0];
				int i = 3;	//beginning of x data
				std::stringstream ss;
				while (c[i] != ' ')	//ends when it reaches a space
				{

					i++;
				}
				while (c[i] != ' ' || c[i] != '\n')	//ends when it reaches a space
				{
					ss << c[i];
					i++;
				}

				int result;
				ss >> result;	//the is the y pixel coordinate

				double y_coord = result;
			

				return y_coord;
			}


	//ALTITUDE PIN
	else if(pin == 16 &&type == 'z')
	{ 
		int file_i2c;		//for reading I2C 
		int length;
		unsigned char buffer[60] = { 0 };
		//----- OPEN THE I2C BUS -----
		char* filename = (char*)"/dev/i2c-1";
		if ((file_i2c = open(filename, O_RDWR)) < 0)
		{
			//ERROR HANDLING: you can check errno to see what went wrong
			printf("Failed to open the i2c bus");
			return;
		}

		int addr = 0x10;          //The default I2C address of the tfmini
		if (ioctl(file_i2c, I2C_SLAVE, addr) < 0)
		{
			printf("Failed to acquire bus access and/or talk to slave.\n");
			//ERROR HANDLING; you can check errno to see what went wrong
			return;
		}


		//----- READ BYTES -----
		length = 9;			//<<< Number of bytes to read
		if (read(file_i2c, buffer, length) != length)		//read() returns the number of bytes actually read, if it doesn't match then an error occurred (e.g. no response from the device)
		{
			//ERROR HANDLING: i2c transaction failed
			printf("Failed to read from the i2c bus.\n");
		}
		else
		{
			
			cout << buffer[2];
			return buffer[2];	//byte which contains distance dtata in meters? 
		}


	}
}

double Xcameratransform(double serialdata, double z)
{
		double theta = 60*3.14/180;
	
	double s = z * tan(theta / 2);
	double x_pixel = serialdata;	//pixel number between -1000 and +1000 for x-coordinate of QR code 

	if(x_pixel > 0)
		return -x_pixel * s / 1000;	//to the left of pad
	else if (x_pixel <0)
		return x_pixel * s / 1000;	//to the right of pad


	/*
	center of the camera's field of view is at x=0, y=0
	left edge of the camera image is always at x=-1000
	right edge of the camera image is always at x=1000
	top edge of the camera image is usually at y=-750
	bottom edge of the camera image is usually at y=750
	*/

}

	double Ycameratransform(double serialdata, double z)
{
	double theta = 50*3.14/180;

	double s = z * tan(theta / 2);
double y_pixel = serialdata;	//pixel number between -750 and +750 for y-coordinate of QR code 

	if(y_pixel > 0)
		return y_pixel * s / 750;	//to the left of pad
	else if (y_pixel < 0)
		return -y_pixel * s / 750;	//to the right of pad
}


//*********************************************************************************************
//********************* EXTENDED KALMAN FILTER FOR NON-LINEAR EOMS*********************
//*********************************************************************************************
/*
void EKFpredict(MatrixXd& x_old, MatrixXd U_old, MatrixXd& P_old, MatrixXd Q, double Ix , double Iy, double Iz, double h )
{
	double u_old = x_old(0);
	double v_old = x_old(1);
	double w_old = x_old(2);
	double p_old = x_old(3);
	double q_old = x_old(4);
	double r_old = x_old(5);
	double phi_old = x_old(6);
	double theta_old=x_old(7);
	double psi_old = phi_old;
	//state predcition
	MatrixXd dx(10, 1);
	dx(0) = r_old * v_old-q_old * w_old + g * sin(theta_old);
	dx(1) = p_old * w_old-r_old * u_old- g * cos(theta_old) * sin(phi_old);
	dx(2) = q_old * u_old - p_old * v_old - g * cos(theta_old) * cos(phi_old) +  U_old(1) / m;
	dx(3) = (Iy - Iz) * q_old * r_old / Ix + U_old(2) / Ix;
	dx(4) = (Iz -Ix) * p_old * r_old / Iy + U_old(3) / Iy;
	dx(5) = (Ix-Iy) * p_old * q_old / Iz + U_old(4) / Iz;
	dx(6) = p_old + sin(phi_old) * tan(theta_old) * q_old + cos(phi_old) * tan(theta_old) * r_old;
	dx(7) = cos(phi_old) * q_old -sin(phi_old) * r_old;
	dx(8) = (sin(phi_old) * q_old) / cos(theta_old) + (cos(phi_old) * r_old) / cos(theta_old);
	dx(9) = -sin(theta_old) * u_old -cos(theta_old) * sin(phi_old) * v_old + cos(theta_old) * cos(phi_old) * w_old;

	MatrixXd xhatbar(10, 1);
	xhatbar = x_old + h * dx;//return this? 
	//covariance prediction
	MatrixXd A(10, 10);
	A(0, 0) = r_old;
	A(0, 2) = -q_old;
	A(0, 3) = -w_old;
	A(0, 5) = v_old;
	A(0, 7) = g * cos(theta_old);
	A(1, 0) = -r_old;
	A(1, 2) = p_old;
	A(1, 3) = w_old;
	A(1, 5) = -u_old;
	A(1, 7) = -g * cos(theta_old) * cos(psi_old);
	A(1, 7) = g * sin(theta_old) * sin(psi_old);
	A(2, 0) = q_old;
	A(2, 1) = -p_old;
	A(2, 3) = -v_old;
	A(2, 4) = u_old;
	A(2, 6) = g * cos(theta_old) * sin(psi_old);
	A(2, 7) = g * sin(theta_old) * cos(psi_old);
	A(3, 4) = (Iyy - Izz) / Ixx * r_old;
	A(3, 5) = (Iyy - Izz) / Ixx * q_old;
	A(4, 3) = (Izz - Ixx) / Iyy * r_old;
	A(4, 5) = (Izz - Ixx) / Iyy * p_old;
	A(5, 3) = (Ixx - Iyy) / Izz * q_old;
	A(5, 4) = (Ixx - Iyy) / Izz * p_old;
	A(6, 3) = 1;
	A(6, 4) = sin(psi_old) * tan(theta_old);
	A(6, 5) = cos(psi_old) * tan(theta_old);
	A(6, 6) = cos(psi_old) * tan(theta_old) * q_old - sin(psi_old) * cos(theta_old) * r_old;
	A(6, 7) = (sin(psi_old) * q_old + cos(psi_old) * r_old) / pow(cos(theta_old), 2);
	A(7, 4) = cos(psi_old);
	A(7, 5) = -sin(psi_old);
	A(7, 6) = -cos(psi_old) * r_old - sin(psi_old) * q_old;
	A(8, 4) = sin(psi_old) / cos(theta_old);
	A(8, 5) = cos(psi_old) / cos(theta_old);
	A(8, 6) = (cos(psi_old) * q_old - sin(psi_old) * r_old) / cos(theta_old);
	A(8, 7) = tan(theta_old) / cos(theta_old) * (sin(psi_old) * r_old + cos(psi_old) * q_old);
	A(9, 0) = -sin(theta_old);
	A(9, 1) = cos(theta_old) * sin(psi_old);
	A(9, 2) = cos(theta_old) * cos(psi_old);
	A(9, 6) = cos(theta_old) * cos(psi_old) * v_old - cos(theta_old) * sin(psi_old) * w_old;
	A(9, 7) = -cos(theta_old) * u_old - sin(theta_old) * sin(psi_old) * u_old; -sin(theta_old) * cos(psi_old) * w_old;
	A = h * A;

	MatrixXd Pbar(10, 10);
	Pbar = A * P_old * A.inverse() + Q;
}
*/

//*********************************************************************************************
//******************** EXTENDED KALMAN FILTER FOR NON-LINEAR EOMs******************************
//*********************************************************************************************
/*
void EKFupdate(MatrixXd U, MatrixXd P_p, MatrixXd&  x_p, MatrixXd y, MatrixXd R, MatrixXd)
{
	double u = x_p(0);
	double v = x_p(1);
	double w = x_p(2);
	double p = x_p(3);
	double q = x_p(4);
	double r= x_p(5);
	double phi = x_p(6);
	double theta = x_p(7);
	double psi = x_p(8);
	double z = x_p(9);
	Matrix4d M;		//Matrix of constants for sensor simulation, needs filled in kf, d, kt are constants
	M >> kf, kf, kf, kf,
		kf* d / pow(2, 0.5), -kf * d / pow(2, 0.5), -kf * d / pow(2, 0.5), kf* d / pow(2, 0.5),
		kf* d / pow(2, 0.5), kf* d / pow(2, 0.5), -kf * d / pow(2, 0.5), -kf * d / pow(2, 0.5),
		kt, -kt, kt, -kt;

	// https://www.politesi.polimi.it/bitstream/10589/80681/3/2013_07_Ascorti.pdf page 35 describes what the Macc and Mg are for sensors
	
	MatrixXd H(10, 10), Macc(), Mgyro(), Sgyro(), Sacc(0);
	H.setZero();
	Macc.setZero();
	Mgyro.setZero();
	Sacc.setZero();
	Sgyro.setZero();
	H(1, 3) = Macc * Sacc * r;
	H(2, 1) = Macc * Sacc * -q;
	H(2, 3) = Macc * Sacc * -r;
	H(3, 1) = Macc * Sacc * p;
	H(3, 2) = Macc * Sacc * q;
	H(1, 2) = Macc * Sacc * -p;

	H(1, 5) = Macc * Sacc * -w;
	H(1, 6) = Macc * Sacc * v;
	H(2, 4) = Macc * Sacc * w;

	H(2, 6) = Macc * Sacc * -u;
	H(3, 4) = Macc * Sacc * -v;
	H(4, 4) = Macc * Sacc * u;
	H(15, 15) = 1;//LiDAR

	MatrixXd I(10, 10);
	I.setIdentity();
	MatrixXd K = P_p * H.transpose() * (R + H * P_p * H.transpose()).inverse();
	MatrixXd x = x_p + K * (y - H *x_p );	//check%%%%%%%%%%%%%%%%%
	MatrixXd P = (I - K * H) * P_p;
	x_p = x;	//
}
*/


//USING THIS METHOD TO SIMULATE through time
void RungeKutta(double h, MatrixXd A, MatrixXd& X, MatrixXd B, MatrixXd U )		 
{

	VectorXd K1(12,1);
	VectorXd K2(12,1);
	VectorXd K3(12,1);
	VectorXd K4(12,1);

	VectorXd initial(12);
	initial = X;
	MatrixXd dX(12, 1);

	dX = A * X + B * U;

	K1 = dX;

	X = X + K1 * (h / 2);


	dX = A * X + B * U;
	K2 = dX;


	X = X + K2 * (h / 2);


	dX = A * X + B * U;

	K3 = dX;

	X = X + K3 * h;

	dX = A * X + B * U;
	K4 = dX;

	X = initial + h * (K1 + 2 * K2 + 2 * K3 + K4) / 6;
}



//linear EOMs LQR CONTROLLER
/*
void LQR(MatrixXd A, MatrixXd R, MatrixXd Q, MatrixXd& u,  MatrixXd x, MatrixXd B)	
{
	MatrixXd K(12, 12), S(12,12);	//creates Gain matrix and S matrix
	
	solveRiccatiIterationD(A, B, Q, R, S);	//solves for S matrix
	
	K= R.inverse() * B.transpose() * S;			//computes gain from A, B, S

		u = -K * x;	//new  input matrix based on feedback as given in research paper

	//*************dont know if i need to do this calculation to create closed loop state matrix *************
		
		//Acl = A - B * K;
		//dX = Acl * X;	//creates new A matrix (A-closed loop) 
}
*/

void inputcalc(MatrixXd& U, MatrixXd x)	//task comes from camera data and heightd comes from the time of flight (0-60s)=2ft, 60s-3min = 0ft
// THIS TAKES DESIRED VALUES AND OUTPUTS CALCULATED PWM SIGNALS
{

	double V_bat = 9;	//battery voltage

	//x matrix is state matrix
	double x = x(0);
	double y = x(1);
	double  z = x(2);
	double phi = x(3);
	double theta = x(4);
	double psi = x(5);
	double dx = x(6);
	double dy = x(7);
	double dz = x(8);
	double dphi = x(9);
	double dtheta = x(10);
	double dpsi = x(11);

	//desired values
	double heightd = -1;	//1 meter
	double rolld = 0;
	double pitchd = 0;
	double yawd = 0;

	double delf = U(0);
	double delmx = U(1);
	double delmy = U(2);
	double delmz = U(3);

	double motor0 = -delmx + delmy + delmz + delf;
	double motor1 = -delmx - delmy - delmz + delf;
	double motor2 = +delmx - delmy + delmz + delf;
	double motor3 = +delmx + delmy - delmz + delf;


	double pwmMotor0 = LIMIT(PWM_MIN + (PWM_MAX - PWM_MIN) * (0.5 + 0.5 * motor0), PWM_MIN, PWM_MAX);
	double pwmMotor1 = LIMIT(PWM_MIN + (PWM_MAX - PWM_MIN) * (0.5 + 0.5 * motor1), PWM_MIN, PWM_MAX);
	double pwmMotor2 = LIMIT(PWM_MIN + (PWM_MAX - PWM_MIN) * (0.5 + 0.5 * motor2), PWM_MIN, PWM_MAX);
	double pwmMotor3 = LIMIT(PWM_MIN + (PWM_MAX - PWM_MIN) * (0.5 + 0.5 * motor3), PWM_MIN, PWM_MAX);

	PWM PWMsignal1, PWMsignal2, PWMsignal3, PWMsignal4;
	//channel 0 = physical channel 1
	PWMsignal1.set_duty_cycle(0, pwmMotor0);


	//channel 1 = physical channel 2
	PWMsignal2.set_duty_cycle(1, pwmMotor1);


	//channel 2 = physical channel 3
	PWMsignal3.set_duty_cycle(2, pwmMotor2);


	//channel 3 = physical channel 4
	PWMsignal4.set_duty_cycle(3, pwmMotor3);

	//https://docs.emlid.com/navio/navio+/dev/pwm-output

}




//*******************function for modeling sensors for testing filter**************************************

	/*void sensor(double z,  double accel[], double omega[], double time, int measurestep, double sensordata[][6], double stdlaser, double stdgyro) //sensor col 1-3 = x, y, z   col 3-6 = omega1,2,3
	{

		unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
		default_random_engine generator(seed);

		normal_distribution<double> distribution(0.2, stdlaser);

		double randnum = distribution(generator);
		normal_distribution<double> distribution(0.2, stdgyro);

		double randnum2 = distribution(generator);

		sensordata[0][measurestep] = z + randnum;

		sensordata[3][measurestep] = omega[0] + randnum2;
		sensordata[4][measurestep] = omega[1] + randnum2;
		sensordata[5][measurestep] = omega[2] + randnum2;

	}*/



//************************Linear kalman filter *****************************

void KFilter(double h, MatrixXd U, MatrixXd A, MatrixXd B, MatrixXd sensor_readings, MatrixXd& X, double dt, MatrixXd Q, MatrixXd R, MatrixXd& P)
{
	

	//sensor readings = [accel_x, accel_y, accel_z, gyro_p, gyro_q, gyro_r, alt]	from sensorread fucntion

	double accel_x = sensor_readings(0);
	double accel_y = sensor_readings(1);
	double accel_z = sensor_readings(2);
	double gyro_p = sensor_readings(3);
	double gyro_q = sensor_readings(4);
	double gyro_r = sensor_readings(5);
	double alt= sensor_readings(6);
	
	MatrixXd C(12, 12);	//sensor matrix
	C.setIdentity();

	//change values for considering accel data. 
	//this would convert accelerometer data into vel and pos directly within the sensor matrix
	C(3, 3) = 0;
	C(4, 4) = 0;
	C(5, 5) = 0;
	C(6, 6) = 0;// accel_x* dt;
	C(7, 7) = 0;// accel_y* dt;
	C(8, 8) = 0;// (accel_z + g)* dt;	//check signs
	C(9, 9) = 0;// X(6)* dt + 0.5 * accel_x * dt * dt;
	C(10, 10) = 0;// X(7)* dt + 0.5 * accel_y * dt * dt;

	

	MatrixXd z_sensor(12, 1);		//measurement vector = sensor matrix * state vector
	z_sensor = C * X;


	//prediction of states
	MatrixXd X_p(12, 1), I(12,12), P_p(12, 12), K(12, 12);
	I.setIdentity();

	RungeKutta(h, A, X, B, U);//numerical integrates to get simulated state vector

	X_p = X;	//predicted state vector 
	
	X_p(3) = X(3)+gyro_p * dt;	//rotation angles from gyro
	X_p(4) = X(4)+gyro_q * dt;
	X_p(5) = X(5)+gyro_r * dt;

	X_p(6) = X(6) + accel_x * dt;	//this uses accel_x,y,z to predict states outside sensor matrix.
	X_p(7) = X(7) + accel_y * dt;
	X_p(8) = X(8) + accel_z * dt;
	X_p(0) = X(0) + X(6) * dt + .5 * accel_x * dt * dt;
	X_p(1) = X(1) + X(7) * dt + .5 * accel_y * dt * dt;
	
	P_p = A * P * A.transpose() + Q;	//Covariance matrix
	K = P_p * C.transpose() * (C * P_p * C.transpose() + R).inverse();	//GAIN matrix

	//UPDATE 
	X = X_p + K * (z_sensor - C* X_p);

	P = (I - K * C) * P_p;


}

