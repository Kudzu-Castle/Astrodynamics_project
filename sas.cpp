/*#include <unistd.h>*/
#include <cstdio>
#include "Navio2/PWM.h"
#include "Navio+/RCOutput_Navio.h"
#include "Navio2/RCOutput_Navio2.h"
#include <Navio2/RCInput_Navio2.h>
#include <Navio+/RCInput_Navio.h>
#include <Navio2/ADC_Navio2.h>
#include <Navio+/ADC_Navio.h>
#include "Common/MPU9250.h"
#include "Navio2/LSM9DS1.h"
#include "Common/MS5611.h"
#include "Common/Util.h"
#include <unistd.h>
#include <string>
#include <memory>
#include <time.h>
#include "network.h"

/* gains/params */
/* feed forward (rad/sec for full stick) */
#define KFF_X 3.14
#define KFF_Y 3.14
#define KFF_Z 3.14
/* Feedback gains P and I */
#define KP_X 0.1
#define KP_Y 0.1
#define KP_Z 0.1
#define KI_X (KP_X*KP_X*100/2) /* Kp^2*b/4 (or/2 for .707 damping) */
#define KI_Y (KP_Y*KP_Y*100/2)
#define KI_Z (KP_Z*KP_Z*20/2)
/* forgetting factor on integral term */
#define TAUI (2*5/(KP_X*100))   /* 6/(Kp*b) */    /* cannot be zero */
/* lead/lag compensator */
#define TAUM 0.025       /* cannot be zero */
#define TAUA (TAUM/3)   /* TAUA = TAUM will eliminate dynamic compensator */

/* motors */
#define PWM_HZ 400
#define PWM_MOTOR0 1   /* right front */
#define PWM_MOTOR1 0   /* right aft */
#define PWM_MOTOR2 3   /* left aft */
#define PWM_MOTOR3 2   /* left front */
#define PWM_MIN 1060   /* mS */
#define PWM_MAX 1860   /* mS */

/* pilot input */
#define PWM_THRUST 0
#define PWM_ROLL   1
#define PWM_PITCH  2
#define PWM_YAW    3
#define PWM_ARM    4
#define PWM_AUTO   5
#define THRUST_MIN 990
#define THRUST_MAX 2016
#define ROLL_MIN 990
#define ROLL_MAX 2016
#define PITCH_MIN 2016  /* note direction reversal */
#define PITCH_MAX 990
#define YAW_MIN 990
#define YAW_MAX 2016
#define ARMED_THRESHOLD 1700
#define AUTO_THRESHOLD  1700
#define AUTO_ON 2000
#define AUTO_OFF 1000
#define AUTO_TIMEOUT 200

/* IMU */
#define IMU_MPU9250 1
//#define IMU_LSM9DS1 1

/* Voltage */
#define A2D_VOLTAGECHANNEL 2

#define LIMIT(x,xl,xu) ((x)>=(xu)?(xu):((x)<(xl)?(xl):(x)))

using namespace Navio;

std::unique_ptr <RCInput> get_rcin() {

	if (get_navio_version() == NAVIO2) {
		auto ptr = std::unique_ptr <RCInput>{ new RCInput_Navio2() };
		return ptr;
	}
	else {
		auto ptr = std::unique_ptr <RCInput>{ new RCInput_Navio() };
		return ptr;
	}

}

std::unique_ptr <RCOutput> get_rcout() {

	if (get_navio_version() == NAVIO2) {
		auto ptr = std::unique_ptr <RCOutput>{ new RCOutput_Navio2() };
		return ptr;
	}
	else {
		auto ptr = std::unique_ptr <RCOutput>{ new RCOutput_Navio() };
		return ptr;
	}

}

std::unique_ptr <InertialSensor> get_inertial_sensor() 
{

#ifdef IMU_MPU9250
	auto ptr = std::unique_ptr <InertialSensor>{ new MPU9250() };
	return ptr;
#endif
#ifdef IMU_LSM9DS1
	auto ptr = std::unique_ptr <InertialSensor>{ new LSM9DS1() };
	return ptr;
#endif

}

std::unique_ptr <ADC> get_converter() {

	if (get_navio_version() == NAVIO2) {
		auto ptr = std::unique_ptr <ADC>{ new ADC_Navio2() };
		return ptr;
	}
	else {
		auto ptr = std::unique_ptr <ADC>{ new ADC_Navio() };
		return ptr;
	}

}

timespec diff(timespec start, timespec end) 
{

	timespec temp;
	if ((end.tv_nsec - start.tv_nsec) < 0) {
		temp.tv_sec = end.tv_sec - start.tv_sec - 1;
		temp.tv_nsec = 1000000000 + end.tv_nsec - start.tv_nsec;
	}
	else {
		temp.tv_sec = end.tv_sec - start.tv_sec;
		temp.tv_nsec = end.tv_nsec - start.tv_nsec;
	}
	return temp;

}

int main(int argc, char* argv[])
{

	int pwm, armed = 0;
	int count = 0;
	double rollStick, pitchStick, yawStick, thrustStick;
	float wx, wy, wz;
	float ax, ay, az;
	double wxTotal = 0, wyTotal = 0, wzTotal = 0, wxTotalAuto = 0, wyTotalAuto = 0, wzTotalAuto = 0;
	double sx = 0, sy = 0, sz = 0, axTotalAuto = 0, ayTotalAuto = 0, azTotalAuto = 0;
	int IMUsamples = 0, IMUsamplesAuto = 0;
	double p = 0, q = 0, r = 0, pp = 0, qq = 0, rr = 0;
	double delmx, delmy, delmz, delf;
	double dt = 1.0 / PWM_HZ, time;

	double e, u1, x4x = 0, x4y = 0, x4z = 0, x5x = 0, x5y = 0, x5z = 0;
	double x4x_dot, x4y_dot, x4z_dot;
	double x5x_dot, x5y_dot, x5z_dot;
	double oldx4x_dot = 0, oldx4y_dot = 0, oldx4z_dot = 0;
	double oldx5x_dot = 0, oldx5y_dot = 0, oldx5z_dot = 0;

	double motor0, motor1, motor2, motor3;
	int pwmMotor0, pwmMotor1, pwmMotor2, pwmMotor3;

	MS5611 barometer;
	float absPress;
	float mx, my, mz;
	char sampleBit = 0;

	int voltage = 12000;
	short pilotPwm[7] = { 1500,1500,1500,1500,1500,1500,1500 };
	short autoPwm[7] = { 1500,1500,1500,1500,1500,1500,1500 };
	int autopilot = 0;
	double rollStick_auto = 0, pitchStick_auto = 0, yawStick_auto = 0, thrustStick_auto = 0;
	int countsSinceAuto = -AUTO_TIMEOUT * 2;

	timespec newtime, oldtime;
	int startSec;

	if (check_apm()) {
		return EXIT_FAILURE;
	}

	/* rc input setup */

	printf("Init rc input\n");
	auto rcin = get_rcin();
	rcin->initialize();

	/* motor control setup */

	printf("Init motors\n");
	auto motors = get_rcout();
	if (getuid()) {
		printf("Not root. Please launch like this: sudo %s\n", argv[0]);
		return EXIT_FAILURE;
	}
	if (!(motors->initialize(PWM_MOTOR0))) { return EXIT_FAILURE; }
	if (!(motors->initialize(PWM_MOTOR1))) { return EXIT_FAILURE; }
	if (!(motors->initialize(PWM_MOTOR2))) { return EXIT_FAILURE; }
	if (!(motors->initialize(PWM_MOTOR3))) { return EXIT_FAILURE; }

	motors->set_frequency(PWM_MOTOR0, PWM_HZ);
	motors->set_frequency(PWM_MOTOR1, PWM_HZ);
	motors->set_frequency(PWM_MOTOR2, PWM_HZ);
	motors->set_frequency(PWM_MOTOR3, PWM_HZ);

	if (!(motors->enable(PWM_MOTOR0))) { return EXIT_FAILURE; }
	if (!(motors->enable(PWM_MOTOR1))) { return EXIT_FAILURE; }
	if (!(motors->enable(PWM_MOTOR2))) { return EXIT_FAILURE; }
	if (!(motors->enable(PWM_MOTOR3))) { return EXIT_FAILURE; }

	/* IMU measurement setup */

	printf("Init IMU and Mag\n");
	auto imu = get_inertial_sensor();
	if (!imu) {
		printf("IMU not found\n");
		return EXIT_FAILURE;
	}
	if (!imu->probe()) {
		printf("IMU not enabled\n");
		return EXIT_FAILURE;
	}
	imu->initialize();

	/* air data setup */

	printf("Init air data\n");
	barometer.initialize();
	barometer.refreshTemperature();
	usleep(10000); // Waiting for temperature data ready
	barometer.readTemperature();
	barometer.refreshPressure();

	/* A2D setup */

	printf("Init a2d\n");
	auto adc = get_converter();
	adc->initialize();

	/* get network ready */

	openPort();

	/* main loop */

	printf("SAS starting\n");

	clock_gettime(CLOCK_REALTIME, &oldtime);
	startSec = oldtime.tv_sec;

	while (true) 
	{

		/* read IMU measurements (be ready to average them) */

		imu->update();
		imu->read_gyroscope(&wx, &wy, &wz);
		wxTotal += wx;
		wyTotal += wy;
		wzTotal += wz;
		wxTotalAuto += wx;
		wyTotalAuto += wy;
		wzTotalAuto += wz;
		imu->read_accelerometer(&ax, &ay, &az);
		axTotalAuto += ax;
		ayTotalAuto += ay;
		azTotalAuto += az;
		IMUsamples++;
		IMUsamplesAuto++;
		//printf( "wx=%f\twy=%f\twz=%f\ts%d\n", wz, wy, wz, IMUsamples );

		/* read clock, is it time to update? (update at PWM_HZ) */

		clock_gettime(CLOCK_REALTIME, &newtime);
		if (diff(oldtime, newtime).tv_nsec > (1000000000 / PWM_HZ)) {
			time = (double)(newtime.tv_sec - startSec) + (double)(newtime.tv_nsec) / 1000000000;
			//if( 0==(count%40) ) printf( "time = %.4f\n", time );
			//printf( "diff = %d\n", diff( oldtime, newtime ).tv_nsec );
			oldtime.tv_nsec += (1000000000 / PWM_HZ);
			if (oldtime.tv_nsec > 1000000000) {
				oldtime.tv_sec--;
				oldtime.tv_nsec -= 1000000000;
			}
			count++;

			/* prepare IMU measurements (average since last update) */

			if (IMUsamples > 0) {
				pp = +wyTotal / IMUsamples;  /* get mounting orientation right here, raw is ENU */
				qq = +wxTotal / IMUsamples;  /* rad/sec */
				rr = -wzTotal / IMUsamples;
			}
			//if( (count%400)<3 ) printf( "t=%.2f\tp=%.2f\tq=%.2f\tr=%.2f\ts%d\n", time, p, q, r, IMUsamples );
			IMUsamples = 0;
			wxTotal = 0;
			wyTotal = 0;
			wzTotal = 0;

			if (0 == (count % 4)) { /* if PWM_HZ is 400, then this happens at 100Hz */

				/* send IMU data out to autopilot computer */

				if (IMUsamplesAuto > 0) {
					p = +wyTotalAuto / IMUsamplesAuto;  /* get mounting orientation right here, raw is ENU */
					q = +wxTotalAuto / IMUsamplesAuto;  /* rad/sec */
					r = -wzTotalAuto / IMUsamplesAuto;
					sx = -ayTotalAuto / IMUsamplesAuto; /* why is it different that the gyro? */
					sy = -axTotalAuto / IMUsamplesAuto;
					sz = +azTotalAuto / IMUsamplesAuto;
				}
				
				IMUsamplesAuto = 0;
				wxTotalAuto = 0;
				wyTotalAuto = 0;
				wzTotalAuto = 0;
				axTotalAuto = 0;
				ayTotalAuto = 0;
				azTotalAuto = 0;

				sendIMUdata(p, q, r, sx, sy, sz);

				/* read pilot inputs to autopilot computer */

				pwm = rcin->read(PWM_ROLL);
				if (pwm <= 0) {
					rollStick = 0;
				}
				else {
					rollStick = ((double)(pwm - ROLL_MIN) * 2) / (ROLL_MAX - ROLL_MIN) - 1;
					pilotPwm[PWM_ROLL] = pwm;
				}

				pwm = rcin->read(PWM_PITCH);
				if (pwm <= 0) {
					pitchStick = 0;
				}
				else {
					pitchStick = ((double)(pwm - PITCH_MIN) * 2) / (PITCH_MAX - PITCH_MIN) - 1;
					pilotPwm[PWM_PITCH] = pwm;
				}

				pwm = rcin->read(PWM_YAW);
				if (pwm <= 0) {
					yawStick = 0;
				}
				else {
					yawStick = ((double)(pwm - YAW_MIN) * 2) / (YAW_MAX - YAW_MIN) - 1;
					pilotPwm[PWM_YAW] = pwm;
				}

				pwm = rcin->read(PWM_THRUST);
				if (pwm <= 0) {
					thrustStick = -1;
				}
				else {
					thrustStick = ((double)(pwm - THRUST_MIN) * 2) / (THRUST_MAX - THRUST_MIN) - 1;
					pilotPwm[PWM_THRUST] = pwm;
				}

				pwm = rcin->read(PWM_ARM); /* motor arming on/off switch */
				if (pwm <= 0) {
					armed = 0; /* maybe just turn autopilot on? */
					thrustStick = -1;
				}
				else {
					if (pwm > ARMED_THRESHOLD) {
						armed = 1;
					}
					else {
						if (1 == armed) { /* motors have becomes disarmed - turn them to minimum */
							motors->set_duty_cycle(PWM_MOTOR0, PWM_MIN);
							motors->set_duty_cycle(PWM_MOTOR1, PWM_MIN);
							motors->set_duty_cycle(PWM_MOTOR2, PWM_MIN);
							motors->set_duty_cycle(PWM_MOTOR3, PWM_MIN);
						}
						armed = 0;
						thrustStick = -1;
					}
					pilotPwm[PWM_ARM] = pwm;
				}

				pwm = rcin->read(PWM_AUTO); /* autopilot switch on/off */
				if (pwm <= 0) {
					autopilot = 0;
				}
				else {
					if (pwm > AUTO_THRESHOLD) {
						if (0 == autopilot) {
							autopilot = 1;
						}
					}
					else {
						if (1 == autopilot) {
							autopilot = 0;
						}
					}
				}
				/* pilotPwm[PWM_AUTO] = pwm; this is handled below */

				//printf( "t=%.2f\tp=%.2f\tq=%.2f\tr=%.2f\ta=%d\n", thrustStick, rollStick, pitchStick, yawStick, armed );

				if (readAutopilotInputs(autoPwm)) {
					countsSinceAuto = 0;
					rollStick_auto = ((double)(autoPwm[PWM_ROLL] - ROLL_MIN) * 2) / (ROLL_MAX - ROLL_MIN) - 1;
					pitchStick_auto = ((double)(autoPwm[PWM_PITCH] - PITCH_MIN) * 2) / (PITCH_MAX - PITCH_MIN) - 1;
					yawStick_auto = ((double)(autoPwm[PWM_YAW] - YAW_MIN) * 2) / (YAW_MAX - YAW_MIN) - 1;
					thrustStick_auto = ((double)(autoPwm[PWM_THRUST] - THRUST_MIN) * 2) / (THRUST_MAX - THRUST_MIN) - 1;
				}
				else {
					if (countsSinceAuto++ > AUTO_TIMEOUT) {  /* turn off autopilot if haven't heard from it in a while... */
						autopilot = 0;
					}
				}

			}

			/* use autopilot inputs instead of pilot inputs if autopilot is on */

			if (1 == autopilot) {
				rollStick = rollStick_auto;
				pitchStick = pitchStick_auto;
				yawStick = yawStick_auto;
				thrustStick = thrustStick_auto;
			}

			/* feedforward and stability augmentation */

			e = rollStick * KFF_X - pp;
			u1 = e * KP_X + x4x * KI_X;
			delmx = (x5x * (TAUA - TAUM) + u1 * TAUM) / TAUA;
			x4x_dot = e - x4x / TAUI;
			x5x_dot = (u1 - x5x) / TAUA;
			x4x += (x4x_dot * 2 - oldx4x_dot) * dt;  oldx4x_dot = x4x_dot;
			x5x += (x5x_dot * 2 - oldx5x_dot) * dt;  oldx5x_dot = x5x_dot;

			e = pitchStick * KFF_Y - qq;
			u1 = e * KP_Y + x4y * KI_Y;
			delmy = (x5y * (TAUA - TAUM) + u1 * TAUM) / TAUA;
			x4y_dot = e - x4y / TAUI;
			x5y_dot = (u1 - x5y) / TAUA;
			x4y += (x4y_dot * 2 - oldx4y_dot) * dt;  oldx4y_dot = x4y_dot;
			x5y += (x5y_dot * 2 - oldx5y_dot) * dt;  oldx5y_dot = x5y_dot;

			e = yawStick * KFF_Z - rr;
			u1 = e * KP_Z + x4z * KI_Z;
			delmz = (x5z * (TAUA - TAUM) + u1 * TAUM) / TAUA;
			x4z_dot = e - x4z / TAUI;
			x5z_dot = (u1 - x5z) / TAUA;
			x4z += (x4z_dot * 2 - oldx4z_dot) * dt;  oldx4z_dot = x4z_dot;
			x5z += (x5z_dot * 2 - oldx5z_dot) * dt;  oldx5z_dot = x5z_dot;

			delf = thrustStick;

			//printf( "t=%.2f\tp=%.2f\tq=%.2f\tr=%.2f\n", delf, delmx, delmy, delmz );

			/* motor mixing */

			motor0 = -delmx + delmy + delmz + delf;
			motor1 = -delmx - delmy - delmz + delf;
			motor2 = +delmx - delmy + delmz + delf;
			motor3 = +delmx + delmy - delmz + delf;

			/* make sure we stay within limits per motor */

			pwmMotor0 = LIMIT(PWM_MIN + (PWM_MAX - PWM_MIN) * (0.5 + 0.5 * motor0), PWM_MIN, PWM_MAX);
			pwmMotor1 = LIMIT(PWM_MIN + (PWM_MAX - PWM_MIN) * (0.5 + 0.5 * motor1), PWM_MIN, PWM_MAX);
			pwmMotor2 = LIMIT(PWM_MIN + (PWM_MAX - PWM_MIN) * (0.5 + 0.5 * motor2), PWM_MIN, PWM_MAX);
			pwmMotor3 = LIMIT(PWM_MIN + (PWM_MAX - PWM_MIN) * (0.5 + 0.5 * motor3), PWM_MIN, PWM_MAX);

			//printf( "0=%.2f\t1=%.2f\t2=%.2f\t3=%.2f\n", motor0, motor1, motor2, motor3 );
			//if( (count%400) < 3 ) printf( "%.2f\t0=%d\t1=%d\t2=%d\t3=%d\n", time, pwmMotor0, pwmMotor1, pwmMotor2, pwmMotor3 );

			/* send out motor commands (this code sends nothing if not armed - on Navio2 that means motor will stop (eventually) */

			if (1 == armed) {
				motors->set_duty_cycle(PWM_MOTOR0, pwmMotor0);
				motors->set_duty_cycle(PWM_MOTOR1, pwmMotor1);
				motors->set_duty_cycle(PWM_MOTOR2, pwmMotor2);
				motors->set_duty_cycle(PWM_MOTOR3, pwmMotor3);
			}

			/* 10 Hz stuff */
			/* read mag and baro and send to autopilot */
			if (1 == (count % 40)) {
				imu->read_magnetometer(&mx, &my, &mz);
				if (sampleBit) {
					barometer.readPressure();
					sampleBit = 0;
				}
				else {
					barometer.readTemperature();
					sampleBit = 1;
				}
				barometer.calculatePressureAndTemperature();
				absPress = barometer.getPressure() * 0.0145038;
				if (count > 1000) sendMagAirdata(mx, my, mz, absPress, voltage); /* unsure of order and sign here */
				//printf( "Ps = %f\n", absPress );
				if (sampleBit) {
					barometer.refreshPressure();
				}
				else {
					barometer.refreshTemperature();
				}
			}

			/* send actual autopilot mode (on or off) */
			if (17 == (count % 40)) {
				if (1 == autopilot) {
					pilotPwm[PWM_AUTO] = AUTO_ON;
				}
				else {
					pilotPwm[PWM_AUTO] = AUTO_OFF;
				}
				sendPilotLink(pilotPwm);
			}

			/* 1 Hz stuff */
			/* read voltage */
			if (30 == (count % 400)) {
				voltage = adc->read(A2D_VOLTAGECHANNEL) * 15040 / 1374;
			}

		}

		usleep(1);  /* release CPU */

	}

	return 0;

}




void readsensors(MatrixXd& sensor, double& gxtotal, double& gytotal, double& gztotal, double& accxtotal, double& accytotal, double& accztotal)
{
	//sensor = {x y z phi theta psi xdot ydot zdot phidot thetadot psidot}
	//********* SAMPLE NAVIO HAT CODE GOES HERE FOR READING IMUS

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

	//need to put in your own serial read function once you figure it out

	alt = serialread(16, 'z'); // read LiDAR pins

	x = Xcameratransform(serialread(17, 'x'), alt); //camera pins or via usb

	y = Ycameratransform(serialread(17, 'y'), alt);

	//put x, y and z from camera into sensor vector
	sensor(6) = x;
	sensor(7) = y;
	sensor(9) = alt;

}



double serialread(int pin, char type)
{
	wiringPiSetup();		// Setup the library
		//camera pins - 18,18
		//altimeter pin - ??
	pinMode(11, OUTPUT);	// Configure GPIO0 as an output	TX
	pinMode(12, INPUT);		// Configure GPIO1 as an input	RX



	if (pin == 17 && type == 'x')		//camera x coord		// serial data from camera looks like ->  T2 XXXX YYY
		if (digitalRead(pin) == 1)		//if receiving data
		{
			serialOpen(1, 115200);

			string serialdata;
			getline(getChar(1), serialdata);
			char* c;
			c = &serialdata[0];
			int i = 3;	//beginning of x data
			std::stringstream ss;
			while (c[i] != ' ')	//ends when it reaches a space
			{
				ss << c[i];
				i++;
			}
			int result;		//TEST FOR NEGATIVE SIGN
			ss >> result;	//the is the x pixel coordinate				


			double x_coord = result;

			serialClose(1);

			//WANT TERSE

			return x_coord;
		}

		else if (pin == 17 && type == 'y')
			if (digitalRead(pin) == 1)
			{
				serialOpen('/dev/ttyS0', 115200);
				read('/dev/ttyS0')	//to get fd?
					serialGetchar(int fd);	//fd = file desc??

				std::string serialdata;
				getline(getChar(), serialdata);
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
				serialClose(1);

				//NEED FUNCTION TO EXTRACT Y COORD

				return y_coord;
			}


	//ALTITUDE PIN
			else if (pin == 16 && type == 'z')
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

				int addr = 0x5a;          //<<<<<The I2C address of the slave
				if (ioctl(file_i2c, I2C_SLAVE, addr) < 0)
				{
					printf("Failed to acquire bus access and/or talk to slave.\n");
					//ERROR HANDLING; you can check errno to see what went wrong
					return;
				}


				//----- READ BYTES -----
				length = 4;			//<<< Number of bytes to read
				if (read(file_i2c, buffer, length) != length)		//read() returns the number of bytes actually read, if it doesn't match then an error occurred (e.g. no response from the device)
				{
					//ERROR HANDLING: i2c transaction failed
					printf("Failed to read from the i2c bus.\n");
				}
				else
				{
					printf("Data read: %s\n", buffer);
				}


			}
}

double Xcameratransform(double serialdata, double z)
{
	double theta = 60 * 3.14 / 180;

	double s = z * tan(theta / 2);
	double x_pixel = serialdata;	//pixel number between -1000 and +1000 for x-coordinate of QR code 

	if (x_pixel > 0)
		return -x_pixel * s / 1000;	//to the left of pad
	else if (x_pixel < 0)
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
	double theta = 50 * 3.14 / 180;

	double s = z * tan(theta / 2);
	double y_pixel = serialdata;	//pixel number between -750 and +750 for y-coordinate of QR code 

	if (y_pixel > 0)
		return y_pixel * s / 750;	//to the left of pad
	else if (y_pixel < 0)
		return -y_pixel * s / 750;	//to the right of pad
}





void RungeKutta(double h, MatrixXd A, MatrixXd& X, MatrixXd B, MatrixXd U)		//USING THIS METHOD TO SIMULATE through time 
{

	VectorXd K1(12, 1);
	VectorXd K2(12, 1);
	VectorXd K3(12, 1);
	VectorXd K4(12, 1);

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


void inputcalc(MatrixXd& U, MatrixXd x)	//task comes from camera data and heightd comes from the time of flight (0-60s)=2ft, 60s-3min = 0ft
// THIS TAKES DESIRED VALUES AND OUTPUTS CALCULATED VOLTAGES? OR FORCES/TORQUES?
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


	//Control parameters
	double k1 = 4;
	double k2 = 2;
	double k3 = 4;
	double k4 = 2;
	double k5 = 4;
	double k6 = 2;
	double k7 = 2;
	double k8 = 2;

	//Control laws
	double control_1 = -k1 * (z - heightd) - k2 * dz;
	double U1 = (m * g + control_1) / (cos(phi) * cos(theta));
	double U2 = Ixx * (-k3 * (phi - rolld) - k4 * dphi);
	double U3 = Iyy * (-k5 * (theta - pitchd) - k6 * dtheta);
	double U4 = Izz * (-k7 * (psi - yawd) - k8 * dpsi);


	double U1 = U(0);
	double U2 = U(1);
	double U3 = U(2);
	double U4 = U(3);

	MatrixXd A(4, 4);	//just a matrix of constants, not properly defined yet

	double b = .0039016, d = .0039016, k = 1, constant = 1.2;

	A << -b, -b, -b, -b,
		0, -d * b, 0, d* b,
		d* b, 0, -d * b, 0,
		k, -k, k, -k;


	//calculates the speed for each motor
	double front_left = 1 / A.determinant() * (U3 * d * b + U4 * k);
	double front_right = 1 / A.determinant() * (U2 * -d * b + U4 * -k);
	double back_left = 1 / A.determinant() * (U3 * -d * b + U4 * k);
	double back_right = 1 / A.determinant() * (U2 * d * b + U4 * -k);


	//old method of rot speed

	/*
	MatrixXd Omega(4, 1);	// needs to be complex to account for negative under sqrt()
	Omega << sqrt(1 / (4 * b) * U1 - 1 / (2 * b * l) * U3 - 1 / (4 * d) * U4),
		sqrt(1 / (4 * b) * U1 - 1 / (2 * b * l) * U2 + 1 / (4 * d) * U4),
		sqrt(1 / (4 * b) * U1 + 1 / (2 * b * l) * U3 - 1 / (4 * d) * U4),
		sqrt(1 / (4 * b) * U1 + 1 / (2 * b * l) * U2 + 1 / (4 * d) * U4);//[rad / s]
	*/

	// The angular rates are converted into the voltage values
	Vector4d Voltages;
	MatrixXd I(4, 4);
	I.setIdentity();


	Voltages(0) = constant * front_left;// [V]
	Voltages(0) = constant * front_right;
	Voltages(0) = constant * back_left;
	Voltages(0) = constant * back_right;

	//Voltage to PWM:
	//duty_cycle x V_battery = V_effective

	PWM PWMsignal1, PWMsignal2, PWMsignal3, PWMsignal4;

	PWMsignal1.enable;
	PWMsignal1.init(0);		//channel 0 = physical channel 1
	PWMsignal1.set_duty_cycle(1, Voltages(0) / V_bat);	//channel , period. 
	PWMsignal1.set_period(1, 200);	//channel, frequency

	PWMsignal2.enable;
	PWMsignal2.init(1);		//channel 1 = physical channel 2
	PWMsignal2.set_duty_cycle(1, Voltages(1) / V_bat);	//channel , period.
	PWMsignal2.set_period(1, 200);	//channel, frequency

	PWMsignal3.enable;
	PWMsignal3.init(2);		//channel 2 = physical channel 3
	PWMsignal3.set_duty_cycle(1, Voltages(2) / V_bat);	//channel ,  period.
	PWMsignal3.set_period(1, 200);	//channel, frequency

	PWMsignal4.enable;
	PWMsignal4.init(3);		//channel 3 = physical channel 4
	PWMsignal4.set_duty_cycle(1, Voltages(3) / V_bat);	//channel ,  period. 
	PWMsignal4.set_period(1, 200);	//channel, frequency

   //this sends PWM signal to these channels I think

	 //******** PWM FUNCTION FROM NAVIO ************

	 //https://docs.emlid.com/navio/navio+/dev/pwm-output
}




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
	double alt = sensor_readings(6);

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
	MatrixXd X_p(12, 1), I(12, 12), P_p(12, 12), K(12, 12);
	I.setIdentity();

	RungeKutta(h, A, X, B, U);//numerical integrates to get simulated state vector

	X_p = X;	//predicted state vector 

	X_p(3) = X(3) + gyro_p * dt;	//rotation angles from gyro
	X_p(4) = X(4) + gyro_q * dt;
	X_p(5) = X(5) + gyro_r * dt;

	X_p(6) = X(6) + accel_x * dt;	//this uses accel_x,y,z to predict states outside sensor matrix.
	X_p(7) = X(7) + accel_y * dt;
	X_p(8) = X(8) + accel_z * dt;
	X_p(0) = X(0) + X(6) * dt + .5 * accel_x * dt * dt;
	X_p(1) = X(1) + X(7) * dt + .5 * accel_y * dt * dt;

	P_p = A * P * A.transpose() + Q;	//Covariance matrix
	K = P_p * C.transpose() * (C * P_p * C.transpose() + R).inverse();	//GAIN matrix

	//UPDATE 
	X = X_p + K * (z_sensor - C * X_p);

	P = (I - K * C) * P_p;


}
