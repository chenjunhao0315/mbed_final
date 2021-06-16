#ifndef BBCAR_H
#define BBCAR_H
#include "parallax_servo.h"
#include "parallax_encoder.h"
#include "parallax_ping.h"
#include "stm32l475e_iot01_gyro.h"
using namespace std::chrono;

class BBCar{
	public:
		BBCar( PwmOut &pin_servo0, PwmOut &pin_servo1, Ticker &servo_ticker, DigitalIn &pin_encoder_left, DigitalIn &pin_encoder_right, Ticker &encoder_right_ticker, Ticker &encoder_left_ticker, Timer &turn_timer);

		Timer *timer;

		parallax_servo servo0;
		parallax_servo servo1;

		parallax_encoder encoder_left;
		parallax_encoder encoder_right;

		float degreeX = 0, degreeY = 0, degreeZ = 0;
		void heading_log();
		float rx() {return degreeX;}
		float ry() {return degreeY;}
		float rz() {return degreeZ;}
		void show_heading();
		void navi_angle(double total_speed, float dir, float factor = 1);

		void controlWheel();
		void stop();
		void goStraight( double speed );

		void setCalibTable( int len0, double pwm_table0[], double speed_table0[], int len1, double pwm_table1[], double speed_table1[] );
		void goStraightCalib( double speed );

		void goStraightCm(double cm);
		void go(double left_speed, double right_speed);

		// turn left/right with a factor of speed
		void turn( double speed, double factor );
		void rotate( double speed, double factor );

		void turnAngle( double angle);

		// limit the value by max and min
		float clamp( float value, float max, float min );
		int turn2speed( float turn );

		float state[3] = {0};
		float Kp = 0, Ki = 0, Kd = 0;
		float a0 = 0, a1 = 0, a2 = 0;

		void pid_init(){
   			state[0] = 0;
   			state[1] = 0;
   			state[2] = 0;
   			a0 = Kp + Ki + Kd;
   			a1 = (-Kp) - 2*Kd;
   			a2 = Kd;
		}

		float pid_process(float in){

   			int out = in*a0 + a1*state[0] + a2*state[1] + state[2];
   			state[1] = state[0];
   			state[0] = in;
   			state[2] = out;
   			return out;
		}
};

#endif
