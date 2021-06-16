#include "bbcar.h"

BBCar::BBCar( PwmOut &pin_servo0, PwmOut &pin_servo1, Ticker &servo_ticker, DigitalIn &pin_encoder_left, DigitalIn &pin_encoder_right, Ticker &encoder_right_ticker, Ticker &encoder_left_ticker, Timer &turn_timer):servo0(pin_servo0), servo1(pin_servo1), encoder_left(pin_encoder_left, encoder_left_ticker), encoder_right(pin_encoder_right, encoder_right_ticker), timer(&turn_timer){
    servo0.set_speed(0);
    servo1.set_speed(0);
    servo_ticker.attach(callback(this, &BBCar::controlWheel), 20ms);
    degreeX = 0;
    degreeY = 0;
    degreeZ = 0;
}

void BBCar::controlWheel(){
    servo0.control();
    servo1.control();
}

void BBCar::stop(){
    servo0.set_factor(1.05);
    servo1.set_factor(1);
    servo0.set_speed(0);
    servo1.set_speed(0);
}

void BBCar::goStraight( double speed ){\
    servo0.set_factor(1.05);
    servo1.set_factor(1);
    servo0.set_speed(speed);
    servo1.set_speed(-speed);
}

void BBCar::setCalibTable( int len0, double pwm_table0[], double speed_table0[], int len1, double pwm_table1[], double speed_table1[] ){
    servo0.set_calib_table(len0, pwm_table0, speed_table0);
    servo1.set_calib_table(len1, pwm_table1, speed_table1);
}
void BBCar::goStraightCalib ( double speed ){
    servo0.set_factor(1.05);
    servo1.set_factor(1);
    servo0.set_speed_by_cm(speed);
    servo1.set_speed_by_cm(-speed);
}

void BBCar::goStraightCm(double cm) {
    encoder_right.reset();
    int speed = (cm > 0) ? 50 : -50;
    while(encoder_right.get_cm() < abs(cm)) {
        encoder_right.count_steps();
        goStraight(speed);
        //printf("step: %d\n", encoder_right.get_steps());
    }
    stop();
}

/*	speed : speed value of servo
    factor: control the speed value with 0~1
            control left/right turn with +/-
*/
void BBCar::turn( double speed, double factor ){
    if(factor>0){
        servo0.set_factor(factor);
        servo1.set_factor(1);
    }
    else if(factor<0){
        servo0.set_factor(1);
        servo1.set_factor(-factor);
    }
    servo0.set_speed(speed);
    servo1.set_speed(-speed);

}

void BBCar::turnAngle(double angle) {
    //show_heading();
    float turn_angle;
    Kp = 2.0; Ki = 1.0; Kd = 0;
    pid_init();

    float degree = rz();
    float target_degree = degree + angle;
    float diff = degree - target_degree;

    while(abs(diff) > 6) {
        //Process the PID control
        float correction = pid_process(diff);
        //bound the value from -0.9 to -.9
        correction = clamp(correction, 0.9, -0.9);
        turn_angle = (angle < 0) ? (1-abs(correction)) : (-1+abs(correction));
        rotate(turn2speed(turn_angle),-turn_angle);
        //ThisThread::sleep_for(100ms);
        degree = rz();
        diff = degree - target_degree;
        printf("degree:%f, target: %f, diff:%f \r\n", degreeZ, angle, diff);
      }
      rotate(turn2speed(turn_angle),turn_angle);
      wait_us(100000);
      //printf("degree:%f, target: %f, diff:%f \r\n", degreeZ, angle, diff);
      stop();
      pid_init();
      //show_heading();
}

void BBCar::show_heading() {
    printf("r: %f %f %f\n", rx(), ry(), rz());
}

void BBCar::heading_log() {
    float pDataXYZ[3] = {0};
    timer->reset();
    BSP_GYRO_GetXYZ(pDataXYZ);
    long int time = duration_cast<microseconds>(timer->elapsed_time()).count();
    pDataXYZ[0] -= 700;
    pDataXYZ[1] += 1960;
    pDataXYZ[2] += 980;
    //printf("pData %f %f %f\n", pDataXYZ[0], pDataXYZ[1], pDataXYZ[2]);
    pDataXYZ[0] /= 4000;
    pDataXYZ[1] /= 4000;
    pDataXYZ[2] /= 4000;
    pDataXYZ[0] *= (float)time / 1000;
    pDataXYZ[1] *= (float)time / 1000;
    pDataXYZ[2] *= (float)time / 1000;
    //printf("pData %f %f %f\n", pDataXYZ[0], pDataXYZ[1], pDataXYZ[2]);
    degreeX += pDataXYZ[0];
    degreeY += pDataXYZ[1];
    degreeZ += pDataXYZ[2];
    ThisThread::sleep_for(50ms);
    //printf("degreeX:%f, degreeY: %f, degreeZ:%f \n", degreeX, degreeY, degreeZ);
}

void BBCar::navi_angle(double total_speed, float dir, float factor) {
    double left_speed = total_speed / 2;
    double right_speed = total_speed / 2;
    double delta_speed = (total_speed / 4) * factor * sin(dir / 57.29);
    left_speed -= delta_speed;
    right_speed += delta_speed;
    go(left_speed, right_speed);
}

void BBCar::go(double left_speed, double right_speed) {
    servo0.set_factor(1);
    servo1.set_factor(1);
    servo0.set_speed(left_speed);
    servo1.set_speed(-right_speed);
}

float BBCar::clamp( float value, float max, float min ){
    if (value > max) return max;
    else if (value < min) return min;
    else return value;
}

int BBCar::turn2speed( float turn ){
    return 25+abs(25*turn);
}

void BBCar::rotate( double speed, double factor ){
    factor *= 7;
    servo0.set_factor(-factor * 1.05);
    servo1.set_factor(factor);
    servo0.set_speed(speed);
    servo1.set_speed(-speed);

}

