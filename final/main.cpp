#include "mbed.h"
#include "bbcar.h"


Ticker encoder_left_ticker;
Ticker encoder_right_ticker;
Ticker distance_ticker;
Timer turn_timer;
DigitalIn pin8(D8), pin9(D9);
BufferedSerial pc(USBTX, USBRX); //tx,rx
BufferedSerial uart(D1, D0); //tx,rx
BufferedSerial xbee(A1, A0);
Ticker servo_ticker;
PwmOut pin5(D10), pin6(D11);



Thread getData;
Thread dis;
Thread heading_thread;
Thread function_thread; // 0: line follow 1: find April
EventQueue dis_queue(32 * EVENTS_EVENT_SIZE);

BBCar car(pin5, pin6, servo_ticker, pin8, pin9, encoder_left_ticker, encoder_right_ticker, turn_timer);

void recordDir();
void readData();

void special_function();

int state = 0;
int read_num = 0;
int TX, TY, TZ, RX, RY, RZ;
int ID, X1, X2, Y1, Y2, THETA, RHO;
int counter = 0;
int dataUpdateL = 0, dataUpdateA = 0;

int total_speed = 125;

void outputDistance();
void distance_queue();

static int function_control;
int prependicular_control;
int task_finish;

bool have_airtag;
bool have_line;


int main(){
    getData.start(readData);
    heading_thread.start(recordDir);
    BSP_GYRO_Init();
    turn_timer.start();
    function_thread.start(special_function);
    char buff[20];
    sprintf(buff, "follow the line\r\n");
    xbee.write(buff, 17);

    //while(1) {
        function_control = 0;
        while(function_control == 0) {
            //printf("TZ: %d tag: %d\n", -TZ, have_airtag);
            if (-TZ < 35 && -TZ > 0) {
                function_control = 3;
            }
            ThisThread::sleep_for(200ms);
        }
        sprintf(buff, "end of line\r\n");
        xbee.write(buff, 13);
        ThisThread::sleep_for(100ms);
        sprintf(buff, "turn 90 degree\r\n");
        xbee.write(buff, 16);
        car.turnAngle(90);
        sprintf(buff, "turn complete\r\n");
        xbee.write(buff, 15);
        sprintf(buff, "cross barrier\r\n");
        xbee.write(buff, 15);
        float now_angle = car.rz();
        while(abs(now_angle - car.rz()) < 130) {
            car.navi_angle(125, -40);
            ID = 0;
        }
        car.stop();
        ThisThread::sleep_for(500ms);
        sprintf(buff, "cross finish\r\n");
        xbee.write(buff, 14);
        sprintf(buff, "align apriltag\r\n");
        xbee.write(buff, 16);
        function_control = 2;
        printf("fun: %d\n", function_control);
        while(function_control == 2) {
            ThisThread::sleep_for(200ms);
        }
        sprintf(buff, "align finish\r\n");
        xbee.write(buff, 14);
        ThisThread::sleep_for(100ms);
        function_control = 3;
        sprintf(buff, "turn back\r\n");
        xbee.write(buff, 11);
        car.turnAngle(170);
        sprintf(buff, "turn finish\r\n");
        xbee.write(buff, 14);
        TZ = 0;
        sprintf(buff, "follow the line\r\n");
        xbee.write(buff, 17);
        ThisThread::sleep_for(500ms);
        function_control = 0;
        printf("fun: %d\n", function_control);
        while(function_control == 0) {
            //printf("TZ: %d tag: %d\n", -TZ, have_airtag);
            if (-TZ < 35 && -TZ > 0) {
                function_control = 3;
            }
            ThisThread::sleep_for(200ms);
        }
        sprintf(buff, "complete\r\n");
        xbee.write(buff, 10);
        car.stop();
        /*ThisThread::sleep_for(100ms);
        function_control = 3;
        printf("fun: %d\n", function_control);
        while(function_control == 3) {
            ThisThread::sleep_for(200ms);
        }
        ThisThread::sleep_for(100ms);*/
    //}
}

void special_function() {
    int preDataL = 0, preDataA = 0;;
    while (true) {
        printf("special function %d\n", function_control);
        while (function_control == 0 && preDataL != dataUpdateL) {
            preDataL = dataUpdateL;
            int Center = (X1 + X2) / 2;
            if (Center < 75 || Center > 85) {
                double ratio = Center / 160.0;
                double left_speed = total_speed * ratio;
                double right_speed = total_speed - left_speed;
                //printf("left: %f right: %f\n", left_speed, right_speed);
                car.go(right_speed, left_speed);
            } else {
                float target_dir;
                if (THETA < 90) {
                    target_dir = THETA;
                } else {
                    target_dir = 180 - THETA;
                }
                printf("tar_dir: %f\n", target_dir);
                car.navi_angle(total_speed, target_dir, 0.3);
            }
            //function_control = 100;
        } 
        while (function_control == 1 && preDataA != dataUpdateA) {
            preDataA = dataUpdateA;
            printf("ID: %d TX: %d TY: %d TZ: %d RX: %d RY: %d RZ: %d\n", ID, TX, TY, TZ, RX, RY, RZ);
            double turn_angle = atan(-TX * 1.0 / TZ) * 54.7;
            printf("turn_angle: %f\n", turn_angle);
            if (abs(turn_angle) > 6) {
                car.turnAngle(turn_angle);
            } else {
                printf("get off\n");
                function_control = 100;
            }
        } 
        while (function_control == 2 && preDataA != dataUpdateA) {
            preDataA = dataUpdateA;
            double turn_angle = atan(-TX * 1.0 / TZ) * 54.7;
            printf("turn_angle: %f\n", turn_angle);
            if (abs(turn_angle) > 6) {
                car.turnAngle(turn_angle);
            } else if (abs(turn_angle) < 6) {
                if ((RY > 10 && RY < 90) || (RY < 350 && RY > 270)) {
                    printf("Go perpendicular RY: %d TZ : %d\n", RY, -TZ);
                    double cor_angle;
                    int cor_length;
                    bool dir = false;
                    if (RY > 270) {
                        cor_angle = (90 + (RY - 360));
                        cor_length = -TZ * sin((360 - RY) / 57.4);
                    } else {
                        cor_angle = -90 + 2 * RY;
                        cor_length = -TZ * sin((RY) / 57.4);
                        dir = true;
                    }
                    printf("Correct angle: %f length: %d\n", cor_angle, cor_length);
                    car.turnAngle(cor_angle);
                    ThisThread::sleep_for(700ms);
                    car.goStraightCm(cor_length);
                    ThisThread::sleep_for(700ms);
                    if (dir) {
                        car.turnAngle(90);
                    } else {
                        car.turnAngle(-90);
                    }
                } else if (ID == 10) {
                    function_control = 100;
                }
            }
        } 
        while (function_control == 100) {
            car.stop();
        }
        //car.stop();
        //ThisThread::sleep_for(200ms);
    }
}

void recordDir() {
    while(true) {
        car.heading_log();
    }
}

void outputDistance() {
    dis_queue.call(distance_queue);
}

void distance_queue() {
    char dis[20];
    sprintf(dis, "Distance: %3d\r\n", -TZ);
    printf("%s\n", dis);
    xbee.write(dis, 15);
}

void readData() {
    uart.set_baud(9600);
    while(1){
        if(uart.readable()){
            char recv[1];
            char num3[3], num4[4];
            uart.read(recv, sizeof(recv));
            if (state == 1) {
                if (read_num == 0) {
                    num3[counter++] = recv[0];
                    if (counter == 3) {
                        //printf("ID: %c%c%c ", num3[0], num3[1], num3[2]);
                        ID = atoi(num3);
                        read_num++;
                        counter = 0;
                    }
                } else if (read_num == 1) {
                    num4[counter++] = recv[0];
                    if (counter == 4) {
                        //printf("TX: %c%c%c%c ", num4[0], num4[1], num4[2], num4[3]);
                        TX = atoi(num4);
                        read_num++;
                        counter = 0;
                    }
                } else if (read_num == 2) {
                    num4[counter++] = recv[0];
                    if (counter == 4) {
                        //printf("TY: %c%c%c%c ", num4[0], num4[1], num4[2], num4[3]);
                        TY = atoi(num4);
                        read_num++;
                        counter = 0;
                    }
                } else if (read_num == 3) {
                    num4[counter++] = recv[0];
                    if (counter == 4) {
                        //printf("TZ: %c%c%c%c ", num4[0], num4[1], num4[2], num4[3]);
                        TZ = atoi(num4);
                        read_num++;
                        counter = 0;
                    }
                } else if (read_num == 4) {
                    num3[counter++] = recv[0];
                    if (counter == 3) {
                        //printf("RX: %c%c%c ", num3[0], num3[1], num3[2]);
                        RX = atoi(num3);
                        read_num++;
                        counter = 0;
                    }
                } else if (read_num == 5) {
                    num3[counter++] = recv[0];
                    if (counter == 3) {
                        //printf("RY: %c%c%c ", num3[0], num3[1], num3[2]);
                        RY = atoi(num3);
                        read_num++;
                        counter = 0;
                    }
                } else if (read_num == 6) {
                    num3[counter++] = recv[0];
                    if (counter == 3) {
                        //printf("RZ: %c%c%c ", num3[0], num3[1], num3[2]);
                        RZ = atoi(num3);
                        read_num++;
                        counter = 0;
                        state = 3;
                    }
                }
                
            }
            if (state == 2) {
                if (read_num == 0) {
                    num3[counter++] = recv[0];
                    if (counter == 3) {
                        //printf("X1: %c%c%c ", num[0], num[1], num[2]);
                        X1 = atoi(num3);
                        read_num++;
                        counter = 0;
                    }
                } else if (read_num == 1) {
                    num3[counter++] = recv[0];
                    if (counter == 3) {
                        //printf("X2: %c%c%c ", num[0], num[1], num[2]);
                        X2 = atoi(num3);
                        read_num++;
                        counter = 0;
                    }
                } else if (read_num == 2) {
                    num3[counter++] = recv[0];
                    if (counter == 3) {
                        //printf("Y1: %c%c%c ", num[0], num[1], num[2]);
                        Y1 = atoi(num3);
                        read_num++;
                        counter = 0;
                    }
                } else if (read_num == 3) {
                    num3[counter++] = recv[0];
                    if (counter == 3) {
                        //printf("Y2: %c%c%c ", num[0], num[1], num[2]);
                        Y2 = atoi(num3);
                        read_num++;
                        counter = 0;
                    }
                } else if (read_num == 4) {
                    num4[counter++] = recv[0];
                    if (counter == 4) {
                        //printf("THETA: %c%c%c%c ", num[0], num[1], num[2], num[3]);
                        THETA = atoi(num4);
                        read_num++;
                        counter = 0;
                    }
                } else if (read_num == 5) {
                    num4[counter++] = recv[0];
                    if (counter == 4) {
                        //printf("RHO: %c%c%c%c ", num[0], num[1], num[2], num[3]);
                        RHO = atoi(num4);
                        read_num++;
                        counter = 0;
                        state = 4;
                    }
                }
                
            }
            if (recv[0] == 'a' && state == 0) {
                state = 1;
                read_num = 0;
                counter = 0;
            } else if (recv[0] == 'l' && state == 0) {
                state = 2;
                read_num = 0;
                counter = 0;
            } else if (recv[0] == 'e' && state == 3) {
                dataUpdateA++;
                //printf("\nTX: %d TY: %d TZ: %d RX: %d RY: %d RZ: %d\n", TX, TY, TZ, RX, RY, RZ);
                //printf("X1: %d X2: %d Y1: %d Y2: %d THETA: %d RHO: %d\n", X1, X2, Y1, Y2, THETA, RHO);
                state = 0;
            } else if (recv[0] == 'e' && state == 4) {
                dataUpdateL++;
                //printf("\nTX: %d TY: %d TZ: %d RX: %d RY: %d RZ: %d\n", TX, TY, TZ, RX, RY, RZ);
                //printf("X1: %d X2: %d Y1: %d Y2: %d THETA: %d RHO: %d\n", X1, X2, Y1, Y2, THETA, RHO);
                state = 0;
            }
            //pc.write(recv, sizeof(recv));
        } 
        /*else {
            TX = 0; TY = 0; TZ = 0; RX = 0; RY = 0; RZ = 0;
            X1 = 0; X2 = 0; Y1 = 0; Y2 = 0; THETA = 0; RHO = 0;
        }*/
    }
}