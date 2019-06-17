#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <fcntl.h>
#include <string.h>
#include <string>
#include <math.h>
#include <sys/time.h>
#include <time.h>
#include <termios.h>	

#include "amiroArucoGlobal.hpp"

#include <ControllerAreaNetwork.h>


#define MAXBYTES        200
#define PI              3.14159265358979323846

#define MAX_SPEED       80000
#define MIN_SPEED       30000

#define MAX_ROT_SPEED   1000000 * 2 * PI * 0.1
#define MIN_ROT_SPEED   1000000 * 2 * PI * 0.03

using namespace std;

//Functions
bool DecodeJevoisMarker(char buffer[]);
double Controller_Pos(double u, double P);
double Controller_Angluar(double u, double P);
float timedifference_msec(struct timeval t0, struct timeval t1);

//Aruco actual detected Marker
Marker_t DecodedMarker;
int actual_ID = -1;
//Tracking Marker ID
int desired_Marker_ID = 42;
//Marker ID, when Start/Stop tracking process
int start_Marker_ID = 0;

//True, when 360 deg Searching process is done
bool Turning_Done = false;
int Turning_Rounds = 1;

//Accuracy in Tracking
const double angular_thresh = 8; //in mm
const double linear_thresh = 10; //in mm
const double angular_thresh_target = 8;
const double linear_thresh_target = 10;

//P-Controller:
double Marker_X_Pos = 1000;
double linear_diff = 1000;

double P_linear = 500;
double P_angular = 5000;
double disired_dist = 250;

const double lowpass_linear = 0.5;
const double lowpass_angular = 0.5;

//Planner States
plannerStates state = IDLE;
plannerStates nextState = IDLE;

int main(int argc, const char *argv[])
{
    //Start Jevois Marker Detection Algortihm
    system("echo streamoff > /dev/ttyACM0");
    system("echo streamoff > /dev/ttyACM0");
    system("echo streamoff > /dev/ttyACM0");
    system("echo streamoff > /dev/ttyACM0");
    system("echo setpar serout USB > /dev/ttyACM0");
    system("echo setmapping 5 > /dev/ttyACM0");
    system("echo setpar serstyle Normal > /dev/ttyACM0");
    //parameter: 2D coordinates: dopose off - 3D: coordinates: dopose on
    system("echo setpar dopose True > /dev/ttyACM0");
    //parameter to the size (width) in millimeters of your actual physical markers.
    system("echo setpar markerlen 94 > /dev/ttyACM0");
    system("echo streamon > /dev/ttyACM0");
    system("echo streamon > /dev/ttyACM0");
    system("echo streamon > /dev/ttyACM0");
    system("echo streamon > /dev/ttyACM0");

    //Variables for Reading Jevois Output Strings
    char buf;
    char buffer[MAXBYTES];
    int n = 0;
    int count;

    //Parameter for ArUco Marker Tracking
    if (argc >= 2)
    {
        P_linear = atof(argv[1]);
    }
    if (argc >= 3)
    {
        P_angular= atof(argv[2]);
    }
    if (argc >= 4)
    {
        disired_dist = atof(argv[3]);
    }
    if (argc >= 5)
    {
        desired_Marker_ID = atoi(argv[4]);
    }
    if (argc >= 6)
    {
        start_Marker_ID = atoi(argv[5]);
    }
    printf("Parameter:\tP_lin = %f\t-\tP_ang = %f\t-\tdistance = %f\t-\tMarker_ID = %d\t-\tStart_ID = %d\n\n", P_linear, P_angular, disired_dist, desired_Marker_ID, start_Marker_ID);

    //Variables for Marker Tracking
    bool marker_tracked = false;

    //Variables for time measurement
    struct timeval t0;
    struct timeval t1;
    float time_diff = 0;
    struct timeval tMarkerStart;
    struct timeval tMarkerEnd;
    float time_marker_diff = 0;

    bool AngularMove = false;
    bool LinearMove = false;

    //CAN Objekt for motor control
    ControllerAreaNetwork CAN;

    //Set Lights off
    CAN.setLightBrightness(0);

    //Open Serial Connection to Jevois
    int fd = open("/dev/ttyACM0", O_RDWR | O_NOCTTY);
    //Set Options for serial connection to jevois
    struct termios options;
	tcgetattr(fd, &options);
	options.c_cflag = B9600 | CS8 | PARENB | CLOCAL | CREAD;		//<Set baud rate
	options.c_iflag = IGNPAR;
	options.c_oflag = 0;
	options.c_lflag = 0;
    options.c_cc[VMIN] = 0;
	tcflush(fd, TCIFLUSH);
	tcsetattr(fd, TCSANOW, &options);

    //Start Main-Loop
    gettimeofday(&tMarkerStart, 0);
    while (1)
    {
        //Read every Character
        count = read(fd, &buf, 1);
        //printf("char: %c\n",buf);

        //Check if message is completly send and decode jevois string
        if (buf == '\n')
        {
            //Decode Message
            bool marker_found = DecodeJevoisMarker(buffer);
            //printf("ID: %d",DecodedMarker.id);
            
            if(marker_found == true) {
                gettimeofday(&tMarkerStart, 0);
                if(marker_tracked == false){
                    //printf("Marker_ID: %d\n",DecodedMarker.id);
                }
                marker_tracked = true;
                actual_ID = DecodedMarker.id;
                Marker_X_Pos = DecodedMarker.x;
                linear_diff = DecodedMarker.z - disired_dist;
                
            } else {
                gettimeofday(&tMarkerEnd, 0);
                time_marker_diff = timedifference_msec(tMarkerStart, tMarkerEnd);
                if (time_marker_diff >= 500) {
                    if(marker_tracked == true){
                        //printf("no marker\n");
                    }
                    marker_tracked = false;
                    actual_ID = -1;
                }
            }

            //Reset message buffer
            n = 0;
            memset(buffer, 0, MAXBYTES);
        }
        else
        {
            buffer[n] = buf;
            n++;
            if (n > MAXBYTES)
            {
                n = 0;
                memset(buffer, 0, MAXBYTES);
            }
            gettimeofday(&tMarkerEnd, 0);
            time_marker_diff = timedifference_msec(tMarkerStart, tMarkerEnd);
            if (time_marker_diff >= 500) {
                if(marker_tracked == true){
                    //printf("no marker\n");
                }
                marker_tracked = false;
                actual_ID = -1;
            }
        }

        //Planer Switch
        switch (state)
        {
        case IDLE:
        {
            if (actual_ID == start_Marker_ID)
            {
                printf("State=SEARCH\n");
                nextState = SEARCH;
            }
            break;
        }
        case SEARCH:
        {
            if (actual_ID == desired_Marker_ID)
            {
                printf("State=DRIVE_MIDDLE_TO_TARGET\n");
                nextState = DRIVE_MIDDLE_TO_TARGET;
            }
            else
            {
                printf("State=START_SEARCH_TURN\n");
                nextState = START_SEARCH_TURN;
            }
            break;
        }
        case START_SEARCH_TURN: 
        {           
            printf("State=SEARCH_TURN\n");
            nextState = SEARCH_TURN;
            break;
        }
        case SEARCH_TURN:
        {
            if (actual_ID == desired_Marker_ID)
            {
                printf("State=DRIVE_MIDDLE_TO_TARGET\n");
                nextState = DRIVE_MIDDLE_TO_TARGET;
                Turning_Done = false;
            }
            else if (Turning_Done == true)
            {
                printf("State=IDLE\n");
                nextState = IDLE;
            }
            break;
        }
        case DRIVE_MIDDLE_TO_TARGET:
        {
            AngularMove = true;
            LinearMove = true;
            if (actual_ID != desired_Marker_ID)
            {
                printf("State=SEARCH\n");
                nextState = SEARCH;
            }
            else 
            {
                if (fabs(Marker_X_Pos) <= angular_thresh) {
                    AngularMove = false;
                }
                if(fabs(linear_diff) <= linear_thresh) {
                    LinearMove = false;
                }

                if (AngularMove == false &&  LinearMove == false) {
                    nextState = TARGET;
                }
            }
            break;
        }
        case TARGET:
        {
            if (actual_ID != desired_Marker_ID || fabs(Marker_X_Pos) > angular_thresh_target || fabs(linear_diff) > linear_thresh_target)
            {
                printf("State=SEARCH\n");
                nextState = SEARCH;
            } 
            break;
        }
        }

        //Planner Controller
        switch (state)
        {
        case IDLE:
        {
            CAN.setTargetSpeed(0,0);
            break;
        }
        case SEARCH:
        {
            Turning_Done = false;
            CAN.setTargetSpeed(0,0);
            break;
        }
        case START_SEARCH_TURN:
        {
            //Start Timer new
            gettimeofday(&t0, 0);
            CAN.setTargetSpeed(0, int(MAX_ROT_SPEED));
            break;
        }
        case SEARCH_TURN:
        {
            //Get new timestamp
            gettimeofday(&t1, 0);
            time_diff = timedifference_msec(t0, t1);
            CAN.setTargetSpeed(0, int(MAX_ROT_SPEED));
            //Stop, when Searching 360deg is done
            //printf("Turning %f\n",((MAX_ROT_SPEED * 0.000001) * (time_diff * 0.001)));
            if(((MAX_ROT_SPEED * 0.000001) * (time_diff * 0.001)) >= (2 * PI * Turning_Rounds)) {
                Turning_Done = true;
            }
            break;
        }
        case DRIVE_MIDDLE_TO_TARGET:
        {           
            //Turn Marker in the middle of the cam frame
            double angular_speed = Controller_Angluar(fabs(Marker_X_Pos), P_angular); 
            if(Marker_X_Pos > 0) {
                angular_speed = -angular_speed;
            }

            if(AngularMove == false) {
                angular_speed = 0;
            }

            
            double linear_speed = Controller_Pos(fabs(linear_diff),P_linear);
            if(linear_diff < 0) {
                linear_speed = -linear_speed;
            }

            if(LinearMove == false) {
                linear_speed = 0;
            }

            CAN.setTargetSpeed(linear_speed, angular_speed);

            //printf("distance to middle turn: %f\n", Marker_X_Pos);
            //printf("distance to middle drive: %f - %f - %f\n", DecodedMarker.z, disired_dist, linear_diff);
            break;
        }
        case TARGET:
        {
            CAN.setTargetSpeed(0,0);
            break;
        }
        }
          state = nextState;
    }

    return 0;
}




bool DecodeJevoisMarker(char buffer[])
{
    //printf("buffer: %s\n", buffer);

    //Split Message
    DecodedMarker.id = -1;
    char *ptr = strtok(buffer, delimiter);

    if (ptr == NULL)
    {
        return false;
    }

    //When detect Start element of message --> decode message
    int c = 0;
    if (strcmp(ptr, "N3") == 0)
    {
        //Decode Message
        while (ptr != NULL)
        {
            ptr = strtok(NULL, delimiter);
            if (c == 0)
            {
                //Marker ID without the character "U"
                DecodedMarker.id = atoi(ptr + 1);
            }
            else if (c == 1)
            {
                DecodedMarker.x = atof(ptr);
            }
            else if (c == 2)
            {
                DecodedMarker.y = atof(ptr);
            }
            else if (c == 3)
            {
                DecodedMarker.z = atof(ptr);
            }
            else if (c == 4)
            {
                DecodedMarker.markerlen = atof(ptr);
                break;
            }
            else
            {
                break;
            }
            c++;
        }
    }

    if (c < 4)
    {
        return false;
    }
    else
    {
        return true;
    }
}


float timedifference_msec(struct timeval t0, struct timeval t1)
{
    return (t1.tv_sec - t0.tv_sec) * 1000.0f + (t1.tv_usec - t0.tv_usec) / 1000.0f;
}


double Controller_Pos(double u, double P) {
    double motor_output = u * P;

    if (motor_output >= MAX_SPEED) {
        motor_output = MAX_SPEED;
    } else if(motor_output <= MIN_SPEED) {
        motor_output = MIN_SPEED;
    }

    return motor_output;
}


double Controller_Angluar(double u, double P) {
    double motor_output = u * P;

    if (motor_output >= MAX_ROT_SPEED) {
        motor_output = MAX_SPEED;
    } else if(motor_output <= MIN_ROT_SPEED) {
        motor_output = MIN_SPEED;
    }

    return motor_output;
}