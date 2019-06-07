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

#define MOTORCONTROL

#ifdef MOTORCONTROL
#include <ControllerAreaNetwork.h>
#endif

#define MAXBYTES 200
#define MAX_SPEED 20000
#define MIN_SPEED 25000
#define PI        3.14159265358979323846

using namespace std;

//Functions
bool DecodeJevoisMarker(char buffer[]);
bool SelectMarkerFromBuffer(Marker_t MarkerBuffer[], unsigned int marker_idx, int Marker_ID);
float timedifference_msec(struct timeval t0, struct timeval t1);

//Aruco actual Marker
int desired_Marker_ID = 42;
Marker_t DecodedMarker;
int actual_ID = -1;
const int MarkerBufferSize = 10;
bool BufferFilled = false;

//Marker ID, when Start/Stop tracking process
int const Start_Stop_ID = 0;

//Planner States
plannerStates state = IDLE;
plannerStates nextState = IDLE;

//True, when 360 deg Searching process is done
double TurningAngularSpeed = 1000000.0 * PI * 0.05;
bool Turning_Done = false;

//for P-Controller
const double angular_thresh = 5;
const double linear_thresh = 10;
double Marker_X_Pos = 1000;
double linear_diff = 1000;

double P = 1000;
double dist_desired = 300;

const double lowpass_linear = 0.5;
const double lowpass_angular = 0.5;

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
        P = atof(argv[1]);
    }
    if (argc >= 3)
    {
        dist_desired = atof(argv[2]);
    }
    if (argc >= 4)
    {
        desired_Marker_ID = atoi(argv[3]);
    }
    printf("Parameter:\tP = %f\t-\tdistance = %f\t-\tMarker_ID = %d\n\n", P, dist_desired, desired_Marker_ID);

    //Variables for Marker Tracking
    double z_last = 0;
    double z = 0;
    Marker_t MarkerBuffer[MarkerBufferSize];
    int marker_idx = -1;
    bool marker_tracked = false;

    //Variables for time measurement
    struct timeval t0;
    struct timeval t1;
    float time_diff = 0;
    struct timeval tMarkerStart;
    struct timeval tMarkerEnd;
    float time_marker_diff = 0;

    //CAN Objekt for motor control
#ifdef MOTORCONTROL
    ControllerAreaNetwork CAN;
    CAN.setLightBrightness(0);
#endif

    //Open Serial Connection to Jevois
    int fd = open("/dev/ttyACM0", O_RDWR | O_NOCTTY);

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

        //Check if message is completly send
        if (buf == '\n')
        {
            //Marker Buffer idx
            marker_idx++;
            if (marker_idx >= 10)
                marker_idx = 0;

            //Decode Message
            bool marker_found = DecodeJevoisMarker(buffer);
            
            
            if(marker_found == true) {
                gettimeofday(&tMarkerStart, 0);
                if(marker_tracked == false){
                    //printf("Marker_ID: %d\n",DecodedMarker.id);
                }
                marker_tracked = true;
                actual_ID = DecodedMarker.id;
                Marker_X_Pos = DecodedMarker.x;
                linear_diff = DecodedMarker.z - dist_desired;
                
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
            if (actual_ID == Start_Stop_ID)
            {
                printf("State=SEARCH\n");
                sleep(2);
                nextState = SEARCH;
            }
            break;
        }
        case SEARCH:
        {
            if (actual_ID == desired_Marker_ID)
            {
                printf("State=MIDLLE_TURN\n");
                nextState = MIDLLE_TURN;
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
                printf("State=MIDLLE_TURN\n");
                nextState = MIDLLE_TURN;
                Turning_Done = false;
            }
            else if (Turning_Done == true)
            {
                printf("State=STOP because Turning\n");
                nextState = STOP;
            }
            break;
        }
        case MIDLLE_TURN:
        {
            if (actual_ID != desired_Marker_ID)
            {
                printf("State=SEARCH\n");
                nextState = SEARCH;
            }
            else if (fabs(Marker_X_Pos) <= angular_thresh)
            {
                printf("State=DRIVE_TO_TARGET\n");
                nextState = DRIVE_TO_TARGET;
            }
            break;
        }
        case DRIVE_TO_TARGET:
        {
            if (actual_ID != desired_Marker_ID)
            {
                printf("State=SEARCH\n");
                nextState = SEARCH;
            }
            else if (fabs(linear_diff) <= linear_thresh)
            {
                printf("State=TARGET\n");
                nextState = TARGET;
            }
            break;
        }
        case TARGET:
        {
            if (actual_ID != desired_Marker_ID || fabs(Marker_X_Pos) > angular_thresh || fabs(linear_diff) > linear_thresh)
            {
                printf("State=SEARCH\n");
                nextState = SEARCH;
            } 
            break;
        }
        case STOP:
        {
            if (actual_ID == Start_Stop_ID) {
                printf("State=SEARCH\n");
                sleep(2);
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
#ifdef MOTORCONTROL
            CAN.setTargetSpeed(0,0);
#endif
            //printf("State=IDLE\n");
            break;
        }
        case SEARCH:
        {
            Turning_Done = false;
#ifdef MOTORCONTROL
            CAN.setTargetSpeed(0,0);
#endif
            //printf("State=SEARCH\n");
            break;
        }
        case START_SEARCH_TURN:
        {
            
            //Start Timer new
            gettimeofday(&t0, 0);
#ifdef MOTORCONTROL
            CAN.setTargetSpeed(0, int(TurningAngularSpeed));
#endif
            break;
        }
        case SEARCH_TURN:
        {
            gettimeofday(&t1, 0);
            time_diff = timedifference_msec(t0, t1);
            //Stop, when Searching 360deg is done
            //printf("Turning %f\n",((TurningAngularSpeed * 0.000001) * (time_diff * 0.001)));
            if(((TurningAngularSpeed * 0.000001) * (time_diff * 0.001)) >= (2 * PI)) {
                Turning_Done = true;
            }

            //printf("State=SEARCH_TURN\n");
            break;
        }
        case MIDLLE_TURN:
        {
            if(Marker_X_Pos < 0) {
#ifdef MOTORCONTROL
                CAN.setTargetSpeed(0, int(TurningAngularSpeed));
#endif 
            } else {
#ifdef MOTORCONTROL
                CAN.setTargetSpeed(0, int(-TurningAngularSpeed));
#endif 
            }
            printf("distance to middle turn: %f\n", Marker_X_Pos);
            //printf("State=MIDLLE_TURN\n");
            break;
        }
        case DRIVE_TO_TARGET:
        {
            if(linear_diff < 0) {
#ifdef MOTORCONTROL
                CAN.setTargetSpeed(-MAX_SPEED, 0);
#endif 
            } else {
#ifdef MOTORCONTROL
                CAN.setTargetSpeed(MAX_SPEED, 0);
#endif 
            }
            printf("distance to middle drive: %f - %f - %f\n", DecodedMarker.z, dist_desired, linear_diff);
            //printf("State=DRIVE_TO_TARGET\n");
            break;
        }
        case TARGET:
        {
#ifdef MOTORCONTROL
            CAN.setTargetSpeed(0,0);
#endif
            break;
        }
        case STOP:
        {
#ifdef MOTORCONTROL
            CAN.setTargetSpeed(0,0);
#endif
            //printf("State=STOP\n");
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


bool SelectMarkerFromBuffer(Marker_t MarkerBuffer[], unsigned int marker_idx, int Marker_ID){
    bool correct_Marker = false;
    int idx = marker_idx;

    for(int i = 0; i < MarkerBufferSize; i++){

        if(idx < 0) {
            idx = idx + MarkerBufferSize;
        }

        DecodedMarker = MarkerBuffer[idx];

        if(MarkerBuffer[idx].id == Marker_ID) {
            correct_Marker = true;   
            break;  
        }

        if(MarkerBuffer[idx].id == Start_Stop_ID) {
            correct_Marker = true;
            break;
        }

        idx--;
    }
    return correct_Marker;
}

float timedifference_msec(struct timeval t0, struct timeval t1)
{
    return (t1.tv_sec - t0.tv_sec) * 1000.0f + (t1.tv_usec - t0.tv_usec) / 1000.0f;
}
