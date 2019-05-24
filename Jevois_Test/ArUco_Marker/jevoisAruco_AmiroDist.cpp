#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <fcntl.h>    
#include <string.h>
#include <string>
#include <math.h>

//#define MOTORCONTROL

#ifdef MOTORCONTROL
#include <ControllerAreaNetwork.h>
#endif

#define MAXBYTES 200
#define MAX_SPEED 70000
#define MIN_SPEED 25000

//Define Marker
typedef struct {
    int id;
    double x;
    double y;
    double z;
    double markerlen;
} Marker_t;

using namespace std;


int main(int argc, const char *argv[])
{
    //Start Jevois Marker Detection Algortihm
    system("echo \"streamoff\" > /dev/ttyACM0");
    system("echo \"setpar serout USB\" > /dev/ttyACM0");
    system("echo \"setmapping 5\" > /dev/ttyACM0");
    system("echo \"setpar serstyle Normal\" > /dev/ttyACM0");
    //parameter: 2D coordinates: dopose off - 3D: coordinates: dopose on
    system("echo \"setpar dopose True\" > /dev/ttyACM0");
    //parameter to the size (width) in millimeters of your actual physical markers.
    system("echo \"setpar markerlen 50\" > /dev/ttyACM0");
    system("echo \"streamon\" > /dev/ttyACM0");

    //Variables for Reading Jevois Output
    char buf;
    char buffer[MAXBYTES];
    int n = 0;
    char delimiter[] = " ";
    char *ptr;
    int c;
    int count;

    //Parameter for ArUco Marker Tracking
    double P = 1000;
    double dist_desired = 150;
    int Marker_ID = 42;
    if (argc >= 2) {
      P = atof(argv[1]);
    }
    if (argc >= 3) {
      dist_desired = atof(argv[2]);
    }
    if (argc >= 4) {
      Marker_ID = atoi(argv[3]);
    }
    printf("Parameter:\tP = %f\t-\tdistance = %f\t-\tMarker_ID = %d\n\n", P, dist_desired, Marker_ID);

    //Variables for Marker Tracking
    double z_last = 0;
    double z = 0;
    double lowpass = 0.5;
    double dead_zone = 5;

    //CAN Objekt for motor control
#ifdef MOTORCONTROL
    ControllerAreaNetwork CAN;
#endif

    //Open Serial Connection to Jevois
    int fd = open("/dev/ttyACM0", O_RDWR);

    //Start Main-Loop
    while(1) {
        //Read every Character
        count = read(fd, &buf, 1);

        //Check if message is completly send
        if (buf == '\n') { 
            //printf("buffer: %s\n", buffer);

            //Split Message
            ptr = strtok(buffer, delimiter);

            if (ptr == NULL) {
                printf("\nError\n");
                continue;
            }
            //When detect Start element of message --> decode message
            if (strcmp(ptr,"N3") == 0) {
                c = 0;
                Marker_t marker;
                //Decode Message
                while (ptr != NULL) {
                    ptr = strtok(NULL, delimiter);
                    if (c == 0) {
                        //Marker ID without the character "U"
                        marker.id = atoi(ptr+1);
                    } else if (c == 1) {
                        marker.x = atof(ptr);
                    } else if (c == 2) {
                        marker.y = atof(ptr);
                    } else if (c == 3) {
                        marker.z = atof(ptr);
                    } else if (c == 4) {
                        marker.markerlen = atof(ptr);
                        break;
                    } else {
                        break;
                    }
                    c++;
                }
                //printf("ID: %d, X: %f, Y: %f, Z: %f, LEN: %f\n", marker.id, marker.x, marker.y, marker.z, marker.markerlen);

                //Do Tracking, when correct marker is detected
                if (marker.id == Marker_ID) {
                  //P-Controller
                  z = lowpass * marker.z + (1 - lowpass) * z_last;
                  double diff = z - dist_desired;
                  int speed = P * diff;
                  //Speed Range
                  speed = max(-MAX_SPEED, min(MAX_SPEED, speed));
                  if (abs(speed) < MIN_SPEED) {
                    speed = MIN_SPEED * (speed / abs(speed));
                  }
                  //Dead Zone
                  if (abs((int)diff) < dead_zone) {
                    speed = 0;
                  }
                  z_last = z;

                  printf("ID: %d, X: %f, Y: %f, Z: %f, speed: %d\n", marker.id, marker.x, marker.y, z, speed);

                  //Set Linear Motor speed
#ifdef MOTORCONTROL
                  CAN.setTargetSpeed(speed,0);
#endif
                } else {
                    printf("Marker with wrong ID detected. Detected ID: %d - Expected ID: %d\n", marker.id, Marker_ID);
                }

            }
            n = 0;
            memset(buffer, 0, MAXBYTES);
        } else {
            buffer[n] = buf;
            n++;
            if (n > MAXBYTES) {
              n = 0;
              memset(buffer, 0, MAXBYTES);
            }
        }
    }

    return 0;
}
