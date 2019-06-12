#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <string.h>
#include <string>
#include <math.h>
#include <termios.h>

#define MOTORCONTROL

#ifdef MOTORCONTROL
#include <ControllerAreaNetwork.h>
#include <Color.h>
#endif

#define MAXBYTES 100
#define PI 3.14159f

using namespace std;

//For mean Calculation of rotation velocity
const int gamma_mean_size = 10;
double gamma_buffer[gamma_mean_size] = {};
unsigned int gamma_index = 0;
double CalculateMean(double new_gamma);

//Output Values from Optical Flow
double COMANV[2];

int main(int argc, const char *argv[])
{
  //Start Jevois Optical Flow Algortihm
  system("echo streamoff > /dev/ttyACM0");
  system("echo streamoff > /dev/ttyACM0");
  system("echo streamoff > /dev/ttyACM0");
  system("echo streamoff > /dev/ttyACM0");
  system("echo setpar serout USB > /dev/ttyACM0");
  system("echo setmapping 9 > /dev/ttyACM0");
  system("echo streamon > /dev/ttyACM0");
  system("echo streamon > /dev/ttyACM0");
  system("echo streamon > /dev/ttyACM0");
  system("echo streamon > /dev/ttyACM0");

  //Parameter for Optical Flow
  double n0 = 37.5; //threshold (optical flow)
  double g = 5.0; //gain (optical flow)
  double k = -3000000.0; //gain (motor)
  double v = 120000.0; //forward velocity
  double alpha = 0.2; //lowpass constant (optical flow)
  if (argc>=2) {
    n0 = atof(argv[1]);
  }
  if (argc>=3) {
    g = atof(argv[2]);
  }
  if (argc>=4) {
    k = -atof(argv[3]);
  }
  if (argc>=5) {
    v = atof(argv[4]);
  }
  if (argc>=6) {
    alpha = atof(argv[5]);
  }
  printf("Parameter:\tn0 = %f\t-\tg = %f\t-\tk = %f\t-\tv = %f\t-\talpha = %f\n\n", n0, g, k, v, alpha);

  //Variables for Reading Jevois Output
  char buf;
  char buffer[MAXBYTES];
  int n = 0;
  int count;
  char delimiter[] = ",";

  //CAN Objekt for motor control
#ifdef MOTORCONTROL
  ControllerAreaNetwork CAN;
  CAN.setLightBrightness(0);
#endif

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
  while(1) {
    //Read every Character
    count = read(fd, &buf, 1);
    //printf("character: %c\n", buf);

    //Check if message is completly send
    if (buf == '\n') {

      //Convert Message to string
      string s(buffer);
      string del(delimiter);
      //printf("buffer: %s\n",s.c_str());

      //Reset Optical Flow Output (vertical and horizontal flow)
      COMANV[0] = 0;
      COMANV[1] = 0;

      //Reset Number of message parts
      int c = 0;

      //Position of delimiter in string
      size_t pos = 0;

      //String for message parts
      string token;

      //Split Message in single parts (OF,firstNumber,sencondNumber)
      while ((pos = s.find(delimiter[0])) != string::npos) {
        token = s.substr(0, pos);
        //Find Beginn of message
        if (c == 0) {
          if (token == "OF") {
          } else {
                break;
          }
          //Find first part of message
        } else if (c == 1) {
          //Convert first Optical Flow Ouput to number (x-Direction)
          COMANV[0] = atof(token.c_str());
        }
        c++;
        s.erase(0, pos + 1);
      }
      token = s.substr(0, pos);
      //Convert second Optical Flow Ouput (y-Direction) to number
      COMANV[1] = atof(token.c_str());

      //Calculate Optical Flow, when vertical and horizontal flow is recieved
      if (c >= 2) {
        double CAD = -atan2(COMANV[1], COMANV[0]) / (2.0f * PI / 130.0f);
        double CN = sqrt(pow(COMANV[0], 2) + pow(COMANV[1], 2));
        double WCAN = 1.0f / (1.0f + pow(CN / n0, -g));
        double gamma = WCAN * CAD + (1.0f - WCAN) * alpha;
        gamma = CalculateMean(gamma);
        gamma *= k;
        printf("X CM %f %f, CAD: %f, gamma: %f\n", COMANV[0], COMANV[1], CAD, gamma);

        //Set Angluar Motor Speed
#ifdef MOTORCONTROL
        CAN.setTargetSpeed(v, gamma);
#endif
      }
      //Reset Buffer
      n = 0;
      memset(buffer, 0, MAXBYTES);
    } else {
      //Store every single char in buffer, until the message is complete
      buffer[n] = buf;
      n++;
    }
  }

  return 0;
}



double CalculateMean(double new_gamma) {
  gamma_buffer[gamma_index] = new_gamma;
  gamma_index++;
  if (gamma_index >= gamma_mean_size)
    gamma_index = 0;

  double mean = 0;
  for(int i=0; i<gamma_mean_size; i++) {
    mean = mean +  gamma_buffer[i]; 
  }
  mean = mean/double(gamma_mean_size);
  return mean;
}