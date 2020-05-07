
#include <EEPROM.h>
#include <ax12.h>
#include <BioloidController.h>
#include <string.h>
#include "poses.h"
#include <math.h>
#include <ros.h>


ros::NodeHandle nh;

//define pan and tilt servo IDs
#define PAN    1
#define TILT   2

// the F2 'C' bracket attached to the tilt servo creates a physical limitation to how far
// we can move the tilt servo. This software limit will ensure that we don't jam the bracket into the servo.
#define TILT_MAX 768
#define TILT_MIN 256

//Upper/Lower limits for the pan servo - by defualt they are the normal 0-1023 (0-300) positions for the servo
#define PAN_MAX 1023
#define PAN_MIN 0

//Default/Home position. These positions are used for both the startup position as well as the
//position the servos will go to when they lose contact with the commander
#define DEFAULT_PAN 512
#define DEFAULT_TILT 512

#define P_X 5472.0
#define P_Y 3648.0
#define phi_X 83.0*M_PI/180.0
#define phi_Y 66.0*M_PI/180.0

int LED = 0;
int number_of_blinks = 0;
String inputString = "";
int pan = DEFAULT_PAN;    //current position of the pan servo
int tilt = DEFAULT_TILT;   //current position of the tilt servo

// Persistant variables
double X_old = 0;
double Y_old = 0;
double X_dot = 0;
double Y_dot = 0;
double X_old_time = 0;
double Y_old_time = 0;
double X_integrator = 0;
double Y_integrator = 0;

// Controller Settings
double sigma = 0.05;
double sat_lim = 10;
double X_i = 0;
double X_d = 0;
double Y_i = 0;
double Y_d = 0;

BioloidController bioloid = BioloidController(1000000); //start the BIOLOID library at 1mbps.

void setup() {

  pinMode(LED,OUTPUT);     // setup user LED
  digitalWrite(LED, LOW);

  delay(1000);  //wait for the bioloid controller to intialize
  bioloid.poseSize = 2;            //2 servos, so the pose size will be 2
  bioloid.readPose();              //find where the servos are currently
  bioloid.setNextPose(PAN,pan);    //prepare the PAN servo to the default position
  bioloid.setNextPose(TILT,tilt);  //preprare the tilt servo to the default position
  bioloid.interpolateSetup(2000);  //setup for interpolation from the current position to the positions set in setNextPose, over 2000ms
  while(bioloid.interpolating > 0) //until we have reached the positions set in setNextPose, execute the instructions in this loop
  {
    bioloid.interpolateStep();//move servos 1 'step
    delay(3);
  }

  // put your setup code here, to run once:
  Serial.begin(9600);
  inputString.reserve(200);
  Serial.println("ready");
}

// the loop routine runs over and over again forever:
void loop() {
  digitalWrite(LED, HIGH);
}

void serialEvent()
{
  digitalWrite(LED, LOW);
  String input = Serial.readStringUntil('\n');

  int comma = input.indexOf(',');

//  Serial.println(input);

  int X_pixle_location = input.substring(0,comma).toInt();
  int Y_pixle_location = input.substring(comma+1).toInt();

  int delta_X = Xcontroller(X_pixle_location);
  int delta_Y = Ycontroller(Y_pixle_location);

//  Serial.print(delta_X);
//  Serial.print(",");
//  Serial.println(delta_Y);

  pan = dxlGetPosition(PAN) + delta_X;
  tilt = dxlGetPosition(TILT) + delta_Y; // Needs to be from current position

  pan = max(PAN_MIN,pan);
  pan = min(PAN_MAX,pan);
  tilt = max(TILT_MIN,tilt);
  tilt = min(TILT_MAX,tilt);

  dxlSetGoalPosition(PAN,pan);
  dxlSetGoalPosition(TILT,tilt);
}

int Xcontroller(int X)
{
  double X_time = millis();
  double dt = X_time - X_old_time;

  X_dot = (2*sigma-dt)/(2*sigma+dt)*X_dot + 2/(2*sigma+dt)*(X_old - X);
  Serial.println(X_dot);

  X_old = X;
  X_old_time = X_time;

//  Serial.println(X);

  int u_unsat = round(atan(X/P_X*tan(phi_X/2))/(0.29*M_PI/180) + X_d*X_dot + X_i*X_integrator);

  int u = min(sat_lim,u_unsat);
  u = max(-sat_lim,u);

  return u;
  //X_integrator = X_integrator + 1/X_i*(u-u_unsat);


}

int Ycontroller(int Y)
{
  double Y_time = millis();
  double dt = Y_time - Y_old_time;

  Y_dot = (2*sigma-dt)/(2*sigma+dt)*Y_dot + 2/(2*sigma+dt)*(Y_old - Y);

  Y_old = Y;
  Y_old_time = Y_time;

  int u_unsat = round(atan(Y/P_Y*tan(phi_Y/2))/(0.29*M_PI/180) + Y_d*Y_dot + Y_i*Y_integrator);

  int u = min(sat_lim,u_unsat);
  u = max(-sat_lim,u);

  return u;

  //Y_integrator = Y_integrator + 1/Y_i*(u-u_unsat);
}

void gothere()
{
  digitalWrite(LED, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(1000);               // wait for a second
  digitalWrite(LED, LOW);    // turn the LED off by making the voltage LOW
  delay(1000);
}
