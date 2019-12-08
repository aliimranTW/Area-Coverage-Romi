/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
   Library Includes.
   Be sure to check each of these to see what variables/functions are made
   global and accessible.
 *                                                                               *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

#include "pins.h"
#include "utils.h"
#include "motors.h"
#include "pid.h"
#include "interrupts.h"
#include "kinematics.h"
#include "line_sensors.h"
#include "irproximity.h"
#include "mapping.h"
#include "RF_Interface.h"
#include <Wire.h>
#include "imu.h"
#include "magnetometer.h"
#include "Pushbutton.h"

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
   Definitions.  Other definitions exist in the .h files above.
   Also ensure you check pins.h for pin/device definitions.
 *                                                                               *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
#define BAUD_RATE 9600



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
   Class Instances.
   This list is complete for all devices supported in this code.
 *                                                                               *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
Kinematics    Pose; //Kinematics class to store position and heading

LineSensor    LineLeft(LINE_LEFT_PIN); //Left line sensor
LineSensor    LineCentre(LINE_CENTRE_PIN); //Centre line sensor
LineSensor    LineRight(LINE_RIGHT_PIN); //Right line sensor

SharpIR       DistanceSensor1(SHARP_IR_PIN); //LEFT
SharpIR       DistanceSensor2(SHARP_IR_PIN_2);//MIDDLE
SharpIR       DistanceSensor3(SHARP_IR_PIN_3);//RIGHT

Imu           Imu;

Magnetometer  Mag; // Class for the magnetometer

Motor         LeftMotor(MOTOR_PWM_L, MOTOR_DIR_L);
Motor         RightMotor(MOTOR_PWM_R, MOTOR_DIR_R);

//These work for our Romi - We strongly suggest you perform your own tuning
PID           LeftSpeedControl( 3.5, 20.9, 0.04 );
PID           RightSpeedControl( 3.5, 20.9, 0.04 );
PID           HeadingControl( 1.5, 0, 0.001 );
PID           WheelDiff(0.7, 0, 0);

Mapper        Map; //Class for representing the map

Pushbutton    ButtonB( BUTTON_B, DEFAULT_STATE_HIGH);
Pushbutton    ButtonA( BUTTON_B, DEFAULT_STATE_HIGH);

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
   Global variables.
   These global variables are not mandatory, but are used for the example loop()
   routine below.
 *                                                                               *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

//Use these variables to set the demand of the speed controller
bool use_speed_controller = true;
float left_speed_demand = 0;
float right_speed_demand = 0;
unsigned long elapsed_time;
unsigned long time_of_read;   // We will store a timestamp in this.
unsigned long time_now;
int counter = 0;
float lastREncoder = 0;
float lastLEncoder = 0;
int n = 0;
float x = 0;
float y = 0;
float L_encoder = 0;
float R_encoder = 0;
float distance1 = 0;
float distance2 = 0;
float distance3 = 0;
float correctCount = 0 ;
int owen = 0;
int obs = 0;
int old_obs = 0;
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
   This setup() routine initialises all class instances above and peripherals.
   It is recommended:
   - You keep this sequence of setup calls if you are to use all the devices.
   - Comment out those you will not use.
   - Insert new setup code after the below sequence.
 *                                                                               *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

void setup()
{
  // These two function set up the pin
  // change interrupts for the encoders.
  setupLeftEncoder();
  setupRightEncoder();
  startTimer();

  //Set speed control maximum outputs to match motor
  LeftSpeedControl.setMax(50);
  RightSpeedControl.setMax(50);

  // For this example, we'll calibrate only the
  // centre sensor.  You may wish to use more.
  LineCentre.calibrate();

  //Setup RFID card

  setupRFID();

  // These functions calibrate the IMU and Magnetometer
  // The magnetometer calibration routine require you to move
  // your robot around  in space.
  // The IMU calibration requires the Romi does not move.
  // See related lab sheets for more information.
  /*
    Wire.begin();
    Mag.init();
    Mag.calibrate();
    Imu.init();
    Imu.calibrate();
  */

  // Set the random seed for the random number generator
  // from A0, which should itself be quite random.
  randomSeed(analogRead(A0));


  // Initialise Serial communication
  Serial.begin( BAUD_RATE );
  delay(1000);
  Serial.println("Board Reset");

  // Romi will wait for you to press a button and then print
  // the current map.
  //
  // !!! A second button press will erase the map !!!
  ButtonB.waitForButton();

  //Map.printMap();

  // Watch for second button press, then begin autonomous mode.
  ButtonB.waitForButton();

  Serial.println("Map Erased - Mapping Started");
  Map.resetMap();

  // Your extra setup code is best placed here:
  // ...
  // ...
  // but not after the following:

  // Because code flow has been blocked, we need to reset the
  // last_time variable of the PIDs, otherwise we update the
  // PID with a large time elapsed since the class was
  // initialised, which will cause a big intergral term.
  // If you don't do this, you'll see the Romi accelerate away
  // very fast!

  LeftSpeedControl.reset();
  RightSpeedControl.reset();

  //left_speed_demand = 5;
  //right_speed_demand = 5;

}


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
   This loop() demonstrates all devices being used in a basic sequence.
   The Romi should:
   - move forwards with random turns
   - log lines, RFID and obstacles to the map.

 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
void loop() {
  // Remember to always update kinematics!!
  Pose.update();
  //doMapping();//updating internal map
  x = Pose.getX();
  y = Pose.getY();
  //Serial.print(x); Serial.print(','); Serial.println(y);
  //delay(2);
  ////////sweep
  if (y >= 1700) {
    LeftMotor.setPower(0);
    RightMotor.setPower(0);
  }
  //premovement --remove later
  if (owen == 0) {
    countRecord();
    straightLinePD(300);
    owen += 1;
  }
  //

  ////////obstacle
  if ( DistanceSensor2.getDistanceInMM() < 150) {
    float x_obstacle = x;
    float y_obstacle = y;
    old_obs = obs;
    obs += 1;
  }

  if (DistanceSensor2.getDistanceInMM() <= 150) {
    countRecord();
    rotateAngle(90);
    countRecord();
    straightLinePD(150);
    countRecord();
    rotateAngle(90);
    countRecord();
    straightLinePD(100);
  }


  if (obstacleFD() == false) {
    if (x >= 1600) {
      countRecord();
      rotateAngle(90);
      countRecord();
      straightLinePD(150);
      countRecord();
      rotateAngle(90);
      countRecord();
      straightLinePD(100);
    }
    else if (x <= 200) {
      countRecord();
      rotateAngle(-90);
      countRecord();
      straightLinePD(150);
      countRecord();
      rotateAngle(-90);
      countRecord();
      straightLinePD(100);
    }
    else {
      countRecord();
      while ((obstacleFD() == false) && (x > 200) && (x < 1600) && (y < 1700)) {
        straightLine();
        Pose.update();
        x = Pose.getX();
        y = Pose.getY();
        //Serial.print(x); Serial.print(','); Serial.println(y);
      }
    }
  }



}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
   We have implemented a random walk behaviour for you
   with a *very* basic obstacle avoidance behaviour.
   It is enough to get the Romi to drive around.  We
   expect that in your first week, should should get a
   better obstacle avoidance behaviour implemented for
   your Experiment Day 1 baseline test.
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

bool obstacleFD() {
  if (DistanceSensor2.getDistanceInMM() <= 150) {
    return true;
  }
  else {
    return false;
  }
}

bool obstacleLD() {
  if (DistanceSensor1.getDistanceInMM() <= 150) {
    return true;
  }
  else {
    return false;
  }
}

bool obstacleRD() {
  if (DistanceSensor3.getDistanceInMM() <= 150) {
    return true;
  }
  else {
    return false;
  }
}


void countRecord() {
  lastLEncoder = left_encoder_count;
  lastREncoder = right_encoder_count;
}

void straightLine() {
  //converDistance = distance / 0.15;
  //printEncoderCount();
  //printKinematics();
  correctCount = WheelDiff.update(left_encoder_count - lastLEncoder, right_encoder_count - lastREncoder);
  //outputR = rightPose.update(desired_count + last_e1, count_e1);
  LeftMotor.setPower(50);
  RightMotor.setPower(50 + correctCount);
}

void straightLinePD(float distance) {
  float converDistance = distance / 0.15;
  //printEncoderCount();
  //printKinematics();

  while (left_encoder_count - lastLEncoder <= converDistance ) {
    correctCount = WheelDiff.update(left_encoder_count - lastLEncoder, right_encoder_count - lastREncoder);
    //outputR = rightPose.update(desired_count + last_e1, count_e1);
    Pose.update();
    x = Pose.getX();
    y = Pose.getY();
    Serial.print(x); Serial.print(','); Serial.println(y);
    delay(1);
    LeftMotor.setPower(50);
    RightMotor.setPower(50 + correctCount);
  }
  LeftMotor.setPower(0);
  RightMotor.setPower(0);
}

void rotateAngle(double angle) {
  if (angle > 0) {
    float total_count = angle / 0.063; //0.24*2(two wheels turning at the same speed
    while (left_encoder_count - lastLEncoder - right_encoder_count + lastREncoder < total_count) {
      LeftMotor.setPower(50);
      RightMotor.setPower(-50);
      Pose.update();
      x = Pose.getX();
      y = Pose.getY();
      Serial.print(x); Serial.print(','); Serial.println(y);
    }
  }
  if (angle < 0) {
    float total_count = -1 * angle / 0.063; //0.24*2(two wheels turning at the same speed
    while (right_encoder_count - lastREncoder - left_encoder_count + lastLEncoder < total_count) {
      LeftMotor.setPower(-50);
      RightMotor.setPower(50);
      Pose.update();
      x = Pose.getX();
      y = Pose.getY();
      Serial.print(x); Serial.print(','); Serial.println(y);
    }
  }
}


void printEncoderCount() {
  Serial.print(left_encoder_count); Serial.print(", "); Serial.print(right_encoder_count); Serial.print(", "); Serial.println(right_encoder_count - left_encoder_count);
}

void printKinematics() {
  Serial.print("X="); Serial.print(Pose.getX()); Serial.print(", ");
  Serial.print("Y="); Serial.print(Pose.getY()); Serial.print(", "); Serial.println(Pose.getThetaDegrees());


}



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
   This function groups up our sensor checks, and then
   encodes into the map.  To get you started, we are
   simply placing a character into the map.  However,
   you might want to look using a bitwise scheme to
   encode more information.  Take a look at mapping.h
   for more information on how we are reading and
   writing to eeprom memory.
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

void doMapping() {

  // Read the IR Sensor and determine distance in
  // mm.  Make sure you calibrate your own code!
  // We threshold a reading between 40mm and 12mm.
  // The rationale being:
  // We can't trust very close readings or very far.
  // ...but feel free to investigate this.
  float distance1 = DistanceSensor1.getDistanceInMM();
  float distance2 = DistanceSensor2.getDistanceInMM();
  float distance3 = DistanceSensor3.getDistanceInMM();
  float distRaw = DistanceSensor2.getDistanceRaw();
  float offset = 40; //the left/right sensor is offsetted about 4cm forwards

  //if( distance < 40 && distance > 12 ) {
  //if ( distRaw > 700) {

  Serial.print("D1="); Serial.print(distance1);
  Serial.print(", D2="); Serial.print(distance2);
  Serial.print(", D3="); Serial.println(distance3);

  if ( (distance1 > 60 && distance1 < 220) || (distance2 > 60 && distance2 < 220 ) || (distance3 > 60 && distance3 < 220) || checkForRFID() || LineCentre.readRaw() > 580 ) { //this distance seems reasonable based on the middle sensor

    if ( distance2 > 60 && distance2 < 220) { //IF MIDDLE SENSOR HAS READING BETWEEN 6MM AND 22MM AWAY THEN..
      Serial.println("IN MIDDLE SENSOR");
      distance2 += 80;//Add distance that the sensor is away from Romi

      //Here we calculate the actual position of the obstacle we have detected
      float projected_x = Pose.getX() + ( distance2 * cos( Pose.getThetaRadians() ) );
      float projected_y = Pose.getY() + ( distance2 * sin( Pose.getThetaRadians() ) );
      Map.updateMapFeature( (byte)'O', projected_y, projected_x );
      //if elapsed time >500

      //Serial.print("X=");Serial.print(projected_x);Serial.print("Y=");Serial.println(projected_y);
    }

    if ( distance1 > 60 && distance1 < 220) { //left
      Serial.println("IN LEFT SENSOR");


      distance1 += 80;//what should I change this value to??

      // Here we calculate the actual position of the obstacle we have detected
      float projected_x = Pose.getX() + offset + ( distance1 * cos( 1.57 ) );
      float projected_y = Pose.getY() + offset + ( distance1 * sin( 1.57 ) );
      Map.updateMapFeature( (byte)'O', projected_y, projected_x );

    }

    if ( distance3 > 60 && distance3 < 140) { //right sensor is limited to about 150
      Serial.println("IN RIGHT SENSOR");
      distance3 += 80;//what should I change this value to??

      // Here we calculate the actual position of the obstacle we have detected
      float projected_x = Pose.getX() + offset + ( distance3 * cos( -1.57 ) );
      float projected_y = Pose.getY() + offset + ( distance3 * sin( -1.57 ) );
      Map.updateMapFeature( (byte)'O', projected_y, projected_x );
    }

    if ( checkForRFID() ) {
      // Add card to map encoding.
      Map.updateMapFeature( (byte)'R', Pose.getY(), Pose.getX() );

      // you can check the position reference and
      // bearing information of the RFID Card in
      // the following way:
      /*
        Serial.println("RFID=");
        Serial.print(serialToBearing( rfid.serNum[0] ));Serial.print(" ");
        Serial.print(serialToXPos( rfid.serNum[0] ));Serial.print(" ");
        Serial.println(serialToYPos( rfid.serNum[0] ));
      */
      //
      // Note, that, you will need to set the x,y
      // and bearing information in rfid.h for your
      // experiment setup.  For the experiment days,
      // we will tell you the serial number and x y
      // bearing information for the cards in use.
    }
    if ( LineCentre.readRaw() > 580 ) {//for some reason RFID is being replaced with L instead of R
      Map.updateMapFeature( (byte)'L', Pose.getY(), Pose.getX() );
      Serial.print("LINE"); Serial.println(LineCentre.readRaw());
    }
  }

  float projected_x1 = Pose.getX();
  float projected_y1 = Pose.getY();

  char current_value = Map.returnPosition(projected_y1, projected_x1);
  //if current value is not equal to R || O || L
  //THEN update position with M

  if (current_value != (char)'O' && current_value != (char)'L' && current_value != (char)'R') {
    Map.updateMapFeature( (byte)'M', projected_y1, projected_x1 );
  }



  Map.printMap();
  // Check RFID scanner.
  // Look inside RF_interface.h for more info.

  // Basic uncalibrated check for a line.
  // Students can do better than this after CW1 ;)

}
