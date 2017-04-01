//
// begin license header
//
// This file is part of Pixy CMUcam5 or "Pixy" for short
//
// All Pixy source code is provided under the terms of the
// GNU General Public License v2 (http://www.gnu.org/licenses/gpl-2.0.html).
// Those wishing to use Pixy source code, software and/or
// technologies under different licensing terms should contact us at
// cmucam@cs.cmu.edu. Such licensing terms are available for
// all portions of the Pixy codebase presented here.
//
// end license header
//
// This sketch is a simple tracking demo that uses the pan/tilt unit.  For
// more information, go here:
//
// http://cmucam.org/projects/cmucam5/wiki/Run_the_Pantilt_Demo
//

#include <PixyI2C.h>
#include <Pixy.h>
#include <Wire.h>

PixyI2C pixy1(0x41); // You can set the I2C address through PixyI2C object
PixyI2C pixy2(0x42); // You can set the I2C address through PixyI2C object

volatile long trig_echo1,trig_echo2;
volatile long duration1, duration1;
const int trigPin = 2;
const int echoPin = 4;
const int trigPin = 3;
const int echoPin = 5;

#define X_CENTER        ((PIXY_MAX_X-PIXY_MIN_X)/2)
#define Y_CENTER        ((PIXY_MAX_Y-PIXY_MIN_Y)/2)

class ServoLoop
{
  public:
    ServoLoop(int32_t pgain, int32_t dgain);

    void update(int32_t error);

    int32_t m_pos;
    int32_t m_prevError;
    int32_t m_pgain;
    int32_t m_dgain;
};


ServoLoop panLoop(300, 500);
ServoLoop tiltLoop(500, 700);

ServoLoop::ServoLoop(int32_t pgain, int32_t dgain)
{
  m_pos = PIXY_RCS_CENTER_POS;
  m_pgain = pgain;
  m_dgain = dgain;
  m_prevError = 0x80000000L;
}

void ServoLoop::update(int32_t error)
{
  long int vel;
  char buf[32];
  if (m_prevError != 0x80000000)
  {
    vel = (error * m_pgain + (error - m_prevError) * m_dgain) >> 10;
    //sprintf(buf, "%ld\n", vel);
    //Serial.print(buf);
    m_pos += vel;
    if (m_pos > PIXY_RCS_MAX_POS)
      m_pos = PIXY_RCS_MAX_POS;
    else if (m_pos < PIXY_RCS_MIN_POS)
      m_pos = PIXY_RCS_MIN_POS;
  }
  m_prevError = error;
}



void setup()
{
  Serial.begin(19200);
  Serial.print("Starting...\n");

  pixy1.init();
  pixy2.init();
  pinMode(echoPin1, INPUT);
  pinMode(trigPin1, OUTPUT);
  pinMode(echoPin2, INPUT);
  pinMode(trigPin2, OUTPUT);
  attachInterrupt(echoPin1, get_duration, CHANGE);
  attachInterrupt(echoPin2, get_duration, CHANGE);

}

void loop()
{

  camMove();
  ultraScan();
}

void camMove() {

  static int i = 0;
  int j;
  uint16_t blocks;
  char buf[32];
  int32_t panError, tiltError;

  blocks = pixy.getBlocks();

  if (blocks)
  {
    panError = X_CENTER - pixy.blocks[0].x;
    tiltError = pixy.blocks[0].y - Y_CENTER;

    panLoop.update(panError);
    tiltLoop.update(tiltError);

    pixy.setServos(panLoop.m_pos, tiltLoop.m_pos);
//    Serial.print(panLoop.m_pos);
//    Serial.print("  ");
//    Serial.println(tiltLoop.m_pos);

    //    i++;
    //
    //    // do this (print) every 50 frames because printing every
    //    // frame would bog down the Arduino
    //    if (i % 50 == 0)
    //    {
    //      sprintf(buf, "Detected %d:\n", blocks);
    //      Serial.print(buf);
    //      for (j = 0; j < blocks; j++)
    //      {
    //        sprintf(buf, "  block %d: ", j);
    //        Serial.print(buf);
    //        pixy.blocks[j].print();
    //      }
    //    }
  }
}


void ultraScan() {
  {
    // establish variables for duration of the ping,
    // and the distance result in inches and centimeters:
    long inches, cm;

    // The sensor is triggered by a HIGH pulse of 10 or more microseconds.
    // Give a short LOW pulse beforehand to ensure a clean HIGH pulse:
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);

    // Read the signal from the sensor: a HIGH pulse whose
    // duration is the time (in microseconds) from the sending
    // of the ping to the reception of its echo off of an object.
    //  duration = pulseIn(echoPin, HIGH);

    // convert the time into a distance
    inches = microsecondsToInches(duration);
    cm = microsecondsToCentimeters(duration);

    Serial.print(inches);
    Serial.print("in, ");
    Serial.print(cm);
    Serial.print("cm");
    Serial.println();

    delay(100);
  }
}


void get_duration1() {
  if (digitalRead(echoPin1)) //if the pin is HIGH, note the time
  {
    trig_echo1 = micros();
        Serial.println(trig_echo1);
  }
  else
  {
    duration1 = micros() - trig_echo1; //if it is low, end the time
        Serial.println(duration1);
    //    cm = microsecondsToCentimeters(duration);
  }
}

void get_duration2() {
  if (digitalRead(echoPin2)) //if the pin is HIGH, note the time
  {
    trig_echo2 = micros();
        Serial.println(trig_echo2);
  }
  else
  {
    duration2 = micros() - trig_echo2; //if it is low, end the time
        Serial.println(duration2);
    //    cm = microsecondsToCentimeters(duration);
  }
}

long microsecondsToInches(long microseconds)
{
  // According to Parallax's datasheet for the PING))), there are
  // 73.746 microseconds per inch (i.e. sound travels at 1130 feet per
  // second).  This gives the distance travelled by the ping, outbound
  // and return, so we divide by 2 to get the distance of the obstacle.
  // See: http://www.parallax.com/dl/docs/prod/acc/28015-PING-v1.3.pdf
  return microseconds / 74 / 2;
}

long microsecondsToCentimeters(long microseconds)
{
  // The speed of sound is 340 m/s or 29 microseconds per centimeter.
  // The ping travels out and back, so to find the distance of the
  // object we take half of the distance travelled.
  return microseconds / 29 / 2;
}
