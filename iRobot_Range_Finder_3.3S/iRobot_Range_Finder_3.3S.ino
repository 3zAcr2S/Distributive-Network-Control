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
// This sketch is like hello_world but uses I2C communications.  If you're
// not sure what I2C is, run the hello_world sketch!
//

/* include libraries necessary */
#include <Wire.h>
#include <PixyI2C.h>
#include <MatrixMath.h>
#include <math.h>

/* define wirings  */
// define pixy through I2C
PixyI2C pixy1(0x41);
PixyI2C pixy2(0x42);


/* define subfunctions*/
void camVecUpdate();
void triangulation();
void getvector();
void driveStraight(int velocity);
void drive(int velocity, int radius);
void rotate(int velocity);

// define communication wires with iRobot
#define rxPin 10
#define txPin 11
#define ddPin 12
/* define key constant and variables */
#define X_CENTER        ((PIXY_MAX_X-PIXY_MIN_X)/2)      // Xmax = 319, Xmin = 0
#define Y_CENTER        ((PIXY_MAX_Y-PIXY_MIN_Y)/2)      // Ymax = 199, Ymin = 0
#define k_d             0.2
#define rotation_speed  3.602413372155987     // = 2 pi / 1.7144 [sec]
#define d_ideal         90               // [cm]
float rad_FoV_X       = 0.004090615434362; // = 75 [deg] (pi/180 [deg]) / 320 (FoV_X / Xmax)
float rad_FoV_Y       = 0.003926990816987; // = 45 [deg] (pi/180 [deg]) / 200 (FoV_Y / Ymax)
int   time_step       = 10 * 1000;   // [ms]
int   time_cam_update = 1000;        // time interval for camera to reveal new information
double dist, t0; //dist in [cm], t in sec
double omega = 360 / 1.74416;

/* camera related parameters */
float       u0[3] = {1, 0, 0}, u1[3], u2[3], v1[3], v2[3];; // orientation: u0 - vehicle, u1 - cam1, u2 - cam2
float       x_c1s1[2], x_c1s2[2], x_c2s1[2], x_c2s2[2];
float       U[2][2], L[2], ltemp[2], tsMat[2], p[3], q[3]; // needed for camera calculation
float       l1[3] = {0, 6, 0}, l2[3] = {0, -6, 0}; // positions of cameras
MatrixMath  m;

/* estimation related parameters */
float       t2, t1, dt;
float       u[3], z[3], Z1[3], Z2[3];
//float       x[6], xprev[6], xest[3], u[3], z[3], P[6][6], Pprev[6][6], Pest[6][6], Ex[6][6], Ez[3][3], K[6][3], Z1[3], Z2[3]; // not useful at this moment

/* iRobot formation related parameters */
float       angle; // angle between objective 1 and 2




void setup()
{


  /* initialize iRobot */
  Serial.begin(19200);
  Serial.print("Starting...\n");
  //Serial1.begin(19200); //XBee
  Serial2.begin(19200); //iRobot

  delay(2000); // NEEDED!!!! To let the robot initialize
  Serial2.write(128); // This command starts the OI. You must always send the Start command before sending any other commands to the OI
  delay(50);
  Serial2.write(131); // Go into save mode, i.e. whenever robot is picked up it stops (see p. 6 of Create's Open Interface documentation)
  delay(50);

  /* initialize pixys */
  pixy1.init();
  pixy2.init();

}

/* initialize all timers */
int t         = millis();
int t_1       = millis();
int t_2       = millis();
int t_rotate  = millis();
int t_move    = millis();

void loop()
{

  int ii = 0;
  // THIS DELAY IS NECCESSCARY ! To help pixy cool down
  /*-=-=-=-=-=-=-=-=-=-=-=-=-*/
  delay(100);
  /*-=-=-=-=-=-=-=-=-=-=-=-=-*/
  Serial.print("Stage 1 \n");

  // get objects from camera 1
  getvector(1);
  Serial.print("Stage 2 \n");
  // time when we finish getting objects
  t_1 = millis();

  // get objects from camera 2
  getvector(2);

  // time when we finish getting objects
  t_2 = millis();


  // the time from last location update till now
  /* around 109 ~ 113 ms at default setting */
  //Serial.println(millis() - t);

  // dalay the printout if necessary, time between two update is at least 1 second.
  if (time_cam_update > (millis() - t))
  {
    delay(time_cam_update - (millis() - t));
  }

  //  //Serial.println(millis() - t);
  if ( (abs(t_1 - t_2) < 200))
  {
    //  //Serial.println(millis() - t);

    t = millis();
    // convert angle based observation into vectors
    Serial.println("Cam Update");
    camVecUpdate();
    Serial.println("Get Distance");
    getdistance();
    Serial.println("Formating");
    formating();

    t = millis();
    /*
        // calculate distance between cameras sets to object 1
        triangulation(u1, u2);

        // save temporary location vector into a permanent one
        for (ii = 0; ii <= 2; ii++)
        {
          Z1[ii] = z[ii];
      //      //Serial.print(ii);
        }
        // calculate distance between cameras sets to object 2
        triangulation(v1, v2);

        // save temporary location vector into a permanent one
        for (ii = 0; ii <= 2; ii++)
        {
          Z2[ii] = z[ii];
      //      //Serial.print(ii);
        }
    */

  }

}

void formating() {
  long int         V1, V2, velocity, radius;
  int         t_r; // local variable to find how long need for rotation
  double      k1, k2, d1, d2;
  //Serial.println("line 180");

  d1        = sqrt(sq(Z1[0]) + sq(Z1[1]));
  d2        = sqrt(sq(Z2[0]) + sq(Z2[1]));

  k1 = (sq(d1) - sq(d_ideal));
  k2 = (sq(d2) - sq(d_ideal));

  V1 = round(Z1[0]  * k1 + Z2[0] * k2); // velocity component x
  V2 = round(Z1[1]  * k1 + Z2[1] * k2);

  velocity = sqrt(sq(V1 / 100) + sq(V2 / 100)) / 100;

  Serial.println(velocity);
  Serial.print("  ");
  Serial.print(V1);
  Serial.print("  ");
  Serial.println(V2);
  angle = atan2(V2, V1);

  if (angle > PI / 2) {
    angle = angle - PI;
    velocity = - velocity;
  }
  else if (angle < -PI / 2) {
    angle = angle + PI;
    velocity = - velocity;
  }


  //dist = -0.0055 * pow(velocity, 2) + 4.9184 * pow(velocity, 1) + -3.4936;
  dist = velocity;

  Serial.println(angle);


  t0 = angle / rotation_speed;
  int t_form = millis();


  t_r = round(( -0.279333102646555 * pow(t0, 3) + 0.738510813696647 * pow(t0, 2) + 0.554393482942655 * pow(t0, 1) - 0.006375031809069 * pow(t0, 0)) * 1000);

  Serial.print("Velocity: ");
  Serial.print(velocity);
  Serial.print("  ");
  Serial.print("angle & t0: ");
  Serial.print(angle);
  Serial.print("  ");
  Serial.print(t0);
  Serial.print("  ");
  Serial.print("Distance [1] & [2]: ");
  Serial.print(d1);
  Serial.print("  ");
  Serial.print(d2);
  Serial.println("  ");

  Serial.print("rotation time: ");
  Serial.print(t_r);
  Serial.print("  distance travels ");
  Serial.println(dist);

  if (t_r > 0) {
    while ( (millis() - t_form) < 1 * t_r)
    {
      drive(100, 1);
      Serial.print("rotate left ");
    }
  }
  else {
    while ( (millis() - t_form) < 1 * (-t_r) )
    {
      drive(100, -1);
      Serial.print("rotate right ");
    }
  }
  Serial.print("\n");

  //Serial.println("line 229");

  delay(100);
  drive(0, 0);
  if ((millis() - t_form) < 3000) {
    delay(3000 - (millis() - t_form) );
  }
  delay(1000); // the delay allows the Analog to Digital converter IC to recover for the next reading.

  t_form = millis();

  while ( (millis() - t_form) < 2000)
  {
    driveStraight(dist);
  }
  delay(100); // the delay allows the Analog to Digital converter IC to recover for the next reading.
  //Serial.println("line 226");
  drive(0, 0);
  if (time_step > (millis() - t)) // delay up to  10 second
  {
    delay(time_step - (millis() - t));
  }
}

void getdistance() {
  // this subfunction is used for call triangulation with different input U & V
  // calculate distance between cameras sets to object 1
  triangulation(u1, u2);
  // save temporary location vector into a permanent one
  for (int ii = 0; ii <= 2; ii++)
  {
    Z1[ii] = z[ii];
  }

}


void getvector(int k) {

  /* normal counter i,j */
  int i = 0;
  int j;
  /* objects blocks from two camera */
  uint16_t blocks1;
  uint16_t blocks2;
  char buf[32];

  /* index of different objects */
  int i_c1s1 = -1;
  int i_c1s2 = -1;
  int i_c2s1 = -1;
  int i_c2s2 = -1;
  /* signature of different index */
  int s_c1_i = -1;
  int s_c2_i = -1;

  /* Part I  */
  // in case we want to get objects from camera 1
  if (k == 1)
  {
    Serial.println("Stage [1.1]");  // stage for debug use

    /* while not both object signature detected */
    while ((i_c1s1 == -1) || (i_c1s2 == -1))
    {
      // get blocks number from camera
      blocks1 = pixy1.getBlocks();

      // counter update, counter i is used to skip some frame for pixy, it can help pixy camera to "cool down"
      i++;

      // in the case camera 1 detects some objective
      if (blocks1)
      {
        // change the module number skips frames
        if ( i % 1 == 0)
        {
//          printing out every detected object information
          Serial.println("========================================================");
          sprintf(buf, "Cam [1] Detected %d , @ i = %d:\n", blocks1, i);
          Serial.print(buf);
          for (j = 0; j < blocks1; j++)
          {
            sprintf(buf, "  block %d: ", j);
            Serial.print(buf);
            pixy1.blocks[j].print();

            // check if signature 2,3 detected, if so remember the index j;
            s_c1_i = pixy1.blocks[j].signature;

            if ((s_c1_i == 2))
            {
              i_c1s1 = j;
            }

            if (s_c1_i == 3)
            {
              i_c1s2 = j;
            }
          }

          // if both signatures detected, save the coordinates of correspoinding objects
          if ((i_c1s1 != -1) && (i_c1s2 != -1))
          {
            x_c1s1[0] = (X_CENTER - pixy1.blocks[i_c1s1].x) * rad_FoV_X;
            x_c1s1[1] = (Y_CENTER - pixy1.blocks[i_c1s1].y) * rad_FoV_Y;
            x_c1s2[0] = (X_CENTER - pixy1.blocks[i_c1s2].x) * rad_FoV_X;
            x_c1s2[1] = (Y_CENTER - pixy1.blocks[i_c1s2].y) * rad_FoV_Y;
          }
        }
      }
    }
    //        //Serial.println("Stage [1.2]"); // stage for debug use
  }

  /* Part II */
  // in case we want toget objects from camera 2. Every line of code is same as first part.
  if (k == 2)
  {

    Serial.println("Stage [2.1]"); // stage for debug use
    while ((i_c2s1 == -1) || (i_c2s2 == -1))
    {
      blocks2 = pixy2.getBlocks();

      i++;
      if (blocks2)
      {
        if ( i % 1 == 0)
        {

          //          Serial.println("-----------------------------------------------------");
          //          sprintf(buf, "Cam [2] Detected %d , @ i = %d:\n", blocks2, i);
          //          Serial.print(buf);
          for (j = 0; j < blocks2; j++)
          {
            //            sprintf(buf, "  block %d: ", j);
            //            Serial.print(buf);
            //            pixy2.blocks[j].print();

            s_c2_i = pixy2.blocks[j].signature;

            if ((s_c2_i == 2))
            {
              i_c2s1 = j;
            }

            if (s_c2_i == 3)
            {
              i_c2s2 = j;
            }
          }
          if ((i_c2s1 != -1) && (i_c2s2 != -1))
          {
            x_c2s1[0] = (X_CENTER - pixy2.blocks[i_c2s1].x) * rad_FoV_X;
            x_c2s1[1] = (Y_CENTER - pixy2.blocks[i_c2s1].y) * rad_FoV_Y;
            x_c2s2[0] = (X_CENTER - pixy2.blocks[i_c2s2].x) * rad_FoV_X;
            x_c2s2[1] = (Y_CENTER - pixy2.blocks[i_c2s2].y) * rad_FoV_Y;
          }
        }
      }
    }
    //    //Serial.println("Stage [2.2]"); // stage for debug use
  }


}



void camVecUpdate() {

  /* part of old code, in case of future use */
  //  double yawCam, pitchCam;
  //  yawCam    = (X_CENTER - pixy1.blocks[i_c1s1].x) * rad_FoV_X; // Xangle [rad] of the center of mass of the block from the center
  //  pitchCam  = (Y_CENTER - pixy1.blocks[i_c1s1].y) * rad_FoV_Y; // Yangle [rad] of the center of mass of the block from the center
  Serial.print(x_c1s1[0]);
  Serial.print("  ");
  Serial.print(x_c1s2[0]);
  Serial.print("  ");
  Serial.print(x_c2s1[0]);
  Serial.print("  ");
  Serial.print(x_c2s2[0]);
  Serial.print("  ");

  Serial.print(x_c1s1[1]);
  Serial.print("  ");
  Serial.print(x_c1s2[1]);
  Serial.print("  ");
  Serial.print(x_c2s1[1]);
  Serial.print("  ");
  Serial.print(x_c2s2[1]);
  Serial.println("  ");




  /* information of object from camera 1 with signature 1 */

  u1[0] = cos(x_c1s1[1]) * cos(x_c1s1[0]);
  u1[1] = cos(x_c1s1[1]) * sin(x_c1s1[0]);
  u1[2] = sin(x_c1s1[1]);
  //  m.Print((float*)u1, 3, 1, "u1");

  /* information of object from camera 1 with signature 2 */
  v1[0] = cos(x_c1s2[1]) * cos(x_c1s2[0]);
  v1[1] = cos(x_c1s2[1]) * sin(x_c1s2[0]);
  v1[2] = sin(x_c1s2[1]);
  //  m.Print((float*)v1, 3, 1, "v1");
  //  anglePrint(yawCam,pitchCam,1); // just for debugging delete this line for real case

  /* information of object from camera 2 with signature 1 */
  u2[0] = cos(x_c2s1[1]) * cos(x_c2s1[0]);
  u2[1] = cos(x_c2s1[1]) * sin(x_c2s1[0]);
  u2[2] = sin(x_c2s1[1]);
  //  m.Print((float*)u2, 3, 1, "u2");
  /* information of object from camera 2 with signature 2 */
  v2[0] = cos(x_c2s2[1]) * cos(x_c2s2[0]);
  v2[1] = cos(x_c2s2[1]) * sin(x_c2s2[0]);
  v2[2] = sin(x_c2s2[1]);
  //  m.Print((float*)v2, 3, 1, "v2");
}



void triangulation(float * u1, float * u2) {
  float       U[2][2], L[2], ltemp[2], tsMat[2], p[3], q[3];
  MatrixMath m;
  /*In order to perform the triangulation, need 2 vectors
    Using 1. the orientation of each camera and
        2. the center of mass of each block,
    will get the vector for each camera*/

  /* U calculation
       U = [ -u1'u1 ,  u2'u1
             -u1'u2 ,  u2'u2 ] */

  U[0][0] = -m.Dot((float*)u1, (float*)u1, 3);
  U[0][1] = m.Dot((float*)u2, (float*)u1, 3);
  U[1][0] = -m.Dot((float*)u1, (float*)u2, 3);
  U[1][1] = m.Dot((float*)u2, (float*)u2, 3);

  /* L calculation
     L = [ (l1-l2)u1
           (l1-l2)u2 ] */

  m.Subtract((float*)l1, (float*)l2, 3, 1, (float*)ltemp);
  L[0] = m.Dot((float*)ltemp, (float*)u1, 3);
  L[1] = m.Dot((float*)ltemp, (float*)u2, 3);

  /* tsMat calculation
     tsMat = U'L */

  m.Invert((float*)U, 2);
  m.Multiply((float*)U, (float*)L, 2, 2, 1, (float*)tsMat);

  /* z calculation
     p = t x u1 + l1
     q = s x u2 + l2
     z = (p + q) / 2 */

  m.Scale((float*)u1, 3, 1, tsMat[0]);
  m.Scale((float*)u2, 3, 1, tsMat[1]);
  m.Add((float*)u1, (float*)l1, 3, 1, (float*)p);
  m.Add((float*)u2, (float*)l1, 3, 1, (float*)q);
  m.Add((float*)p, (float*)q, 3, 1, (float*)z);
  m.Scale((float*)z, 3, 1, 0.5);

  m.Print((float*)z, 3, 1, "z"); // just for debugging delete this line for real case
  //Serial.print("\n");
}

void driveStraight(int velocity) {
  byte cmd[] = {(byte) 137,
                (byte)highByte(velocity),
                (byte)lowByte(velocity),
                (byte)128,
                (byte)0
               }; ;    // these are the bytes for the special case of driving straight

  for (int i = 0; i < 5; i++) {
    Serial2.write(cmd[i]);
  }

}

void drive(int velocity, int radius) {
  byte cmd[] = {(byte) 137,
                (byte)highByte(velocity),
                (byte)lowByte(velocity),
                (byte)highByte(radius),
                (byte)lowByte(radius)
               };

  for (int i = 0; i < 5; i++) {
    Serial2.write(cmd[i]);
  }
}
