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

#include <Wire.h>
#include <PixyI2C.h>
#include <MatrixMath.h>
#include <math.h>

#define X_CENTER        ((PIXY_MAX_X-PIXY_MIN_X)/2)      // Xmax = 319, Xmin = 0
#define Y_CENTER        ((PIXY_MAX_Y-PIXY_MIN_Y)/2)      // Ymax = 199, Ymin = 0

float rad_FoV_X = 0.004090615434362; // = 75 [deg] (pi/180 [deg]) / 320 (FoV_X / Xmax)
float rad_FoV_Y = 0.003926990816987; // = 45 [deg] (pi/180 [deg]) / 200 (FoV_Y / Ymax)



void camVecUpdate();
void triangulation();
void getvector();
MatrixMath m;
/* camera related parameters */
float u0[3] = {1, 0, 0}, u1[3], u2[3]; // orientation: u0 - vehicle, u1 - cam1, u2 - cam2
float  v1[3], v2[3];
float x_c1s1[2], x_c1s2[2], x_c2s1[2], x_c2s2[2];

float U[2][2], L[2], ltemp[2], tsMat[2], p[3], q[3]; // needed for camera calculation
float l1[3] = {0, 6, 0}, l2[3] = {0, -6, 0}; // positions of cameras

/* estimation related parameters */
float t2, t1, dt;
float x[6], xprev[6], xest[3], u[3], z[3], P[6][6], Pprev[6][6], Pest[6][6], Ex[6][6], Ez[3][3], K[6][3], Z1[3], Z2[3];



//PixyI2C pixy;
PixyI2C pixy1(0x45); // You can set the I2C address through PixyI2C object
PixyI2C pixy2(0x46);

// ------- index of different objects
int i_c1s1 = -1;
int i_c1s2 = -1;
int i_c2s1 = -1;
int i_c2s2 = -1;
int s_c1_i = -1;
int s_c2_i = -1;
int t = millis();
int t_1 = millis();
int t_2 = millis();


void setup()
{
  Serial.begin(9600);
  Serial.print("Starting...\n");

  pixy1.init();
  pixy2.init();
}


void loop()
{
  delay(100);
  Serial.println("Stage [1.0]");
  getvector(1);
  t_1 = millis();
  Serial.println("Stage [2.0]");
  getvector(2);
  t_2 = millis();

  if (abs(t_1 - t_2) < 100);
  {
        Serial.println("PASSSSSS");
        camVecUpdate();
        triangulation(u1, u2);
        triangulation(v1, v2);
        delay(200);
  }



  /*
    static int i = 0;
    int j;
    uint16_t blocks1;
    uint16_t blocks2;
    char buf[32];

    i_c1s1 = -1;
    i_c1s2 = -1;
    s_c1_i = -1;
    s_c2_i = -1;
    i_c2s1 = -1;
    i_c2s2 = -1;

    blocks1 = pixy1.getBlocks();
    blocks2 = pixy2.getBlocks();



    if (blocks1 || blocks2)
    {
      i++;
      if (blocks1)
      {

        if ( i % 50 == 0)
        {
          //        Serial.println("========================================================");
          //        sprintf(buf, "Cam [1] Detected %d , @ i = %d:\n", blocks1, i);
          //        Serial.print(buf);
          for (j = 0; j < blocks1; j++)
          {
            //          sprintf(buf, "  block %d: ", j);
            //          Serial.print(buf);
            //          pixy1.blocks[j].print();

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
          if ((i_c1s1 != -1) && (i_c1s2 != -1))
          {
            t_1 = millis();
            Serial.print("t_1 = ");
            Serial.print(t_1);
            Serial.print("t_2 = ");
            Serial.println(t_2);

            x_c1s1[0] = (X_CENTER - pixy1.blocks[i_c1s1].x) * rad_FoV_X;
            x_c1s1[1] = (X_CENTER - pixy1.blocks[i_c1s1].y) * rad_FoV_Y;
            x_c1s2[0] = (X_CENTER - pixy1.blocks[i_c1s2].x) * rad_FoV_X;
            x_c1s2[1] = (X_CENTER - pixy1.blocks[i_c1s2].y) * rad_FoV_Y;

          }
          //        sprintf(buf, "index of signature 2 is : %d \n", i_c1s1);
          //        Serial.print(buf);
          //        sprintf(buf, "index of signature 3 is : %d \n", i_c1s2);
          //        Serial.print(buf);
          //        sprintf(buf, "c1s1 : %d, c1s2 : %d, c2s1 : %d, c2s2 : %d\n", i_c1s1 != -1, i_c1s2 != -1, i_c2s1 != -1, i_c2s2 != -1);
          //        Serial.print(buf);
          //        Serial.print("blocks2 is: ");
          //        Serial.println(blocks2);
        }
      }

      if (blocks2)
      {
        if ( i % 50 == 1)
        {
          //        t_2 = millis();
          //        Serial.println("-------------------------------------------------------");
          //        sprintf(buf, "Cam [2] Detected %d , @ i = %d:\n", blocks2, i);
          //        Serial.print(buf);
          for (j = 0; j < blocks2; j++)
          {
            //          sprintf(buf, "  block %d: ", j);
            //          Serial.print(buf);
            //          pixy2.blocks[j].print();

            s_c2_i = pixy2.blocks[j].signature;

            if (pixy2.blocks[j].signature == 2)
            {
              i_c2s1 = j;
            }
            if (pixy2.blocks[j].signature == 3)
            {
              i_c2s2 = j;
            }
          }
          if ((i_c2s1 != -1) && (i_c2s2 != -1))
          {
            t_2 = millis();

            Serial.print("t_1 = ");
            Serial.print(t_1);
            Serial.print("t_2 = ");
            Serial.println(t_2);

            x_c2s1[0] = (X_CENTER - pixy1.blocks[i_c2s1].x) * rad_FoV_X;
            x_c2s1[1] = (X_CENTER - pixy1.blocks[i_c2s1].y) * rad_FoV_Y;
            x_c2s2[0] = (X_CENTER - pixy1.blocks[i_c2s2].x) * rad_FoV_X;
            x_c2s2[1] = (X_CENTER - pixy1.blocks[i_c2s2].y) * rad_FoV_Y;

          }
        }
      }

      if ( (abs(t_1 - t_2) < 100 )  && (t_1) && (t_2))
      {
        t_1 = 0;
        t_2 = 0;
        t = millis();
        Serial.println("PASSSSSS");
        camVecUpdate();
        triangulation(u1, u2);
        triangulation(v1, v2);
        //delay(1000);
      }


    }
  */
}

void getvector(int k)
{
  int i = 0;
  int j;
  uint16_t blocks1;
  uint16_t blocks2;
  char buf[32];

  i_c1s1 = -1;
  i_c1s2 = -1;
  i_c2s1 = -1;
  i_c2s2 = -1;

  if (k == 1)
  {
    Serial.println("Stage [1.1]");
    while ((i_c1s1 == -1) || (i_c1s2 == -1))
    {

      blocks1 = pixy1.getBlocks();

      i++;
      if (blocks1)
      {
        if ( i % 10 == 0)
        {

          Serial.println("========================================================");
          sprintf(buf, "Cam [1] Detected %d , @ i = %d:\n", blocks1, i);
          Serial.print(buf);
          for (j = 0; j < blocks1; j++)
          {
            sprintf(buf, "  block %d: ", j);
            Serial.print(buf);
            pixy1.blocks[j].print();

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
          if ((i_c1s1 != -1) && (i_c1s2 != -1))
          {
//            t_1 = millis();
//            Serial.print("t_1 = ");
//            Serial.print(t_1);
//            Serial.print("t_2 = ");
//            Serial.println(t_2);
            x_c1s1[0] = (X_CENTER - pixy1.blocks[i_c1s1].x) * rad_FoV_X;
            x_c1s1[1] = (X_CENTER - pixy1.blocks[i_c1s1].y) * rad_FoV_Y;
            x_c1s2[0] = (X_CENTER - pixy1.blocks[i_c1s2].x) * rad_FoV_X;
            x_c1s2[1] = (X_CENTER - pixy1.blocks[i_c1s2].y) * rad_FoV_Y;
          }
        }
      }
    }
  }
  if (k == 2)
  {
    Serial.println("Stage [2.1]");
    while ((i_c2s1 == -1) || (i_c2s2 == -1))
    {

      blocks2 = pixy2.getBlocks();

      i++;
      if (blocks2)
      {
        if ( i % 10 == 0)
        {

          Serial.println("-----------------------------------------------------");
          sprintf(buf, "Cam [2] Detected %d , @ i = %d:\n", blocks2, i);
          Serial.print(buf);
          for (j = 0; j < blocks2; j++)
          {
            sprintf(buf, "  block %d: ", j);
            Serial.print(buf);
            pixy2.blocks[j].print();

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
            x_c2s1[1] = (X_CENTER - pixy2.blocks[i_c2s1].y) * rad_FoV_Y;
            x_c2s2[0] = (X_CENTER - pixy2.blocks[i_c2s2].x) * rad_FoV_X;
            x_c2s2[1] = (X_CENTER - pixy2.blocks[i_c2s2].y) * rad_FoV_Y;
          }
        }
      }
    }
  }


}



void camVecUpdate() {
  double yawCam, pitchCam;


  yawCam    = (X_CENTER - pixy1.blocks[i_c1s1].x) * rad_FoV_X; // Xangle [rad] of the center of mass of the block from the center
  pitchCam  = (Y_CENTER - pixy1.blocks[i_c1s1].y) * rad_FoV_Y; // Yangle [rad] of the center of mass of the block from the center

  u1[0] = cos(x_c1s1[1]) * cos(x_c1s1[0]);
  u1[1] = cos(x_c1s1[1]) * sin(x_c1s1[0]);
  u1[2] = sin(x_c1s1[1]);

  yawCam    = (X_CENTER - pixy1.blocks[i_c1s2].x) * rad_FoV_X; // Xangle [rad] of the center of mass of the block from the center
  pitchCam  = (Y_CENTER - pixy1.blocks[i_c1s2].y) * rad_FoV_Y; // Yangle [rad] of the center of mass of the block from the center

  v1[0] = cos(x_c1s2[1]) * cos(x_c1s2[0]);
  v1[1] = cos(x_c1s2[1]) * sin(x_c1s2[0]);
  v1[2] = sin(x_c1s2[1]);
  Serial.print("@ time = ");
  //    Serial.print(t_c1);
  Serial.print(millis());
  Serial.println(" found camera 1");




  //  anglePrint(yawCam,pitchCam,1); // just for debugging delete this line for real case

  yawCam    = (X_CENTER - pixy2.blocks[i_c2s1].x) * rad_FoV_X; // Xangle [rad] of the center of mass of the block from the center
  pitchCam  = (Y_CENTER - pixy2.blocks[i_c2s1].y) * rad_FoV_Y; // Yangle [rad] of the center of mass of the block from the center

  u2[0] = cos(x_c2s1[1]) * cos(x_c2s1[0]);
  u2[1] = cos(x_c2s1[1]) * sin(x_c2s1[0]);
  u2[2] = sin(x_c2s1[1]);

  yawCam    = (X_CENTER - pixy2.blocks[i_c2s2].x) * rad_FoV_X; // Xangle [rad] of the center of mass of the block from the center
  pitchCam  = (Y_CENTER - pixy2.blocks[i_c2s2].y) * rad_FoV_Y; // Yangle [rad] of the center of mass of the block from the center

  v2[0] = cos(x_c2s2[1]) * cos(x_c2s2[0]);
  v2[1] = cos(x_c2s2[1]) * sin(x_c2s2[0]);
  v2[2] = sin(x_c2s2[1]);
  Serial.print("@ time = ");
  //  Serial.print(t_c2);
  Serial.print(millis());
  Serial.println(" found camera 2");

}



void triangulation(float * u1, float * u2) {
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
  Serial.print("\n");
}

