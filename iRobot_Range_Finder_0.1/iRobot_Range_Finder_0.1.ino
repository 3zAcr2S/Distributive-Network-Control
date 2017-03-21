#include <Wire.h>
#include <PixyI2C.h>

#define X_CENTER        ((PIXY_MAX_X-PIXY_MIN_X)/2)      // Xmax = 319, Xmin = 0
#define Y_CENTER        ((PIXY_MAX_Y-PIXY_MIN_Y)/2)      // Ymax = 199, Ymin = 0

float rad_FoV_X = 0.004090615434362; // = 75 [deg] (pi/180 [deg]) / 320 (FoV_X / Xmax)
float rad_FoV_Y = 0.003926990816987; // = 45 [deg] (pi/180 [deg]) / 200 (FoV_Y / Ymax)

PixyI2C pixy1(0x46); // You can set the I2C address through PixyI2C object
PixyI2C pixy2(0x45);

void dataminingMust();
void camVecUpdate();
void anglePrint(double, double, int);

#include <MatrixMath.h>
MatrixMath m;
#include <math.h>

/* camera related parameters */
float u0[3] = {1, 0, 0}, u1[3], u2[3]; // orientation: u0 - vehicle, u1 - cam1, u2 - cam2
float  v1[3], v2[3];

float U[2][2], L[2], ltemp[2], tsMat[2], p[3], q[3]; // needed for camera calculation
float l1[3] = {0, 2.5, 0}, l2[3] = {0, -2.5, 0}; // positions of cameras

/* estimation related parameters */
float u[3], z[3], Z1[3], Z2[3];

int i_c1s1 = -1;
int i_c1s2 = -1;
int i_c2s1 = -1;
int i_c2s2 = -1;
int s_c1_i = -1;
int s_c2_i = -1;
int t_c1 = 0;
int t_c2 = 0;

void setup()
{
  Serial.begin(9600);
  Serial.print("Starting...\n");

  pixy1.init();
  pixy2.init();
}


int ij = 0;

void loop()
{

  Serial.print("iteration: ");
  Serial.print(ij);
  delay(20);


  t_c1 = 0;
  t_c2 = 0;
  dataminingMust();

  // kalman();
  ij++;

}




void dataminingMust() {
  Serial.println(" stage 0");
  static int i1 = 0;
  static int i2 = 0;
  int j;
  uint16_t blocks1 = 0;
  uint16_t blocks2 = 0;
  int ii = 0;
  char buf[32];
  bool target = 1;
  bool password_1 = 0;
  bool password_2 = 0;
  int i_c1s1 = -1;
  int i_c1s2 = -1;
  int s_c1_i = -1;
  int s_c2_i = -1;
  int i_c2s1 = -1;
  int i_c2s2 = -1;


  while (target) {

    //    Serial.print("iteration: ");
    //    Serial.print(ij);
    //    Serial.println(" stage 1");

    i_c1s1 = -1;
    i_c1s2 = -1;
    s_c1_i = -1;
    s_c2_i = -1;
    i_c2s1 = -1;
    i_c2s2 = -1;


    blocks1 = pixy1.getBlocks();
    blocks2 = pixy2.getBlocks();

    if (blocks1)
    {
      i1++;
      if ( (i1 % 1) == 0)
      {
        //        Serial.println("========================================================");
        //        sprintf(buf, "Cam [1] Detected %d , @ i1 = %d:\n", blocks1, i1);
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
          t_c1 = millis();
          camVecUpdate(1);
          password_1 = 1;
        }

        //        sprintf(buf, "index of signature 2 is : %d \n", i_c1s1);
        //        Serial.print(buf);
        //        sprintf(buf, "index of signature 3 is : %d \n", i_c1s2);
        //        Serial.print(buf);
        //        sprintf(buf, "c1s1 : %d, c1s2 : %d, c2s1 : %d, c2s2 : %d\n", i_c1s1 != -1, i_c1s2 != -1, i_c2s1 != -1, i_c2s2 != -1);
        //        Serial.print(buf);

      }
    }

    if (blocks2)
    {
      i2++;
      if ( (i2 % 1) == 0)
      {
        //        Serial.println("------------------------------------------------------------");
        //        sprintf(buf, "Cam [2] Detected %d , @ i2 = %d:\n", blocks1, i2);
        //        Serial.print(buf);
        for (j = 0; j < blocks2; j++)
        {
          //          sprintf(buf, "  block %d: ", j);
          //          Serial.print(buf);
          //          pixy2.blocks[j].print();
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
          t_c2 = millis();
          camVecUpdate(2);
          password_2 = 1;
        }

        //        sprintf(buf, "index of signature 2 is : %d \n", i_c2s1);
        //        Serial.print(buf);
        //        sprintf(buf, "index of signature 3 is : %d \n", i_c2s2);
        //        Serial.print(buf);


      }
    }
    if (blocks1 || blocks2)
    {
      if ( (abs(t_c2 - t_c1) < 300) && t_c1 && t_c2  )
      {
        Serial.println("YES");
        target = 0;
      }
    }
  }



  Serial.print("iteration ");
  Serial.print(ij);
  Serial.println(" stage 2");
  //  Serial.print(buf);
  //  sprintf(buf, "u1[1] : %f, u1[2] : %f, u1[3] : %f;   u2[1] : %f, u2[2] : %f, u2[3] : %f\n", u1[1], u1[2], u1[3], u2[1], u2[2], u2[3]);
  //  Serial.print(buf);
  //  sprintf(buf, "v1[1] : %f, v1[2] : %f, v1[3] : %f;   v2[1] : %f, v2[2] : %f, v2[3] : %f\n", v1[1], v1[2], v1[3], v2[1], v2[2], v2[3]);
  //  Serial.print(buf);

  target = 1;

  triangulation(u1, u2);

  //  for (ii = 0; ii <= 3; ii++)
  //  {
  //    Z1[ii] = z[ii] ;
  //  }
  triangulation(v1, v2);
  //  for (ii = 0; ii <= 3; ii++)
  //  {
  //    Z2[ii] = z[ii] ;
  //  }
  delay(1000); // just for debugging delete this line for real case


}

void datamining() {
  /*  t1 = millis();
    uint16_t blocks1;
    char buf1[32];
    uint16_t blocks2;
    char buf2[32];
    int ii;

    Serial.println(blocks1);
    delay(20); // pixy fps: 50 so delay 20ms will suffice it to process
    for (ii = 0; ii <= blocks1; ii++) { // check there is two signature captured
      if (pixy1.blocks[ii].signature == 1) i_c1s1 = ii;
      if (pixy1.blocks[ii].signature == 2) i_c1s2 = ii;
    }

    blocks2 = pixy2.getBlocks();
    delay(20); // pixy fps: 50 so delay 20ms will suffice it to process
    for (ii = 0; ii <= blocks2; ii++) { // check there is two signature captured
      if (pixy2.blocks[ii].signature == 1) i_c2s1 = ii;
      if (pixy2.blocks[ii].signature == 2) i_c2s2 = ii;
    }

    if (i_c1s1 * i_c1s2 * i_c2s1 * i_c2s2) {

      camVecUpdate();
      triangulation(u1, u2);

      delay(1000); // just for debugging delete this line for real case
    }
    t2 = millis();
    dt = (t2 - t1) / 1000; // without necessary delay, it's about 0.14 seconds
  */
}

void triangulation(float*u1, float*u2) {
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
  Serial.print(t_c1);
  Serial.print("  ");
  Serial.print(t_c2);
  Serial.print("\n");
}

/*-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
  -=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-*/


void camVecUpdate(int k) {
  double yawCam, pitchCam;
  char buf[32];
  if (k == 1)
  {
    yawCam    = (X_CENTER - pixy1.blocks[i_c1s1].x) * rad_FoV_X; // Xangle [rad] of the center of mass of the block from the center
    pitchCam  = (Y_CENTER - pixy1.blocks[i_c1s1].y) * rad_FoV_Y; // Yangle [rad] of the center of mass of the block from the center

    u1[0] = cos(pitchCam) * cos(yawCam);
    u1[1] = cos(pitchCam) * sin(yawCam);
    u1[2] = sin(pitchCam);

    yawCam    = (X_CENTER - pixy1.blocks[i_c1s2].x) * rad_FoV_X; // Xangle [rad] of the center of mass of the block from the center
    pitchCam  = (Y_CENTER - pixy1.blocks[i_c1s2].y) * rad_FoV_Y; // Yangle [rad] of the center of mass of the block from the center

    v1[0] = cos(pitchCam) * cos(yawCam);
    v1[1] = cos(pitchCam) * sin(yawCam);
    v1[2] = sin(pitchCam);
    Serial.print("@ time = ");
    Serial.print(t_c1);
    Serial.println(u1[0]);
//    Serial.print(buf);
//    sprintf(buf, "u1[1] : %f, u1[2] : %f, u1[3] : %f;  u2[1] : %f, v1[2] : %f, v1[3] : %f\n", u1[0], u1[1], u1[2], v1[0], v1[1], v1[2]);
//    Serial.print(buf);


  }





  //  anglePrint(yawCam,pitchCam,1); // just for debugging delete this line for real case
  if (k == 2)
  {
    yawCam    = (X_CENTER - pixy2.blocks[i_c2s1].x) * rad_FoV_X; // Xangle [rad] of the center of mass of the block from the center
    pitchCam  = (Y_CENTER - pixy2.blocks[i_c2s1].y) * rad_FoV_Y; // Yangle [rad] of the center of mass of the block from the center

    u2[0] = cos(pitchCam) * cos(yawCam);
    u2[1] = cos(pitchCam) * sin(yawCam);
    u2[2] = sin(pitchCam);

    yawCam    = (X_CENTER - pixy2.blocks[i_c2s2].x) * rad_FoV_X; // Xangle [rad] of the center of mass of the block from the center
    pitchCam  = (Y_CENTER - pixy2.blocks[i_c2s2].y) * rad_FoV_Y; // Yangle [rad] of the center of mass of the block from the center

    v2[0] = cos(pitchCam) * cos(yawCam);
    v2[1] = cos(pitchCam) * sin(yawCam);
    v2[2] = sin(pitchCam);
    Serial.print("@ time = ");
    Serial.println(t_c2);
//    Serial.print(buf);
//    sprintf(buf, "u2[1] : %f, u2[2] : %f, u2[3] : %f;  v2[1] : %f, v2[2] : %f, v3[3] : %f\n", u2[0], u2[1], u2[2], v2[0], v2[1], v2[2]);
//    Serial.print(buf);

  }
}

void anglePrint(double yaw, double pitch, int ind) {
  String cam = "Cam";


  Serial.print(cam + ind); // which camera
  Serial.print(": x rad = ");
  Serial.print(yaw, 3);
  Serial.print(", ");
  Serial.print("y rad = ");
  Serial.print(pitch, 3);
  Serial.print("\n");
  delay(50); // just for debugging delete this line for real case
}
