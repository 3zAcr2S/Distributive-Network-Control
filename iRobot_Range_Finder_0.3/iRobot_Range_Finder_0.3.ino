#include <Wire.h>
#include <PixyI2C.h>

#define X_CENTER        ((PIXY_MAX_X-PIXY_MIN_X)/2)      // Xmax = 319, Xmin = 0
#define Y_CENTER        ((PIXY_MAX_Y-PIXY_MIN_Y)/2)      // Ymax = 199, Ymin = 0

float rad_FoV_X = 0.004090615434362; // = 75 [deg] (pi/180 [deg]) / 320 (FoV_X / Xmax)
float rad_FoV_Y = 0.003926990816987; // = 45 [deg] (pi/180 [deg]) / 200 (FoV_Y / Ymax)

PixyI2C pixy1(0x46); // You can set the I2C address through PixyI2C object
PixyI2C pixy2(0x45);

void initialSetup();
void datamining();
void dataminingMust();
void camVecUpdate();
void anglePrint(double, double, int);
void VectorCalc(int, int, int, int, int);

#include <MatrixMath.h>
MatrixMath m;
#include <math.h>

/* camera related parameters */
float u0[3] = {1, 0, 0}, u1[3], u2[3]; // orientation: u0 - vehicle, u1 - cam1, u2 - cam2
float  v1[3], v2[3];

float U[2][2], L[2], ltemp[2], tsMat[2], p[3], q[3]; // needed for camera calculation
float l1[3] = {0, 2.5, 0}, l2[3] = {0, -2.5, 0}; // positions of cameras

/* estimation related parameters */
bool beginningKalman = 1; // for "initialSetup();" <-- for xprev_0 and P_0
float t2, t1, dt;
float A[6][6], B[6][3], C[3][6], Atrnsp[6][6], Ctrnsp[6][3], Mtemp[3][3], Mtemp1[6][6]; //Kalman Filter
float x[6], xprev[6], xest[3], u[3], z[3], P[6][6], Pprev[6][6], Pest[6][6], Ex[6][6], Ez[3][3], K[6][3], Z1[3], Z2[3];
float accelNoise, measNoise;

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
  Serial.begin(38400);
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
  //  datamining();
  //initialSetup();
  dataminingMust();
  // kalman();
  ij++;
  t_c1 = 0;
  t_c2 = 0;
}

void kalman() {
  /* A definition      , B definition        , C definition  , A'(Atrnsp)  , C'(Ctrnsp)
     A = [ I , (dt)I     B = [ (dt^2 / 2)I     C = [ I , 0 ] , Atrnsp = A'   Ctrnsp = C'
           0 ,   I   ]            (dt)I    ]                                              */

  m.Identity((float*)Mtemp, 3);
  m.BlockCopy((float*)Mtemp, 3, 0, 0, 2, 2, (float*)A, 6);
  m.BlockCopy((float*)Mtemp, 3, 3, 3, 5, 5, (float*)A, 6);
  m.Scale((float*)Mtemp, 3, 3, dt);
  m.BlockCopy((float*)Mtemp, 3, 0, 3, 2, 5, (float*)A, 6);

  m.BlockCopy((float*)Mtemp, 3, 3, 0, 5, 2, (float*)B, 3);
  m.Scale((float*)Mtemp, 3, 3, dt / 2);
  m.BlockCopy((float*)Mtemp, 3, 0, 0, 2, 2, (float*)B, 3);

  m.Identity((float*)Mtemp, 3);
  m.BlockCopy((float*)Mtemp, 3, 0, 0, 2, 2, (float*)C, 6);

  m.Transpose((float*)A, 6, 6, (float*)Atrnsp);

  m.Transpose((float*)C, 3, 6, (float*)Ctrnsp);

  /*  xest = Ax + Bu  */
  m.Multiply((float*)A, (float*)xprev, 2, 2, 1, (float*)xest);
  m.Zeros((float*)u, 6, 1);
  u[0] = -x[3]; u[1] = -x[4];
  m.Scale((float*)u, 2, 1, 3.46);
  m.Multiply((float*)B, (float*)u, 2, 2, 1, (float*)u);
  m.Add((float*)xest, (float*)u, 3, 1, (float*)xest);

  /*  pest = APA' + Ex  */
  m.Multiply((float*)A, (float*)P, 2, 2, 2, (float*)Pest);
  m.Multiply((float*)Pest, (float*)Atrnsp, 2, 2, 2, (float*)Pest);
  m.Add((float*)Pest, (float*)Ex, 2, 2, (float*)Pest);

  /*  K = PC'(CPC'+Ez)^(-1)  */
  m.Multiply((float*)Pest, (float*)Ctrnsp, 6, 6, 3, (float*)K);
  m.Multiply((float*)C, (float*)K, 3, 6, 3, (float*)Mtemp1);
  m.Add((float*)Mtemp1, (float*)Ez, 3, 3, (float*)Mtemp1);
  m.Invert((float*)Mtemp1, 3);
  m.Multiply((float*)K, (float*)Mtemp1, 6, 3, 3, (float*)K);

  /*  x = xest + K(xest - z)  */
  m.Subtract((float*)xest, (float*)z, 6, 1, (float*)xest);
  m.Multiply((float*)K, (float*)xest, 6, 3, 3, (float*)x);
  m.Add((float*)xest, (float*)x, 3, 3, (float*)x);

  /*  P = (I - KC)Pest  */
  m.Multiply((float*)K, (float*)C, 6, 3, 6, (float*)P);
  m.Identity((float*)Mtemp, 6);
  m.Subtract((float*)Mtemp, (float*)P, 6, 6, (float*)P);
  m.Multiply((float*)P, (float*)Pest, 6, 3, 3, (float*)P);

  /*  Pprev = P  */
  m.Copy((float*)P, 6, 6, (float*)Pprev);

  /*  xprev = x  */
  m.Copy((float*)x, 6, 6, (float*)xprev);

  //  m.Print((float*)x,6,6,"x");
}

void initialSetup() {
  //  MatrixMath m;
  if (beginningKalman) {
    /*x_0 setup
      two (MUST worked out) data can be collected
      (x2 - x1) / dt can get me velocity          */
    //    Serial.print("\n\nHere is algorithm time line begins\n\n"); // just for debugging delete this line for real case
    dataminingMust();
    m.BlockCopy((float*)z, 1, 0, 0, 1, 0, (float*)xprev, 1);
    //    m.Print((float*)xprev,6,1,"xprev"); // just for debugging delete this line for real case
    //    Serial.print("\n\nCalling the second datamining\n\n"); // just for debugging delete this line for real case
    dataminingMust();
    m.BlockCopy((float*)z, 1, 0, 0, 1, 0, (float*)x, 1);
    //    m.Print((float*)x,6,1,"x"); // just for debugging delete this line for real case
    m.Subtract((float*)x, (float*)xprev, 6, 1, (float*)x);
    m.Scale((float*)x, 6, 1, 1 / dt);
    m.BlockCopy((float*)x, 1, 0, 0, 2, 0, (float*)z, 1);
    m.BlockCopy((float*)z, 1, 3, 0, 5, 0, (float*)xprev, 1);
    //    m.Print((float*)xprev,6,1,"Complete xprev"); // just for debugging delete this line for real case

    /*P_0 setup
      P_0 = Ex
       Ex = [ (dt^4/4) I , (dt^3/2) I
              (dt^3/2) I ,   dt^2 I   ]*/
    //    Serial.print("dt = "); // just for debugging delete this line for real case
    //    Serial.print(dt);
    m.Identity((float*)Mtemp, 3);
    m.Scale((float*)Mtemp, 3, 3, dt * dt);
    m.BlockCopy((float*)Mtemp, 3, 3, 3, 5, 5, (float*)Ex, 6);
    m.Scale((float*)Mtemp, 3, 3, dt / 2);
    m.BlockCopy((float*)Mtemp, 3, 0, 3, 2, 5, (float*)Ex, 6);
    m.BlockCopy((float*)Mtemp, 3, 3, 0, 5, 2, (float*)Ex, 6);
    m.Scale((float*)Mtemp, 3, 3, dt / 2);
    m.BlockCopy((float*)Mtemp, 3, 0, 0, 2, 2, (float*)Ex, 6);
    //    m.Print((float*)Ex,6,6,"Ex"); // just for debugging delete this line for real case
    m.Scale((float*)Ex, 6, 6, accelNoise * accelNoise);
    m.Copy((float*)Ex, 6, 6, (float*)Pprev);
    //    m.Print((float*)Pprev,6,6,"Pprev"); // just for debugging delete this line for real case

    /*Ez setup
      Ez*/
    m.Identity((float*)Ez, 3);
    m.Scale((float*)Ez, 3, 3, measNoise * measNoise);
    beginningKalman = 0;
  }
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

  int i_c1s1 = -1;
  int i_c1s2 = -1;
  int s_c1_i = -1;
  int s_c2_i = -1;
  int i_c2s1 = -1;
  int i_c2s2 = -1;
  int t_c1 = 0;
  int t_c2 = 0;



  //    i_c1s1 = -1;
  //    i_c1s2 = -1;
  //    s_c1_i = -1;
  //    s_c2_i = -1;
  //    i_c2s1 = -1;
  //    i_c2s2 = -1;



  Serial.print("iteration: ");
  Serial.print(ij);
  Serial.println(" stage 1");

  while ( target )
  {

    blocks1 = pixy1.getBlocks();
    blocks2 = pixy2.getBlocks();

    if (blocks1 && blocks2)
    {


      for (j = 0; j < blocks1; j++) {
        sprintf(buf, "cam[1]  block %d: ", j);
        Serial.print(buf);
        pixy1.blocks[j].print();

        s_c1_i = pixy1.blocks[j].signature;
        if (s_c1_i == 2)
        {
          i_c1s1 = j;
        }

        if (s_c1_i == 3)
        {
          i_c1s2 = j;
        }


        //        switch (pixy1.blocks[j].signature) {
        //          case 2:
        //            i_c1s1 = j;
        //            break;
        //          case 3:
        //            i_c1s2 = j;
        //            break;
        //          default:
        //            i_c1s2 = -1;
        //            i_c1s1 = -1;
        //            // if nothing else matches, do the default
        //            // default is optional
        //            break;
        //        }
      }




      for (j = 0; j < blocks2; j++) {
        sprintf(buf, "cam[2]  block %d: ", j);
        Serial.print(buf);
        pixy2.blocks[j].print();
        s_c2_i = pixy2.blocks[j].signature;
        if (s_c2_i == 2)
        {
          i_c2s1 = j;
        }
        if (s_c2_i == 3)
        {
          i_c2s2 = j;
        }

        //        switch (pixy2.blocks[j].signature) {
        //          case 2:
        //            i_c2s1 = j;
        //            break;
        //          case 3:
        //            i_c2s2 = j;
        //            break;
        //          default:
        //            i_c2s2 = -1;
        //            i_c2s1 = -1;
        //            // if nothing else matches, do the default
        //            // default is optional
        //            break;
        //        }
      }

    }

    if ((i_c1s1 != -1) && (i_c1s2 != -1) && (i_c2s1 != -1) && (i_c2s2 != -1))
    {
      Serial.println("PASS");
      target = 0;
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
  camVecUpdate();

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

  t2 = millis();
  dt = (t2 - t1) / 1000; // without necessary delay, it's about 0.14 seconds
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

/*-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
  -=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-*/


void VectorCalc(int x_1, int y_1, int x_2, int y_2, int k) {
  double yaw_1, pitch_1, yaw_2, pitch_2;
  yaw_1   = (X_CENTER - x_1) * rad_FoV_X;
  pitch_1 = (Y_CENTER - y_1) * rad_FoV_Y;
  yaw_2   = (X_CENTER - x_2) * rad_FoV_X;
  pitch_2 = (Y_CENTER - y_2) * rad_FoV_Y;
  switch (k) {
    case 1:
      u1[0] = cos(pitch_1) * cos(yaw_1);
      u1[1] = cos(pitch_1) * sin(yaw_1);
      u1[2] = sin(pitch_1);

      v1[0] = cos(pitch_2) * cos(yaw_2);
      v1[1] = cos(pitch_2) * sin(yaw_2);
      v1[2] = sin(pitch_2);
      break;
    case 2:
      u2[0] = cos(pitch_1) * cos(yaw_1);
      u2[1] = cos(pitch_1) * sin(yaw_1);
      u2[2] = sin(pitch_1);

      v2[0] = cos(pitch_2) * cos(yaw_2);
      v2[1] = cos(pitch_2) * sin(yaw_2);
      v2[2] = sin(pitch_2);
      break;
    default:
      // if nothing else matches, do the default
      // default is optional
      break;
  }

}




void camVecUpdate() {
  double yawCam, pitchCam;

  //  if (k == 1)
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
    Serial.println(" found camera 1");

  }





  //  anglePrint(yawCam,pitchCam,1); // just for debugging delete this line for real case
  //  if (k == 2)
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
    Serial.print(t_c2);
    Serial.println(" found camera 2");

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
