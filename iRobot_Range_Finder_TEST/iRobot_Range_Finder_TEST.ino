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

void setup()
{
  Serial.begin(9600);
  Serial.print("Starting...\n");

  pixy1.init();
  pixy2.init();
}


void loop()
{
  static int i = 0;
  int j;
  uint16_t blocks1;
//  uint16_t blocks2;
  char buf[32];

  i_c1s1 = -1;
  i_c1s2 = -1;
  s_c1_i = -1;
//  i_c2s1 = -1;
//  i_c2s2 = -1;

  blocks1 = pixy1.getBlocks();
//  blocks2 = pixy2.getBlocks();
  if (blocks1)
  {
    i++;

    // do this (print) every 50 frames because printing every
    // frame would bog down the Arduino
    if (i % 50 == 0)
    {
      sprintf(buf, "Detected %d:\n", blocks1);
      Serial.print(buf);
      for (j = 0; j < blocks1; j++)
      {
        sprintf(buf, "  block %d: ", j);
        Serial.print(buf);
        pixy1.blocks[j].print();
        s_c1_i = int(pixy1.blocks[j].signature);
/*        Serial.println(pixy1.blocks[j].signature);
        Serial.print(s_c1_i);
        Serial.print(" = 2");
        Serial.println(s_c1_i == 2); */
        if((s_c1_i == 2));
        {
          i_c1s1 = j;
          Serial.print("this is 1th if");
          sprintf(buf, "index [%d] has signature [%d] \n", j, int(pixy1.blocks[j].signature));
          Serial.print(buf);
        }
        Serial.println(s_c1_i == 3);
        if(0);
        {
          i_c1s2 = j;
          Serial.print("this is 2nd if");
          sprintf(buf, "index [%d] has signature [%d] \n", j, int(pixy1.blocks[j].signature));
          Serial.print(buf);
        }
      }
      sprintf(buf, "index of signature 2 is : %d \n", i_c1s1);
      Serial.print(buf);
      sprintf(buf, "index of signature 3 is : %d \n", i_c1s2);
      Serial.print(buf);

    }
  }

/*  i = 0;
  if (blocks2)
  {
    i++;

    // do this (print) every 50 frames because printing every
    // frame would bog down the Arduino
    if (i % 50 == 0)
    {
      sprintf(buf, "Detected %d:\n", blocks2);
      Serial.print(buf);
      for (j = 0; j < blocks2; j++)
      {
        sprintf(buf, "  block %d: ", j);
        Serial.print(buf);
        pixy2.blocks[j].print();

        Serial.println(pixy2.blocks[j].signature);
        if (pixy2.blocks[j].signature == 2);
        {
          i_c1s1 = j;
        }
        if (pixy2.blocks[j].signature == 3);
        {
          i_c1s2 = j;
        }
      }
      sprintf(buf, "index of signature 2 is : %d \n", i_c1s1);
      Serial.print(buf);
      sprintf(buf, "index of signature 3 is : %d \n", i_c1s2);
      Serial.print(buf);

    }
  }
*/
}

