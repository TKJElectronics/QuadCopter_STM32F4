/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"
#include "IMU_Vectors.h"

// Computes the dot product of two vectors
float Vector_Dot_Product(float vector1[3], float vector2[3])
{
  float op=0;

  for(int c=0; c<3; c++)
  {
    op+=vector1[c]*vector2[c];
  }

  return op;
}

// Computes the cross product of two vectors
void Vector_Cross_Product(float vectorOut[3], float v1[3], float v2[3])
{
  vectorOut[0]= (v1[1]*v2[2]) - (v1[2]*v2[1]);
  vectorOut[1]= (v1[2]*v2[0]) - (v1[0]*v2[2]);
  vectorOut[2]= (v1[0]*v2[1]) - (v1[1]*v2[0]);
}

// Multiply the vector by a scalar.
void Vector_Scale(float vectorOut[3], float vectorIn[3], float scale2)
{
  for(int c=0; c<3; c++)
  {
    vectorOut[c]=vectorIn[c]*scale2;
  }
}

// Adds two vectors
void Vector_Add(float vectorOut[3], float vectorIn1[3], float vectorIn2[3])
{
  for(int c=0; c<3; c++)
  {
    vectorOut[c]=vectorIn1[c]+vectorIn2[c];
  }
}

//Multiply two 3x3 matrixs. This function developed by Jordi can be easily adapted to multiple n*n matrix's. (Pero me da flojera!).
void Matrix_Multiply(float a[3][3], float b[3][3],float mat[3][3])
{
  float op[3];
  for(int x=0; x<3; x++)
  {
    for(int y=0; y<3; y++)
    {
      for(int w=0; w<3; w++)
      {
       op[w]=a[x][w]*b[w][y];
      }
      mat[x][y]=0;
      mat[x][y]=op[0]+op[1]+op[2];
    }
  }
}

// Init rotation matrix using euler angles
void init_rotation_matrix(float m[3][3], float yaw, float pitch, float roll)
{
  float c1 = cos(roll);
  float s1 = sin(roll);
  float c2 = cos(pitch);
  float s2 = sin(pitch);
  float c3 = cos(yaw);
  float s3 = sin(yaw);

  // Euler angles, right-handed, intrinsic, XYZ convention
  // (which means: rotate around body axes Z, Y', X'')
  m[0][0] = c2 * c3;
  m[0][1] = c3 * s1 * s2 - c1 * s3;
  m[0][2] = s1 * s3 + c1 * c3 * s2;

  m[1][0] = c2 * s3;
  m[1][1] = c1 * c3 + s1 * s2 * s3;
  m[1][2] = c1 * s2 * s3 - c3 * s1;

  m[2][0] = -s2;
  m[2][1] = c2 * s1;
  m[2][2] = c1 * c2;
}

float constrain(float x, float a, float b)
{
	if (x >= a && x <= b)
		return x;
	else if (x < a)
		return a;
	else if (x > b)
		return b;
	else
		return 0;
}

float findMedianFloat(float *data, int arraySize)
{
  float temp;
  bool done = 0;
  u8 i;

   // Sorts numbers from lowest to highest
  while (done != 1)
  {
    done = 1;
    for (i=0; i<(arraySize-1); i++)
	{
      if (data[i] > data[i+1])
	  {     // numbers are out of order - swap
        temp = data[i+1];
        data[i+1] = data[i];
        data[i] = temp;
        done = 0;
      }
    }
  }

  return data[arraySize/2]; // return the median value
}

// Low pass filter, kept as regular C function for speed
float filterSmooth(float currentData, float previousData, float smoothFactor)
{
  if (smoothFactor != 1.0) //only apply time compensated filter if smoothFactor is applied
  {
    return (previousData * (1.0 - smoothFactor) + (currentData * smoothFactor));
  }
  return currentData; //if smoothFactor == 1.0, do not calculate, just bypass!
}

float low_pass_filter(float vNew,float vPrev,float factor) {
	return (vPrev*factor + vNew) / ( 1 + factor);
}
