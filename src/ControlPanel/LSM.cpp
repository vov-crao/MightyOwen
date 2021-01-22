#include "LSM.h"

#include <stdint.h> 
#include <Arduino.h>

bool LSM_3
( const int* X
, const float* Y
, const int Num
, float* A
, float* B
, float* C
)
{
  float SumX = 0;
  float SumX2 = 0;
  float SumX3 = 0;
  float SumX4 = 0;

  float SumY = 0;
  float SumYX = 0;
  float SumYX2 = 0;

  for (int i = 0; i < Num; i++)
  {
    float x = X[i];
    float y = Y[i];

    SumY += y;
    SumYX += y*x;
    SumX += x;
    
    x *= x;
    SumX2 += x;
    SumYX2 += y*x;
    
    x *= X[i];
    SumX3 += x;
    
    x *= X[i];
    SumX4 += x;
  }

  float Det = SumX4 * SumX2 * Num;
  Det += 2*(SumX3 * SumX2 * SumX);
  Det -= SumX2 * sq(SumX2);
  Det -= SumX4 * sq(SumX);
  Det -= Num * sq(SumX3);

  float Det1 = SumYX2 * SumX2 * Num;
  Det1 += SumX2 * SumX * SumYX;
  Det1 += SumX3 * SumX * SumY;
  Det1 -= sq(SumX2) * SumY;
  Det1 -= sq(SumX) * SumYX2;
  Det1 -= SumX3 * SumYX * Num;

  float Det2 = SumX4 * SumYX * Num;
  Det2 += SumX3 * SumX2 * SumY;
  Det2 += SumX2 * SumX * SumYX2;
  Det2 -= sq(SumX2) * SumYX;
  Det2 -= SumX4 * SumX * SumY;
  Det2 -= SumX3 * SumYX2 * Num;

  float Det3 = SumX4 * SumX2 * SumY;
  Det3 += SumX3 * SumX * SumYX2;
  Det3 += SumX2 * SumX3 * SumYX;
  Det3 -= sq(SumX2) * SumYX2;
  Det3 -= sq(SumX3) * SumY;
  Det3 -= SumX4 * SumX * SumYX;

  if (Det < 1e-10)
    return false;
    
  if (A) *A = Det1 / Det;
  if (B) *B = Det2 / Det;
  if (C) *C = Det3 / Det;

  return true;
}

bool test1()
{
  const int X[] =   {0, 1 ,2 ,3 ,4 ,5 ,6 ,7 ,8 ,9 ,10,11,12,13,14,15,16,17,18,19,20,21,22};
  const float Y[] = {19,20,20,21,22,23,23,24,24,25,25,26,26,26,27,27,27,28,28,28,28,29,29};

  Serial.println("Test1:");
  
  float A=0,B=0,C=0;
  const bool Ok = LSM_3(X, Y, sizeof(X)/sizeof(X[0]), &A, &B, &C);

  Serial.print("A=");
  Serial.print(A);
  Serial.print(",B=");
  Serial.print(B);
  Serial.print(",C=");
  Serial.print(C);
  Serial.print(" ");
  Serial.println(Ok ? "Ok" : "FAILED");

  return Ok;
}
