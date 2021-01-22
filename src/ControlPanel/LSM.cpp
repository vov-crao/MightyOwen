#include "LSM.h"

#include <stdint.h> 
#include <Arduino.h>

// Find LSM solution of equation A*X^2 + B*X + C = Y
/*********************************************************************/
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

/*********************************************************************/
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

/*********************************************************************/
bool SolveEq3Cyclic::CalcSums()
{
  SumX = 0;
  SumX2 = 0;
  SumX3 = 0;
  SumX4 = 0;

  SumY = 0;
  SumYX = 0;
  SumYX2 = 0;

  if (m_Num < 3)
    return false;

  const float X0 = m_X[m_StartIndex];
  for (int i = 0; i < m_Num; i++)
  {
    const int Index = (m_StartIndex + i) % BUFFER_LEN;
    float x = m_X[Index] - X0;
    const float y = m_Y[Index];
    const float X = x;

    SumY += y;
    SumYX += y*x;
    SumX += x;
    
    x *= X;
    SumX2 += x;
    SumYX2 += y*x;
    
    x *= X;
    SumX3 += x;
    
    x *= X;
    SumX4 += x;
  }

  return true;
}

/*********************************************************************/
bool SolveEq3Cyclic::CalcDets()
{
  m_Det = SumX4 * SumX2 * m_Num;
  m_Det += 2*(SumX3 * SumX2 * SumX);
  m_Det -= SumX2 * sq(SumX2);
  m_Det -= SumX4 * sq(SumX);
  m_Det -= m_Num * sq(SumX3);

  m_Det1 = SumYX2 * SumX2 * m_Num;
  m_Det1 += SumX2 * SumX * SumYX;
  m_Det1 += SumX3 * SumX * SumY;
  m_Det1 -= sq(SumX2) * SumY;
  m_Det1 -= sq(SumX) * SumYX2;
  m_Det1 -= SumX3 * SumYX * m_Num;

  m_Det2 = SumX4 * SumYX * m_Num;
  m_Det2 += SumX3 * SumX2 * SumY;
  m_Det2 += SumX2 * SumX * SumYX2;
  m_Det2 -= sq(SumX2) * SumYX;
  m_Det2 -= SumX4 * SumX * SumY;
  m_Det2 -= SumX3 * SumYX2 * m_Num;

  m_Det3 = SumX4 * SumX2 * SumY;
  m_Det3 += SumX3 * SumX * SumYX2;
  m_Det3 += SumX2 * SumX3 * SumYX;
  m_Det3 -= sq(SumX2) * SumYX2;
  m_Det3 -= sq(SumX3) * SumY;
  m_Det3 -= SumX4 * SumX * SumYX;

  if (m_Det < 1e-10)
    return false;

  return true;
}

/*********************************************************************/
bool SolveEq3Cyclic::CalcCoeffs()
{
  if (m_Det < 1e-10)
    return false;

  m_a2 = m_Det1 / m_Det;
  m_a1 = m_Det2 / m_Det;
  m_a0 = m_Det3 / m_Det;

  return true;
}

/*********************************************************************/
void SolveEq3Cyclic::Add(const float X, const float Y)
{
  Serial.print("Add(");
  Serial.print(X);
  Serial.print(",");
  Serial.print(Y);
  Serial.print(") ");

  int Index = m_StartIndex + m_Num;
  if (Index >= BUFFER_LEN)
    Index -= BUFFER_LEN;

  Serial.print(Index);
    
  m_X[Index] = X;
  m_Y[Index] = Y;

  if (m_Num >= BUFFER_LEN)
  {
    m_StartIndex++;
    if (m_StartIndex >= BUFFER_LEN)
      m_StartIndex = 0;
  }    
  else
    m_Num++;

  Serial.print(" S=");
  Serial.print(m_StartIndex);
  Serial.print(",N=");
  Serial.println(m_Num);
}

/*********************************************************************/
void SolveEq3Cyclic::Reset()
{
  m_StartIndex = 0;
  m_Num = 0;
}

/*********************************************************************/
bool SolveEq3Cyclic::SolveLSM()
{
  if (!CalcSums())
    return false;
  if (!CalcDets())
    return false;
  if (!CalcCoeffs())
    return false;

  return true;
}

/*********************************************************************/
bool test2()
{
  const int X[] =   {0, 1 ,2 ,3 ,4 ,5 ,6 ,7 ,8 ,9 ,10,11,12,13,14,15,16,17,18,19,20,21,22};
  const float Y[] = {19,20,20,21,22,23,23,24,24,25,25,26,26,26,27,27,27,28,28,28,28,29,29};

  Serial.print("Test2:");

  SolveEq3Cyclic Eq;

  const int Num = sizeof(X)/sizeof(X[0]);
  Serial.println(Num);

  for (int i = 0; i < Num/2; i++)
  {
    Eq.Add(0, 0);
  }

  for (int i = 0; i < Num; i++)
  {
    Eq.Add(X[i], Y[i]);
  }

  const bool Ok = Eq.SolveLSM();

  Serial.print("a2=");
  Serial.print(Eq.a2());
  Serial.print(",a1=");
  Serial.print(Eq.a1());
  Serial.print(",a0=");
  Serial.print(Eq.a0());
  Serial.print(" ");
  Serial.println(Ok ? "Ok" : "FAILED");

  return Ok;
}
