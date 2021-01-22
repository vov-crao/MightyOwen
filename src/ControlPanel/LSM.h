#pragma once

bool LSM_3
( const int* X
, const float* Y
, const int Num
, float* A
, float* B
, float* C
);

bool test1();

const int BUFFER_LEN = 30;

class SolveEq3Cyclic
{
  // Buffers handling
  float m_X[BUFFER_LEN];
  float m_Y[BUFFER_LEN];
  int m_StartIndex = 0;
  int m_Num = 0;
/*
  // Calc variables
  float SumX = 0;
  float SumX2 = 0;
  float SumX3 = 0;
  float SumX4 = 0;

  float SumY = 0;
  float SumYX = 0;
  float SumYX2 = 0;
*/
  // Determinants
  float m_Det = 0;
  float m_Det1 = 0;
  float m_Det2 = 0;
  float m_Det3 = 0;
  
  // Coeffs
  // a2*X^2 + a1*X + a0 = Y
  float m_a0 = 0;
  float m_a1 = 0;
  float m_a2 = 0;
  
private:
  bool CalcSums();
  bool CalcDets();
  bool CalcCoeffs();
  
public:
  int Num() const { return m_Num; }

  // Solved
  float a0() const { return m_a0; }
  float a1() const { return m_a1; }
  float a2() const { return m_a2; }
  
  int GetIndex(const int Offset) const;
  
  float Xmax() const;
  float XmaxIn() const;
  float Ymax() const;
  
  void Add(const float X, const float Y);
  void Reset();
  bool SolveLSM();
};

bool test2();
