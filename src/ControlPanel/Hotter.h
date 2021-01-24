#pragma once

#include <stdint.h> 
#include <Arduino.h>
/*******************************************************
 * 
 * 
 * 
 *******************************************************/
class HotterMotor
{
  // Motor speed handling
  byte m_MotorHigh = 0;
  byte m_MotorLow = 0;
  byte m_MotorCurrent = 0;
  char m_MotorDelta = 0;

  int m_TempTarget = 0;
  int m_TempGist = 0;
  int m_TempLast = 0;

  unsigned long m_LastMotorCorrection = 0;

  float CubesSum = 0;
  
public:
  HotterMotor(const byte MotorMax, const byte MotorMin, const int TempTarget, const int TempGist);

  byte GetMotorHigh() const { return m_MotorHigh; }
  byte GetMotorLow() const { return m_MotorLow; }
  byte GetMotorCur() const { return m_MotorCurrent; }

  void AddTemp(const int T);
};
