#include "Hotter.h"

/*********************************************************************/
HotterMotor::HotterMotor
( const byte MotorMax
, const byte MotorMin
, const int TempTarget
, const int TempGist
)
  : m_MotorHigh(max(MotorMin, MotorMax))
  , m_MotorLow(min(MotorMin, MotorMax))
  , m_TempTarget(TempTarget)
  , m_TempGist(TempGist)
  , m_MotorDelta((m_MotorHigh - m_MotorLow)/2)
{
}

/*********************************************************************/
void HotterMotor::AddTemp(const int T)
{
  int NewMotorSpeed = m_MotorCurrent;
  if (T > (m_TempTarget + 4*m_TempGist))
  {
    NewMotorSpeed = m_MotorLow;
  }
  else if (T > (m_TempTarget + 2*m_TempGist))
  {
    if (T > m_TempLast)
      NewMotorSpeed -= m_MotorDelta;
  }
  else if (T > (m_TempTarget + m_TempGist))
  {
    if (T > m_TempLast)
      NewMotorSpeed -= m_MotorDelta / 2;
  }
  else if (T < (m_TempTarget - m_TempGist))
  {
    if (T < m_TempLast)
      NewMotorSpeed += m_MotorDelta / 2;
  }
  else if (T < (m_TempTarget - 2*m_TempGist))
  {
    if (T < m_TempLast)
      NewMotorSpeed += m_MotorDelta;
  }
  else if (T < (m_TempTarget - 4*m_TempGist))
  {
    NewMotorSpeed = m_MotorHigh;
  }

  if (NewMotorSpeed > m_MotorHigh)
    NewMotorSpeed = m_MotorHigh;
  if (NewMotorSpeed < m_MotorLow)
    NewMotorSpeed = m_MotorLow;

  Serial.print("T=");
  Serial.print(T);
  Serial.print(",*MotV=");
  Serial.print(NewMotorSpeed);
  Serial.print(",MV=");
  Serial.print(m_MotorCurrent);
  
  m_TempLast = T;

  const unsigned long CurTime = millis();
  if ((NewMotorSpeed != m_MotorCurrent) && ((m_LastMotorCorrection + 10000) < CurTime))
  {
    Serial.print("*");

    const unsigned long DT = CurTime - m_LastMotorCorrection;
    const float CubesIncm3 = float(DT) * m_MotorCurrent / 4266.0 * 128 / 1000;
    CubesSum += CubesIncm3;

    Serial.print(",dT=");
    Serial.print(DT);
    Serial.print(",dV=");
    Serial.print(CubesIncm3);
    Serial.print("/");
    Serial.print(CubesSum);

    m_MotorCurrent = byte(NewMotorSpeed);

    m_LastMotorCorrection = CurTime;
  }

  Serial.println();
}
