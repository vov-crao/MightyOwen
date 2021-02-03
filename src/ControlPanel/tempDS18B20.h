#pragma once

#include <OneWire.h>

class ds18b20
{
public:
  typedef enum
  {
      d8 = 0
    , d4
    , d2
    , d1
  } eResolution;
  
private:
  byte m_data[9];
  byte m_resolution = d8;
  byte m_pin = 255;
  OneWire m_wire;
  bool m_IsPresent = true;
  bool m_IsWorking = false;
  bool m_IsBusy = false;
  
public:
  ds18b20(const byte Pin, const eResolution Resolution = d8);

  bool IsPresent() const { return m_IsPresent; }
  bool IsWorking() const { return m_IsWorking; }
  bool IsBusy() const { return m_IsBusy; }
  
  byte GetResolutionBits();
  
  int getLastTemp() const;
  int getLastTempRaw() const;
  float getLastFloatTemp() const;
  int getNewTemp();
  int getFreshTemp();
  bool newMeasure(bool IsAsync = true);
};
