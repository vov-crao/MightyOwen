#include "tempDS18B20.h"

/*********************************************************************/
ds18b20::ds18b20(const byte Pin, const eResolution Resolution)
  : m_resolution(Resolution)
  , m_pin(Pin)
  , m_wire(Pin)
  , m_IsPresent(m_wire.reset() != 0)
{
}

/*********************************************************************/
bool ds18b20::newMeasure(bool IsAsync)
{
  bool IsPOR = false;
  for (byte i = 0; i < 25; i++)
  {
    m_IsWorking = false;
    m_IsPresent = false;
    
    if (m_wire.reset() == 0)
      return false;
      
    m_wire.write(0xCC);
    m_wire.write(0x44); // measure temperature

    if (m_wire.reset() == 0)
      return false;

    m_IsPresent = true;
    
    // Async measuring only send command to measure temperature.
    // The result will be available latter (read bit == 1)
    // In async just read previous temperature value. The next read (in 1 sec) will read actual data.
    if (!IsAsync)
    {
      // wait for the actual data
      while (m_wire.read_bit() == 0)
      {
        delayMicroseconds(60); 
      }
    }
    
    m_wire.write(0xCC);
    m_wire.write(0xBE);

    m_wire.read_bytes(m_data, 9);

    const byte CRC = m_wire.crc8(m_data, 8);

    if (CRC == m_data[8])
    {
      // Power-on-reset value detected. The next value will be actual.
      // The actual value can compare with POR value, so second one is anyway actual.
      if (m_data[1] == 0x05 && m_data[0] == 0x50)
      {
        if (!IsPOR)
        {
          IsPOR = true;
          // Get actual data now.
          IsAsync = false;
          continue;
        }
      }
        
      m_IsWorking = true;
      return true;
    }
  }

  m_IsWorking = false;
  return false;
}

/*********************************************************************/
int ds18b20::getNewTemp()
{
  if (newMeasure())
    return getLastTemp();

  return 0;
}

/*********************************************************************/
byte ds18b20::GetResolutionBits()
{
  if (newMeasure())
    return 9 + ((m_data[4] >> 5) & 0x3);

  return 12;
}

/*********************************************************************/
int ds18b20::getLastTemp() const
{
  int T = getLastTempRaw();
  T += 1 << 3; // round temp on 0.5C
      
  return T >> 4;
}

/*********************************************************************/
int ds18b20::getLastTempRaw() const
{
  return (m_data[1] << 8) | m_data[0];
}

/*********************************************************************/
float ds18b20::getLastFloatTemp() const
{
  const int T = getLastTempRaw();
      
  return float(T) / 16.0;
}
