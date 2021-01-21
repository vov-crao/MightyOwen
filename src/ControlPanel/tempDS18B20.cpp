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
bool ds18b20::newMeasure()
{
  for (byte i = 0; i < 3; i++)
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
    
    m_wire.write(0xCC);
    m_wire.write(0xBE);

    m_wire.read_bytes(m_data, 9);
    const byte CRC = m_wire.crc8(m_data, 8);

    if (CRC == m_data[8])
    {
      // Power-on-reset value detected. Get nex value.
      if (m_data[1] == 0x05 && m_data[0] == 0x50)
        continue;
        
      m_IsWorking = true;
      return true;
    }
  }

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
int ds18b20::getLastTemp()
{
  int T = (m_data[1] << 8) | m_data[0];
  T += 1 << 3; // round temp on 0.5C
      
  return T >> 4;
}
