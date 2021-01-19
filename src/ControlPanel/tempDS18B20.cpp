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
bool ds18b20::read()
{
  for (byte i = 0; i < 2; i++)
  {
    if (m_wire.reset() == 0)
      return false;
    m_wire.write(0xCC);
    m_wire.write(0x44); // measure temperature
  //delay(500);
 
    if (m_wire.reset() == 0)
      return false;
        
    m_wire.write(0xCC);
    m_wire.write(0xBE);

    m_wire.read_bytes(m_data, 9);
    const byte CRC = m_wire.crc8(m_data, 8);

    if (CRC == m_data[8])
    {
      return true;
    }
  }

  return false;
}

/*********************************************************************/
int ds18b20::getTemp()
{
  if (read())
  {
    int T = (m_data[1] << 8) | m_data[0];
    return T >> 4;
  }

  return 0;
}
