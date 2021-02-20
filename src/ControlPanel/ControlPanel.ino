#include <StaticThreadController.h>
#include <Thread.h>

#include <LiquidCrystal_I2C.h> // библиотека для 4 строчного дисплея
LiquidCrystal_I2C lcd(0x27,20,4);  // Устанавливаем дисплей

#include <EEPROM.h>

#define VERSION_MAJOR 1
#define VERSION_MIDDLE 0
#define VERSION_MINOR 1

// Start button
#define START_BUTTON_PIN 11

// Start LED
#define START_LED_PIN A2

// Blinking led
#define BLINKING_LED_PIN 13

// Temperature sensors pins
#define WATER_SENSOR_PIN 7
#define EXHOST_SENSOR_PIN 10

// Encoder buttons on pins
#define ENCODER_A_PIN 2
#define ENCODER_B_PIN 3
#define ENCODER_C_PIN 4

enum eEncoderState {eNone, eLeft, eRight, eButton};
bool ButtonPrev;

// EEPROM indexes of global variables
const byte STORE_FUEL_SPEED_MAX  = 0;
const byte STORE_FUEL_SPEED_MIN  = 1;
const byte STORE_TARGET_TEMP     = 2;
const byte STORE_MOTOR_MAX       = 3;
const byte STORE_MOTOR_MIN       = 4;
const byte STORE_TEMP_MAX        = 5;
const byte STORE_TEMP_GIST       = 6;
const byte STORE_INDEX_MAX       = 7;

byte VarIndex = STORE_INDEX_MAX;

byte StoreValueUpdatedFlags = 0xFF;

// Stepper Motor
#define STEPPER_MOTOR_PULSE_PIN 9
#define STEPPER_MOTOR_DIR_PIN   8

//
//****************************************************************************************

byte motorSpeedMax = 0;// скорость максимальная
byte motorSpeedMin = 0; // скорость минимальная 
byte motorSpeedCurrent = 0;
int t1 = 0; // выставляемая пользователем температура теплоносителя
int T_max_avar = 0;  // температура отключения мотора верхняя
int T_min_avar = 0;  // температура отключения мотора нижняя
int t3 = 0;           // температура включения защиты
int GST = 0;         // Гистерезис терморегулятора

int t2 = 0; // температура передаваемая с термодатчика
const int SteppingMotorHz = 6400 / 5 / 10; // 6400 pulses to make full turn for 5 sec using speed 10
bool IsMaxTempReached = false;
bool StartButtonPressed = false; //кнопка ПУСК

unsigned long g_LastTimeWorkingTemp = 0;

Thread ledThread = Thread(); // создаём поток управления светодиодом
Thread soundThread = Thread(); // создаём поток управления 

//****************************************************************************************
class SerialLog
{
public:
  static inline 
  void begin(const int SpeedBaud)
  {
    Serial.begin(SpeedBaud);
  }
  
  template <typename T>
  inline
  SerialLog& operator <<(const T& V)
  {
    Serial.print(V);
    return *this;
  }

};

//****************************************************************************************
class NL
{
};

/*********************************************************************/
template<>
SerialLog& SerialLog::operator << <NL>(const NL&)
{
  Serial.println();
  return *this;
}

/*********************************************************************/

SerialLog LOG;

//****************************************************************************************
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
  
public:
  ds18b20(const byte Pin, const eResolution Resolution = d8);

  bool IsPresent() const { return m_IsPresent; }
  bool IsWorking() const { return m_IsWorking; }
  byte GetResolutionBits();
  
  int getLastTemp() const;
  int getLastTempRaw() const;
  float getLastFloatTemp() const;
  int getNewTemp();
  bool newMeasure(bool IsAsync = true);
};

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
  for (byte i = 0; i < 5; i++)
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

//****************************************************************************************

ds18b20 TempWater(WATER_SENSOR_PIN);

//****************************************************************************************
void printVersion() 
{
  LOG << "Software version: " << int(VERSION_MAJOR) << "." << int(VERSION_MIDDLE) << "." << int(VERSION_MINOR) << NL();
}

//****************************************************************************************

/* Pin to interrupt map:
* D0-D7 = PCINT 16-23 = PCIR2 = PD = PCIE2 = pcmsk2
* D8-D13 = PCINT 0-5 = PCIR0 = PB = PCIE0 = pcmsk0
* A0-A5 (D14-D19) = PCINT 8-13 = PCIR1 = PC = PCIE1 = pcmsk1
*/

#include <avr/interrupt.h>  
#include <avr/io.h>
#include <util/atomic.h>

bool pinA_prev = digitalRead(ENCODER_A_PIN);
bool pinB_prev = digitalRead(ENCODER_B_PIN);

volatile int8_t rotateTicks = 0;

/*********************************************************************/
ISR(PCINT2_vect )
{
  // A
  const bool pinA = digitalRead(ENCODER_A_PIN) != LOW;
  const bool pinB = digitalRead(ENCODER_B_PIN) != LOW;

  if (pinA != pinA_prev)
  {
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
      if (pinA == pinB)
        rotateTicks--; // if A changed twice - we returns back without affecting B
      else
        rotateTicks++;
    }
    
    pinA_prev = pinA;
  }

  // B
  if (pinB != pinB_prev)
  {
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
      if (pinA == pinB)
        rotateTicks++; 
      else
        rotateTicks--;
    }
      
    pinB_prev = pinB;
  }

  // Correct ticks if any inconsistency
  if (pinA == pinB)
  {
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
      rotateTicks &= ~1;
    }
  }
}

/*********************************************************************/
eEncoderState GetEncoderStateISR() 
{
  eEncoderState Result = eNone;

  if (digitalRead(ENCODER_C_PIN) == LOW) 
  {
    if (ButtonPrev) 
    {
        Result = eButton;
        ButtonPrev = 0;
    }
  }
  else 
  {
    ButtonPrev = 1;

    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
      if ((rotateTicks & 0x3) == 0)
      {
        if (rotateTicks < 0)
        {
          Result = eLeft;
          rotateTicks += 4;
        }
        else if (rotateTicks > 0)
        {
          Result = eRight;
          rotateTicks -= 4;
        }
      }
    }
  }
    
  return Result;
}

byte g_DefaultFactory[] = 
{
    20    // STORE_FUEL_SPEED_MAX
  , 10    // STORE_FUEL_SPEED_MIN
  , 60    // STORE_TARGET_TEMP
  , 95    // STORE_MOTOR_MAX
  , 50    // STORE_MOTOR_MIN
  , 95    // STORE_TEMP_MAX
  , 4     // STORE_TEMP_GIST
};

byte g_DefaultLimits[] = 
{
    11, 110   // STORE_FUEL_SPEED_MAX
  , 10, 109   // STORE_FUEL_SPEED_MIN
  , 10, 95    // STORE_TARGET_TEMP
  , 25, 95    // STORE_MOTOR_MAX
  , 25, 95    // STORE_MOTOR_MIN
  , 25, 95    // STORE_TEMP_MAX
  , 1, 10     // STORE_TEMP_GIST
};

/*********************************************************************/
void resetStorageToFactoryDefaults() 
{
  LOG << "Restore Factory Defaults:" << NL(); 
  
  for (byte i = 0; i < STORE_INDEX_MAX; i++)
  {
    const byte Value = g_DefaultFactory[i];

    LOG << " " << i << "=" << Value << NL();

    EEPROM.write(i, Value);
  }

  LOG << "Restored Factory Defaults!" << NL();
}

/*********************************************************************/
bool fixStorageToCorrectValues() 
{
  bool Ok = true;
  
  LOG << "Fix EEPROM to correct:" << NL(); 
  
  for (byte i = 0; i < STORE_INDEX_MAX; i++)
  {
    const byte Value = EEPROM.read(i);
    const byte Min = g_DefaultLimits[i*2];
    const byte Max = g_DefaultLimits[i*2+1];

    LOG << " " << i << "=" << Value << " " << Min << "<" << Max << NL();

    if ((Value < Min) || (Value > Max))
    {
      LOG << "  Wrong!" << NL(); 

      const byte NewValue = g_DefaultFactory[i];

      LOG << i << "<=" << NewValue << NL();

      EEPROM.write(i, NewValue);
      Ok = false;
    }
  }

  LOG << (Ok ? "Ok!" : "Fixed!") << NL();

  return Ok;
}

/*********************************************************************/
void readStorageValues() 
{
  for (byte i = 0; i < STORE_INDEX_MAX; i++)
  {
    const byte Value = EEPROM.read(i);

    switch (i)
    {
      case STORE_FUEL_SPEED_MAX:  motorSpeedMax = Value; break;
      case STORE_FUEL_SPEED_MIN:  motorSpeedMin = Value; break;
      case STORE_TARGET_TEMP:     t1 = Value; break;
      case STORE_MOTOR_MAX:       T_max_avar = Value; break;
      case STORE_MOTOR_MIN:       T_min_avar = Value; break;
      case STORE_TEMP_MAX:        t3 = Value; break;
      case STORE_TEMP_GIST:       GST = Value; break;
      default: break;
    }
  }
}

/*********************************************************************/
void SetMotorSpeed(const byte NewSpeed) 
{ 
  motorSpeedCurrent = NewSpeed;
  if (motorSpeedCurrent == 0)
  {
    noTone(STEPPER_MOTOR_PULSE_PIN);

    LOG << "STOP MOTOR!!" << NL();
  }
  else
  {
    const int motorSpeed = int(motorSpeedCurrent) * SteppingMotorHz;
    tone(STEPPER_MOTOR_PULSE_PIN, motorSpeed); 

    LOG << "Motor Speed=" << motorSpeedCurrent << NL();
  }
}

/*********************************************************************/
void setup() 
{
  Serial.begin(9600);

  // SW version
  printVersion();
  
  // Fix EEPROM for correctness
  fixStorageToCorrectValues();

  // Restore variables from Storage
  readStorageValues();
  
  // Start LED
  pinMode(START_LED_PIN, OUTPUT);
  if (!StartButtonPressed) 
    digitalWrite(START_LED_PIN, HIGH);

    // Start button.
    pinMode(START_BUTTON_PIN, INPUT);
    digitalWrite(START_BUTTON_PIN, HIGH); // use pull up resistor
   
  t2 = TempWater.getNewTemp();

  Wire.begin();//Инициализирует библиотеку Wire и подключается к шине I2C как ведущий (мастер) или ведомый.
  Wire.beginTransmission(0x27);
  
  lcd.begin(20, 4); // initialize the lcd
  
  pinMode(STEPPER_MOTOR_DIR_PIN, OUTPUT);      
  pinMode(STEPPER_MOTOR_PULSE_PIN, OUTPUT); 

  SetMotorSpeed(0);

  pinMode(BLINKING_LED_PIN, OUTPUT);
     
    ledThread.onRun(ledBlink);  // назначаем потоку задачу
    ledThread.setInterval(1000); // задаём интервал срабатывания, мсек
        
    soundThread.onRun(sound);     // назначаем потоку задачу
    soundThread.setInterval(150); // задаём интервал срабатывания, мсек

    // Encoder pins
    pinMode(ENCODER_A_PIN, INPUT); // PCINT18
    pinMode(ENCODER_B_PIN, INPUT);  // PCINT19
    pinMode(ENCODER_C_PIN, INPUT_PULLUP); // Кнопка не подтянута к +5 поэтому задействуем внутренний pull-up резистор

    // Disable soft interrupts on all pins.
    PCMSK2 = 0;
    PCMSK1 = 0;
    PCMSK0 = 0;

    // pin change interrupt 2 is enabled 
    PCICR |= (1 << PCIE2);
    ///TODO: Add detecting PCINT numbers from pins
    // pins 4,5,6
    // PCMSK2 = (1<<PCINT20) | (1<<PCINT21); // Encoder left/right buttons
    // pins 2,3,4
    PCMSK2 = (1<<PCINT18) | (1<<PCINT19); // Encoder left/right buttons
}

byte StoreCurrentValue = 0;

/*********************************************************************/
void CheckLimitStoreVar()
{
  if (StoreCurrentValue < 1)
  {
    StoreCurrentValue = 1;
    return;
  }
  
  switch(VarIndex)
  {
    case STORE_FUEL_SPEED_MAX: 
      {
        StoreCurrentValue = constrain(StoreCurrentValue, EEPROM.read(STORE_FUEL_SPEED_MIN)+1, 110);
        break;
      }
    case STORE_FUEL_SPEED_MIN:
      {
        StoreCurrentValue = constrain(StoreCurrentValue, 10, EEPROM.read(STORE_FUEL_SPEED_MAX)-1);
        break;
      }
    case STORE_TARGET_TEMP:
      {
        StoreCurrentValue = constrain(StoreCurrentValue, 10, 95);
        break;
      }
    case STORE_MOTOR_MAX:
      {
        StoreCurrentValue = constrain(StoreCurrentValue, 25, 95);
        break;
      }
    case STORE_MOTOR_MIN:
      {
        StoreCurrentValue = constrain(StoreCurrentValue, 25, 95);
        break;
      }
    case STORE_TEMP_MAX:
      {
        StoreCurrentValue = constrain(StoreCurrentValue, 25, 95);
        break;
      }
    case STORE_TEMP_GIST:
      {
        StoreCurrentValue = constrain(StoreCurrentValue, 1, 10);
        break;
      }
    default: break;
  }
}

/*********************************************************************/
void SaveUpdatedVarToStoreVar() 
{
  // Store prev updated value
  if (VarIndex < STORE_INDEX_MAX)
  {
    const byte OldValue = EEPROM.read(VarIndex);
    LOG << " Old value:" << OldValue;

    if (StoreCurrentValue != OldValue)
    {
      LOG << " Write new value:" << StoreCurrentValue;
            
      EEPROM.write(VarIndex, StoreCurrentValue);

      // MArk this Index as updated - it need to be displayed with new value;
      StoreValueUpdatedFlags |= 1 << VarIndex;
    }
    LOG << NL();
  }
}

/*********************************************************************/
void UpdateVarWithStoreVar()
{
  switch(VarIndex)
  {
    case STORE_FUEL_SPEED_MAX:  motorSpeedMax = StoreCurrentValue; break;
    case STORE_FUEL_SPEED_MIN:  motorSpeedMin = StoreCurrentValue; break;
    case STORE_TARGET_TEMP:     t1 = StoreCurrentValue; break;
    case STORE_MOTOR_MAX:       T_max_avar = StoreCurrentValue; break;
    case STORE_MOTOR_MIN:       T_min_avar = StoreCurrentValue; break;
    case STORE_TEMP_MAX:        t3 = StoreCurrentValue; break;
    case STORE_TEMP_GIST:       GST = StoreCurrentValue; break;
    default: break;
  }

  SaveUpdatedVarToStoreVar();
}

/*********************************************************************/
void loop() 
{
    // Start button pressed
    if (!StartButtonPressed && digitalRead(START_BUTTON_PIN) == LOW) 
    {
      StartButtonPressed = true;  
      // Start LED off
      digitalWrite(START_LED_PIN, LOW); 
      LOG << "Start button pressed" << NL(); 
    }
/*
    if (Serial.available() > 0)
    {
      int Symbol = 0;
      Symbol = Serial.read();
      Serial.print("#>");
      Serial.println(Symbol, HEX);
      if (Symbol == 0x30)
      {
        StartButtonPressed = true;
        digitalWrite(START_LED_PIN, LOW); 
        Serial.println("Start button pressed"); 
      }
    }
*/    
    if (ledThread.shouldRun())
        ledThread.run(); // запускаем поток
    
    if (soundThread.shouldRun())
        soundThread.run(); // запускаем поток

  switch (GetEncoderStateISR()) 
  {
    case eNone: return;
    
    case eLeft: 
      {
        StoreCurrentValue--;
        LOG << "Left turn:" << StoreCurrentValue << NL();

        CheckLimitStoreVar();
        break;
      }
      
    case eRight:  
      {
        StoreCurrentValue++;

        LOG << "Right turn:" << StoreCurrentValue << NL();

        CheckLimitStoreVar();
        break;
      }

    // Pressed Encoder button
    case eButton: 
      {
        LOG << "Knob pressed. Old index:" << VarIndex << NL();
        
        ++VarIndex;

        if (VarIndex > STORE_INDEX_MAX)
          VarIndex = 0;

        LOG << " New index:" << VarIndex << NL();

        if (VarIndex < STORE_INDEX_MAX)
        {
          StoreCurrentValue = EEPROM.read(VarIndex);
          LOG << " ee value:" << StoreCurrentValue << NL();
        }
          
        StoreValueUpdatedFlags = 0xFF;
      }
  }

  UpdateVarWithStoreVar();
}

// Поток светодиода:
/*********************************************************************/
void ledBlink() 
{ 
    static bool ledStatus = false;    // состояние светодиода Вкл/Выкл
    ledStatus = !ledStatus;           // инвертируем состояние
    digitalWrite(BLINKING_LED_PIN, ledStatus);  // включаем/выключаем светодиод
}

/*********************************************************************/
byte PrintMarker(const byte Index)
{
  if (VarIndex == Index) 
  {
    if (StoreCurrentValue == EEPROM.read(VarIndex))
      return lcd.print("<");    
    return lcd.print("*");
  }
  return lcd.print(" ");
}

/*********************************************************************/
void PrintValueOn1Line(const byte Col, const byte Row, const char* Descr, const int Value, const byte ValueWidth, const byte Index)
{
  if (StoreValueUpdatedFlags & (1 << Index))
  {
    lcd.setCursor(Col, Row);  
    lcd.print(Descr);

    byte Offset = lcd.print(Value);
    Offset += PrintMarker(Index);
    
    for (; Offset < ValueWidth; Offset++)
      lcd.print(' ');
      
    StoreValueUpdatedFlags &= ~(1 << Index);
  }
}

/*********************************************************************/
void PrintValueOn2Line(const byte Col, const byte Row, const char* Descr, const int Value, const byte ValueWidth, const byte Index, const byte Shift = 1)
{
  if (StoreValueUpdatedFlags & (1 << Index))
  {
    lcd.setCursor(Col, Row);  
    lcd.print(Descr);
    lcd.setCursor(Col+Shift, Row+1);  

    byte Offset = lcd.print(Value);
    Offset += PrintMarker(Index);
    
    for (; Offset < ValueWidth; Offset++)
      lcd.print(' ');
      
    StoreValueUpdatedFlags &= ~(1 << Index);
  }
}

const byte SCREEN_INDEX_MAIN  = 0;
const byte SCREEN_INDEX_MOTOR_TEST1  = 1;

byte ScreenIndex = SCREEN_INDEX_MAIN;

/*********************************************************************/
void PrintScreen() 
{ 
  lcd.setBacklight(255);

  if (ScreenIndex == SCREEN_INDEX_MAIN)
  {
    PrintValueOn1Line(0, 0, "Vmax=", motorSpeedMax, 4, STORE_FUEL_SPEED_MAX);
    PrintValueOn1Line(0, 1, "Vmin=", motorSpeedMin, 3, STORE_FUEL_SPEED_MIN);
    PrintValueOn1Line(13, 0, "t1=", t1, 3, STORE_TARGET_TEMP);

    PrintValueOn2Line(0, 2, "Tmax", T_max_avar, 4, STORE_MOTOR_MAX);
    PrintValueOn2Line(5, 2, "Tmin", T_min_avar, 4, STORE_MOTOR_MIN);
    PrintValueOn2Line(10, 2, "t3", t3, 3, STORE_TEMP_MAX, 0);
    PrintValueOn2Line(13, 2, "Gst", GST, 3, STORE_TEMP_GIST);
  }
  else if (ScreenIndex == SCREEN_INDEX_MOTOR_TEST1)
  {
    PrintValueOn1Line(0, 0, "Vmax=", motorSpeedMax, 4, STORE_FUEL_SPEED_MAX);
    PrintValueOn1Line(0, 1, "Vmin=", motorSpeedMin, 3, STORE_FUEL_SPEED_MIN);
    PrintValueOn1Line(9, 0, "Vh=", motorSpeedMax, 4, STORE_FUEL_SPEED_MAX);
    PrintValueOn1Line(9, 1, "Vl=", motorSpeedMin, 3, STORE_FUEL_SPEED_MIN);
  }
}

/*********************************************************************/
void sound() 
{ 
  PrintScreen();
  
  //-----
  static unsigned long s_NextTempUpdate = 0;

  const unsigned long CurrTime = millis();

  if (CurrTime >= s_NextTempUpdate)
  {
    const byte tw = TempWater.getNewTemp();

    if (TempWater.IsWorking())
    {
      t2 = tw;
      g_LastTimeWorkingTemp = CurrTime;
    }

    LOG << "Water(";
    if (TempWater.IsPresent())
      LOG << "P";
    if (TempWater.IsWorking())
      LOG << "W";
    LOG << ") temp=" << t2 << NL();

    if (ScreenIndex == SCREEN_INDEX_MAIN)
    {
      lcd.setCursor(13,1);  
      lcd.print("t2=     ");
      lcd.setCursor(16,1);
      if (!TempWater.IsPresent())
        lcd.print("--");
      else if (!TempWater.IsWorking())
        lcd.print("??");
      else
        lcd.print(t2);
    }

    // Update full screen periodically
    StoreValueUpdatedFlags = 0xFF;

    s_NextTempUpdate = CurrTime + 1000;
  }

  ///TODO: Add delay of 20 sec to off motor. 
  // Add pressing start button for 2 sec to start motor again.
  if (!TempWater.IsWorking())
  {
    if (g_LastTimeWorkingTemp + 20000ul < CurrTime)
    {
      LOG << "Temp sensor is out of order for 20 sec. STOP MOTOR!" << NL();
      StartButtonPressed = false;
      digitalWrite(START_LED_PIN, HIGH);

      SetMotorSpeed(0);

      g_LastTimeWorkingTemp = CurrTime;

      return;
    }
  }
    
  if (t2 >= t3) 
  {
    IsMaxTempReached = true;
    LOG << "MAX temp!" << NL();
  } 
     
  if (StartButtonPressed) 
  {
    const byte MotorSpeedLast = motorSpeedCurrent;
    
    if (t2 < t1-GST) 
    {
      motorSpeedCurrent = motorSpeedMax;
    }
    else if (t2 >= t1+GST) 
    {
      motorSpeedCurrent = motorSpeedMin;
    }
    else
    {
      // If turn ON the case (motor is off) and temp is inbetween GST - set speed to max
      if (motorSpeedCurrent == 0)
      {
        motorSpeedCurrent = motorSpeedMax;
      }
    }

    // Limit motor speed to be in actual speed ranges
//    motorSpeedCurrent = constrain(motorSpeedCurrent, motorSpeedMin, motorSpeedMax);

    if (motorSpeedCurrent != MotorSpeedLast)
    {
      SetMotorSpeed(motorSpeedCurrent);
    }
  }

    // Motor speed current
    lcd.setCursor(9,0);  
    lcd.print("    ");
    lcd.setCursor(9,0);
    lcd.print(motorSpeedCurrent);


  if (IsMaxTempReached)
  {
    if ((t2 >= T_max_avar) || (t2 <= T_min_avar))  
    {
      SetMotorSpeed(0);
      
      LOG << "ALL is BAD!! Emergency STOP!!" << NL();
      exit(0);
    } 
  }
}
