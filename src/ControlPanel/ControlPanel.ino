#include <StaticThreadController.h>
#include <Thread.h>

#include <LiquidCrystal_I2C.h> // библиотека для 4 строчного дисплея
LiquidCrystal_I2C lcd(0x27,20,4);  // Устанавливаем дисплей

#include <EEPROM.h>

// Start button
#define START_BUTTON_PIN 11

// Start LED
#define START_LED_PIN A2

// Blinking led
#define BLINKING_LED_PIN 13

// Temperature sensors pins
#define WATER_SENSOR_PIN 12
#define EXHOST_SENSOR_PIN 10

#include "tempDS18B20.h"
ds18b20 TempWater(WATER_SENSOR_PIN);
ds18b20 TempExOut(EXHOST_SENSOR_PIN);

// Encoder buttons on pins
#define ENCODER_A_PIN 4
#define ENCODER_B_PIN 5
#define ENCODER_C_PIN 6

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

// Stepper Motor
#define STEPPER_MOTOR_PULSE_PIN 9
#define STEPPER_MOTOR_DIR_PIN   8

//
//****************************************************************************************

int motorSpeedMax = EEPROM.read(STORE_FUEL_SPEED_MAX);// скорость максимальная
int motorSpeedMin = EEPROM.read(STORE_FUEL_SPEED_MIN); // скорость минимальная 
int t1 = EEPROM.read(STORE_TARGET_TEMP); // выставляемая пользователем температура теплоносителя
int T_max_avar = EEPROM.read(STORE_MOTOR_MAX);  // температура отключения мотора верхняя
int T_min_avar = EEPROM.read(STORE_MOTOR_MIN);  // температура отключения мотора нижняя
int t3 = EEPROM.read(STORE_TEMP_MAX);           // температура включения защиты
int GST = EEPROM.read(STORE_TEMP_GIST);         // Гистерезис терморегулятора

int t2; // температура передаваемая с термодатчика
int tout;
const int SteppingMotorHz = 6400 / 5 / 10; // 6400 pulses to make full turn for 5 sec using speed 10
bool IsMaxTempReached = false;
bool StartButtonPressed = false; //кнопка ПУСК

Thread ledThread = Thread(); // создаём поток управления светодиодом
Thread soundThread = Thread(); // создаём поток управления 
Thread blinkThread = Thread(); // создаём поток мигания курсором

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

/*********************************************************************/
void setup() 
{
  // Start LED
  pinMode(START_LED_PIN, OUTPUT);
  if (!StartButtonPressed) 
    digitalWrite(START_LED_PIN, HIGH);

    // Start button.
    pinMode(START_BUTTON_PIN, INPUT);
    digitalWrite(START_BUTTON_PIN, HIGH); // use pull up resistor
   
  t2 = TempWater.getTemp();
  tout = TempExOut.getTemp();

  Wire.begin();//Инициализирует библиотеку Wire и подключается к шине I2C как ведущий (мастер) или ведомый.
  Wire.beginTransmission(0x27);
  
  lcd.begin(20, 4); // initialize the lcd
  
  Serial.begin(9600);
  
  pinMode(STEPPER_MOTOR_DIR_PIN, OUTPUT);      
  pinMode(STEPPER_MOTOR_PULSE_PIN, OUTPUT); 
  
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
    PCMSK2 = (1<<PCINT20) | (1<<PCINT21); // Encoder left/right buttons
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
      Serial.println("Start button pressed"); 
    }
    
    if (ledThread.shouldRun())
        ledThread.run(); // запускаем поток
    
    if (soundThread.shouldRun())
        soundThread.run(); // запускаем поток

    if (blinkThread.shouldRun())
        blinkThread.run(); // запускаем поток
    
  switch (GetEncoderStateISR()) 
  {
    case eNone: return;
    
    case eLeft: 
      {
        StoreCurrentValue--;
        Serial.print("Left turn:"); 
        Serial.println(StoreCurrentValue, 10); 

        CheckLimitStoreVar();
        break;
      }
      
    case eRight:  
      {
        StoreCurrentValue++;

        Serial.print("Right turn:"); 
        Serial.println(StoreCurrentValue, 10); 

        CheckLimitStoreVar();
        break;
      }

    // Pressed Encoder button
    case eButton: 
      {
        Serial.print("Knob pressed. Old index:"); 
        Serial.print(VarIndex, 10); 
        
        // Store prev updated value
        if (VarIndex < STORE_INDEX_MAX)
        {
          const byte OldValue = EEPROM.read(VarIndex);
          Serial.print(" Old value:"); 
          Serial.print(OldValue, 10); 
          if (StoreCurrentValue != OldValue)
          {
            Serial.print(" Write new value:"); 
            Serial.print(StoreCurrentValue, 10); 
            
            EEPROM.write(VarIndex, StoreCurrentValue);
          }
        }

        Serial.println();
        
        ++VarIndex;

        if (VarIndex > STORE_INDEX_MAX)
          VarIndex = 0;
        
        Serial.print(" New index:"); 
        Serial.println(VarIndex, 10); 

        if (VarIndex < STORE_INDEX_MAX)
        {
          Serial.print(" ee value:"); 
          StoreCurrentValue = EEPROM.read(VarIndex);
          Serial.println(StoreCurrentValue, 10); 
        }
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

    t2 = TempWater.getTemp();
    tout = TempExOut.getTemp();

    lcd.setBacklight(255);

    lcd.setCursor(13,1);  
    lcd.print("t2=     ");
    lcd.setCursor(16,1);  
    lcd.print(t2);

    lcd.setCursor(17,2);  
    lcd.print("to");
    lcd.setCursor(17,3);  
    lcd.print("    ");
    lcd.setCursor(17,3);  
    lcd.print(tout);
}

/*********************************************************************/
void PrintMarker(const byte Index)
{
  if (VarIndex == Index) 
  {
    if (StoreCurrentValue == EEPROM.read(VarIndex))
      lcd.print("<");    
    else
      lcd.print("*");
  }
}

 // Поток сирены:
/*********************************************************************/
void sound() 
{ 
  lcd.setCursor(0,0);  
  lcd.print("Vmax=    ");
  lcd.setCursor(5,0);  
  lcd.print(motorSpeedMax);
  PrintMarker(STORE_FUEL_SPEED_MAX);
     
  lcd.setCursor(0,1); 
  lcd.print("Vmin=    ");
  lcd.setCursor(5,1);  
  lcd.print(motorSpeedMin);
  PrintMarker(STORE_FUEL_SPEED_MIN);
     
  lcd.setCursor(13,0);  
  lcd.print("t1=    ");
  lcd.setCursor(16,0);  
  lcd.print(t1);
  PrintMarker(STORE_TARGET_TEMP);

  lcd.setCursor(0,2);
  lcd.print("Tmax");
  lcd.setCursor(1,3); 
  lcd.print("    "); 
  lcd.setCursor(1,3); 
  lcd.print(T_max_avar);
  PrintMarker(STORE_MOTOR_MAX);
                   
  lcd.setCursor(5,2);  
  lcd.print("Tmin");
  lcd.setCursor(6,3);  
  lcd.print("    ");
  lcd.setCursor(6,3); 
  lcd.print(T_min_avar);
  PrintMarker(STORE_MOTOR_MIN);
      
  lcd.setCursor(10,2);  
  lcd.print("t3");
  lcd.setCursor(10,3);  
  lcd.print("   ");
  lcd.setCursor(10,3);  
  lcd.print(t3);
  PrintMarker(STORE_TEMP_MAX);
     
  lcd.setCursor(13,2);  
  lcd.print("Gst");
  lcd.setCursor(14,3); 
  lcd.print("   "); 
  lcd.setCursor(14,3); 
  lcd.print(GST);
  PrintMarker(STORE_TEMP_GIST);
  
  if (t2 >= t3) 
  {
    IsMaxTempReached = true;
    Serial.println("MAX temp!");
  } 
     
  if (StartButtonPressed) 
  {
    if (t2<t1-GST) 
    {
      const int motorSpeed = motorSpeedMax * SteppingMotorHz; //включается максимальная скорость ШД
      tone(STEPPER_MOTOR_PULSE_PIN, motorSpeed); 
    }
     
    if (t2>=t1+GST) 
    {
      const int motorSpeed = motorSpeedMin * SteppingMotorHz; //включается минимальная скорость ШД
      tone(STEPPER_MOTOR_PULSE_PIN, motorSpeed); 
    }
  }

  if (IsMaxTempReached)
  {
    if ((t2 >= T_max_avar) || (t2 <= T_min_avar))  
    {
      tone(STEPPER_MOTOR_PULSE_PIN, 0); 
      Serial.println("ALL is BAD!! Emergency EXIT!!");
      exit(0);
    } 
  }
}
