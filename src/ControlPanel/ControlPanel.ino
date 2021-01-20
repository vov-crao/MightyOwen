#include <StaticThreadController.h>
#include <Thread.h>
//#include <ThreadController.h>

// Start button
#define START_BUTTON_PIN 11

// Start LED
#define START_LED_PIN A2

#include <EEPROM.h> // подключаем библиотеку EEPROM для записи в ПЗУ
//#include <LiquidCrystalRus.h>

#include "tempDS18B20.h"
ds18b20 TempWater(12);
ds18b20 TempExOut(10);

// Encoder buttons on pins
#define ENCODER_A_PIN 4
#define ENCODER_B_PIN 5
#define ENCODER_C_PIN 6

unsigned long CurrentTime, LastTime;
enum eEncoderState {eNone, eLeft, eRight, eButton};
byte EncoderA, EncoderB, EncoderAPrev,counter;
bool ButtonPrev;
int N = 0; // переменная для перехода между Vmax Vmin t1

/*********************************************************************/
eEncoderState GetEncoderState() {
  // Считываем состояние энкодера
  eEncoderState Result = eNone;
  CurrentTime = millis();
  if (CurrentTime - LastTime >= 5) {
    // Считываем не чаще 1 раза в 5 мс для уменьшения ложных срабатываний
    LastTime = CurrentTime;
    if (digitalRead(ENCODER_C_PIN) == LOW ) {
      if (ButtonPrev) {
        Result = eButton; // Нажата кнопка
        ButtonPrev = 0;
        N++;
       }
    }
    else {
      ButtonPrev = 1;
      EncoderA = digitalRead(ENCODER_A_PIN);
      EncoderB = digitalRead(ENCODER_B_PIN);
      if ((!EncoderA) && (EncoderAPrev)) { // Сигнал A изменился с 1 на 0
        if (EncoderB) Result = eRight;     // B=1 => энкодер вращается по часовой
        else          Result = eLeft;      // B=0 => энкодер вращается против часовой
      }
      EncoderAPrev = EncoderA; // запомним текущее состояние
      
    }
  }
  return Result;
}

//****************************************************************************************

#include <LiquidCrystal_I2C.h> // библиотека для 4 строчного дисплея
LiquidCrystal_I2C lcd(0x27,20,4);  // Устанавливаем дисплей

const int soundPin = 9;  // шагового двигателя

const int ledPin =20 ;  // переменная с номером пина светодиода/экрана

int y=EEPROM.read(0);// скорость максимальная
int x=EEPROM.read(1); // скорость минимальная 
int t1=EEPROM.read(2); // выставляемая пользователем температура теплоносителя
int t2; // температура передаваемая с термодатчика
int tout;
int Temp;
int koof=130; // 
int motorSpeed=1;
bool Flag=false; // флаг, обозначающий, что котел еще не нагревался до рабочей температуры ни разу
bool blinkStatus = false;    // состояние курсора Вкл/Выкл
int T_max_avar =EEPROM.read(3);//85 ;  // температура отключения мотора верхняя
int T_min_avar =EEPROM.read(4);//45 ;  // температура отключения мотора нижняя
int t3=EEPROM.read(5);//50 ;  // температура включения защиты
int GST = EEPROM.read(6);//1 ;  // Гистерезис терморегулятора
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
        N++;
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
     // pinMode(soundPin, OUTPUT); // объявляем пин 3 как выход.
      pinMode(8, OUTPUT);      
      pinMode(9, OUTPUT); 
      pinMode(ledPin, OUTPUT);   // объявляем пин 13 как выход.
      
     
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

/*********************************************************************/
void loop() {
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
    
    // Проверим, пришло ли время сменить тональность сирены:
    if (soundThread.shouldRun())
        soundThread.run(); // запускаем поток

    // Проверим, пришло ли время помигать курсором:
    if (blinkThread.shouldRun())
        blinkThread.run(); // запускаем поток
    
//    switch (GetEncoderState()) {
  switch (GetEncoderStateISR()) 
  {
    case eNone: return;
    case eLeft: {   // Энкодер вращается влево
        counter--;
         if (counter<1 && N!=8) counter++; 
         if (N==6  && counter<25) counter++; //ограничиваем температуру t3 -срабатывание защиты  
         if (N==8  && counter>254) counter++; //ограничиваем температуру t3 -срабатывание защиты                                                                                                                                                                                                        ) counter++; //ограничиваем температуру t3 -срабатывание защиты  
        break;
      }
    case eRight:  {  // 1 и 2 Энкодеры вращается вправо
        
       counter++;
     
       if (N==3  && counter>95) counter--; //t1-ограничиваем рабочую температуру
       if (N==4  && counter>95) counter--; //T_max_avar -ограничиваем
       if (N==6  && counter>90) counter--; //ограничиваем температуру t3 -срабатывание защиты  
      
       if (N==7  && counter>10) counter--; // ограничим ввод гистерезиса 10*/
       if (N==8  && counter>1) counter--; // ограничим ввод пламени 1/
      if (counter>110) counter--; 
         break;
    }
       
    case eButton: { // Нажали кнопку
       
       switch (N) {
  case 1:
    //выполняется, когда N равно 1
   counter=EEPROM.read(0);
       break;
  case 2:
    //выполняется когда  N равно 2
    counter=EEPROM.read(1); 
    break;
  case 3:
    //выполняется когда  N равно 3
    counter=EEPROM.read(2); 
    break;
 
  case 4:
    //выполняется, когда M равно 1
   counter=EEPROM.read(3); 
    break;
  case 5:
    //выполняется когда  M равно 2
    counter=EEPROM.read(4); 
    break;
  case 6:
    //выполняется когда  M равно 3
    counter=EEPROM.read(5); 
    break;
  case 7:
    //выполняется когда  M равно 4
    counter=EEPROM.read(6); 
    break;
  case 8:
    //выполняется когда  M равно 8
    counter=EEPROM.read(7); 
    break;
 }
        break;
    }
  }
  Serial.println(counter);
 //Serial.print(N);
    if (N==1){
             
              y=counter;
              EEPROM.write(0, y); // запись числo y=Vmax в ячейку 0
             
    }
   if (N==2){
        x=counter;
        EEPROM.write(1, x); // запись числo x=Vmin в ячейку 1
    }
    if (N==3){
        t1=counter;
        EEPROM.write(2, t1); // запись числo t1 в ячейку 2
    }
    
   if (N==4){
       T_max_avar=counter;
       EEPROM.write(3, T_max_avar); // запись числа в ячейку 3
    }
   if (N==5){
        T_min_avar=counter;
        EEPROM.write(4, T_min_avar); // запись числа в ячейку 4
    }
    if (N==6){
        t3=counter;
        EEPROM.write(5, t3); // запись числo t3 в ячейку 5
    }
    if (N==7){
        GST=counter;
        EEPROM.write(6, GST); // запись числo GST в ячейку 6
    }
    if (N==8) 
    {
      N=0; 
    }
}

// Поток светодиода:
/*********************************************************************/
void ledBlink() { 
   
  
    static bool ledStatus = false;    // состояние светодиода Вкл/Выкл
    ledStatus = !ledStatus;           // инвертируем состояние
    digitalWrite(ledPin, ledStatus);  // включаем/выключаем светодиод

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
// Поток мигания курсором:

 // Поток сирены:
void sound() { 
   
  //  int sensorReading = analogRead(A0);//измерение значения потенциометра А0 мин.скорость
    // map it to a range from 0 to 100:
      
   // int x = value;
    //  if (M>0) {
         
      lcd.setCursor(0,2);
      lcd.print("Tmax");
      lcd.setCursor(1,3); 
      lcd.print("    "); 
      lcd.setCursor(1,3); 
      if (N==4) {
       lcd.print(T_max_avar);
       lcd.print("<");    
      }
      else {
      lcd.print(T_max_avar);
      }
                   
      lcd.setCursor(5,2);  
      lcd.print("Tmin");
      lcd.setCursor(6,3);  
      lcd.print("    ");
      lcd.setCursor(6,3); 
      if (N==5) {
       lcd.print(T_min_avar);
       lcd.print("<");    
      }
      else {
      lcd.print(T_min_avar);
      }
      lcd.setCursor(10,2);  
      lcd.print("t3");
      lcd.setCursor(10,3);  
      lcd.print("   ");
      lcd.setCursor(10,3);  
      if (N==6) {
      lcd.print(t3);
      lcd.print("<");    
      }
      else {
      lcd.print(t3);
      }
     
      lcd.setCursor(13,2);  
      lcd.print("Gst");
      lcd.setCursor(14,3); 
      lcd.print("   "); 
      lcd.setCursor(14,3); 
       if (N==7) {
       lcd.print(GST);
       lcd.print("<");    
      }
      else {
      lcd.print(GST);
      } 
    
     lcd.setCursor(0,0);  
     
     lcd.print("Vmax=    ");
     lcd.setCursor(5,0);  
     if (N==1) {
     lcd.print(y);
     lcd.print("<");    
     }
     else {
     lcd.print(y);
     }
    
     
     lcd.setCursor(0,1); 
     lcd.print("Vmin=    ");
     lcd.setCursor(5,1);  
      if (N==2) {
     lcd.print(x);
     lcd.print("<"); 
    // lcd.print("\x7E");    
     }
     else {
     lcd.print(x);
     }
     
 //  int sensorReading3 = analogRead(A5);//измерение значения потенциометра А2 задаваемая темп. теплоносителя
 //  int t1 = map(sensorReading3, 0, 1023, 1, 101);   // map it to a range from 0 to 100:
     lcd.setCursor(13,0);  
     lcd.print("t1=    ");
     lcd.setCursor(16,0);  
     if (N==3) {
     lcd.print(t1);
     lcd.print("<");    
     }
     else {
     lcd.print(t1);
     }
      if (t2>=t3) {
      Flag=true;  //температура теплоносителя нагрелась до t3 градусов .было 50 градусов
     } 
     
     if (StartButtonPressed) 
     {// если кнопка ПУСКА нажата
     if (t2<t1-GST) {  //  если температура текущая меньше температуры, установленной пользователем
        int motorSpeed = y*koof; //включается максимальная скорость ШД
        int ton = motorSpeed; 
       // tone(soundPin, ton); 
       tone(9, ton); 
         }
     
     if (t2>=t1+GST) { //если температура текущая больше или равно температуры, установленной пользователем
        int motorSpeed = x*koof; //включается минимальная скорость ШД
        int ton = motorSpeed;  
        tone(9, ton); 
      //  Flag = true; // температура теплоносителя нагрелась до установленой пользователем температуры t1   
     }
     }
}
