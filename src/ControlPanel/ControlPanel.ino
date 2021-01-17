#include <StaticThreadController.h>
#include <Thread.h>
//#include <ThreadController.h>

#include "GyverButton.h" // библиотека подключения кнопки 1811
#define PIN 14        //кнопка подключена сюда (PIN --- КНОПКА --- GND)1811
GButton butt1(PIN); //1811

#include <EEPROM.h> // подключаем библиотеку EEPROM для записи в ПЗУ
//#include <LiquidCrystalRus.h>

#include <OneWire.h>
OneWire  ds(12);  //

#define pin_CLK 2
#define pin_DT  3
#define pin_Btn 4

int flame_sensor = 7; // датчик пламени
int flame_detected;   // датчик пламени

unsigned long CurrentTime, LastTime;
enum eEncoderState {eNone, eLeft, eRight, eButton};
byte EncoderA, EncoderB, EncoderAPrev,counter;
bool ButtonPrev;
int N = 0; // переменная для перехода между Vmax Vmin t1

eEncoderState GetEncoderState() {
  // Считываем состояние энкодера
  eEncoderState Result = eNone;
  CurrentTime = millis();
  if (CurrentTime - LastTime >= 5) {
    // Считываем не чаще 1 раза в 5 мс для уменьшения ложных срабатываний
    LastTime = CurrentTime;
    if (digitalRead(4) == LOW ) {  //  if (digitalRead(pin_Btn) == LOW ) 
      if (ButtonPrev) {
        Result = eButton; // Нажата кнопка
        ButtonPrev = 0;
        N++;
       }
    }
    else {
      ButtonPrev = 1;
      EncoderA = digitalRead(2);// EncoderA = digitalRead(pin_DT)
      EncoderB = digitalRead(3);//EncoderB = digitalRead(pin_CLK);
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
int Temp;
int koof=130; // 
int motorSpeed=1;
bool Flag=false; // флаг, обозначающий, что котел еще не нагревался до рабочей температуры ни разу
bool blinkStatus = false;    // состояние курсора Вкл/Выкл
int T_max_avar =EEPROM.read(3);//85 ;  // температура отключения мотора верхняя
int T_min_avar =EEPROM.read(4);//45 ;  // температура отключения мотора нижняя
int t3=EEPROM.read(5);//50 ;  // температура включения защиты
int GST = EEPROM.read(6);//1 ;  // Гистерезис терморегулятора
byte Dpl = EEPROM.read(7);  // 0-датчик пламени выключен/1-датчик пламени включен
bool Flag_knopka=false; //кнопка ПУСК

Thread ledThread = Thread(); // создаём поток управления светодиодом
Thread soundThread = Thread(); // создаём поток управления 
Thread blinkThread = Thread(); // создаём поток мигания курсором

void setup() {
   pinMode(A2, OUTPUT);//1811
    if (Flag_knopka==false) digitalWrite(A2, HIGH);//1811
  
   pinMode(flame_sensor, INPUT);//инициализация датчик поамени
// Создаем новый символ.
   byte data[2];           // объявляем массив из 2-х байт
  ds.reset();             // инициализируем датчик
  ds.write(0xCC);         // пропускаем адресацию к конкретному датчику (у нас он один)
  ds.write(0x44);         // даем команду измерять температуру
  delay(500);           // ждем 1 секунду, пока измеряется температура
 
  ds.reset();            // снова инициализируем датчик
  ds.write(0xCC);        // снова пропускаем адресацию
  ds.write(0xBE);         // даем команду готовности считывать температуру
  data[0] = ds.read();    //считываем  младший
  data[1] = ds.read();    // и старший байты
  int Temp = (data[1] << 8) + data[0];  // преобразуем считанную информацию
  Temp = Temp >> 4;                     // к нужному виду.
  Serial.println(Temp);                 // выводим результат в последовательный порт.
     t2 = Temp+1;//прибавляем к температуре датчика +1
  
  pinMode(2,  INPUT);
  pinMode(3, INPUT);
  pinMode(4, INPUT_PULLUP); // Кнопка не подтянута к +5 поэтому задействуем внутренний pull-up резистор

  Serial.begin(9600);
     
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

   
}

void loop() {
// Проверим, пришло ли время переключиться светодиоду:
    butt1.tick();  // обязательная функция отработки. Должна постоянно опрашиваться //1811
    if (butt1.isSingle()) {Flag_knopka=true;  digitalWrite(A2, LOW); Serial.println("Single"); } //1811
    
    if (ledThread.shouldRun())
        ledThread.run(); // запускаем поток
    
    // Проверим, пришло ли время сменить тональность сирены:
    if (soundThread.shouldRun())
        soundThread.run(); // запускаем поток

    // Проверим, пришло ли время помигать курсором:
    if (blinkThread.shouldRun())
        blinkThread.run(); // запускаем поток
    
    switch (GetEncoderState()) {
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
     if (N==8){
        Dpl=counter;
        EEPROM.write(7, Dpl); // запись числo GST в ячейку 7
    }
     if (N==9) N=0; 
   //*********************************************************
   
   //*******************************************************
}

// Поток светодиода:
void ledBlink() { 
   
  
    static bool ledStatus = false;    // состояние светодиода Вкл/Выкл
    ledStatus = !ledStatus;           // инвертируем состояние
    digitalWrite(ledPin, ledStatus);  // включаем/выключаем светодиод
  
    byte data[2];           // объявляем массив из 2-х байт
  ds.reset();             // инициализируем датчик
  ds.write(0xCC);         // пропускаем адресацию к конкретному датчику (у нас он один)
  ds.write(0x44);         // даем команду измерять температуру
  //delay(500);           // ждем 1 секунду, пока измеряется температура
 
  ds.reset();            // снова инициализируем датчик
  ds.write(0xCC);        // снова пропускаем адресацию
  ds.write(0xBE);         // даем команду готовности считывать температуру
  data[0] = ds.read();    //считываем  младший
  data[1] = ds.read();    // и старший байты
  int Temp = (data[1] << 8) + data[0];  // преобразуем считанную информацию
  Temp = Temp >> 4;                     // к нужному виду.
  Serial.println(Temp);                 // выводим результат в последовательный порт.
     t2 = Temp+1;//прибавляем к температуре датчика +1
   
  // выводим результат в последовательный порт.
 //    Serial.print("t2=");
  //   Serial.println(t2);
      
    lcd.setBacklight(255);
    lcd.setCursor(13,1);  
    lcd.print("t2=     ");
    lcd.setCursor(16,1);  
    lcd.print(t2);
   
    flame_detected = digitalRead(flame_sensor);
    if (flame_detected == 1) // нет огня
  {
    Serial.println("Нет пламени....");
    //digitalWrite(buzzer, HIGH);
    //digitalWrite(LED, HIGH);
//    delay(200);
    //digitalWrite(LED, LOW);
  //  delay(200);
  }
  else
  {
    Serial.println("Вижу пламя!!!");
    //digitalWrite(buzzer, LOW);
    //digitalWrite(LED, LOW);
  }
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
      lcd.setCursor(17,2);  
      lcd.print("Dpl");
      lcd.setCursor(18,3); 
      lcd.print("   "); 
      lcd.setCursor(18,3); 
       if (N==8) {
       lcd.print(Dpl);
       lcd.print("<");    
      }
      else {
      lcd.print(Dpl);
      } 
   //   }
    
  //  int sensorReading2 = analogRead(A3);//измерение значения потенциометра А1 мак.скорость
 //    int y = value;
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
     if (Flag_knopka==true) {// если кнопка ПУСКА нажата
     if (t2<t1-GST) {  //  если температура текущая меньше температуры, установленной пользователем
        int motorSpeed = y*koof; //включается максимальная скорость ШД
        int ton = motorSpeed; 
       // tone(soundPin, ton); 
       tone(9, ton); 
         }
     }
      if (Flag_knopka==true) {// если кнопка ПУСКА нажата
     if (t2>=t1+GST) { //если температура текущая больше или равно температуры, установленной пользователем
        int motorSpeed = x*koof; //включается минимальная скорость ШД
        int ton = motorSpeed;  
        tone(9, ton); 
      //  Flag = true; // температура теплоносителя нагрелась до установленой пользователем температуры t1   
     }
     }
     if (t2>0 && t2<101) {
     if ((t2>=T_max_avar && Flag==true) || (t2<=T_min_avar && Flag==true )|| (flame_detected == 1 && Flag==true && Dpl==true))  {
     exit(0);
     Serial.println(t2);
     } 
     }
          
      
     if ((t2<1) || (t2>101)) { 
      asm("JMP 0");  
     }
   // if (ton <= 500) {  // до частоты 500 Гц 
   //     ton += 100;  // увеличиваем тональность сирены
   // }
   // else {  // по достижении 500 Гц
     //   ton = 100;  // сбрасываем тональность до 100 Гц
   // }
}
