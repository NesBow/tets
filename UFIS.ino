/*
██╗░░░██╗░█████╗░░██████╗░███████╗██╗░░░██╗██╗░░░░░██╗░░░░░███████╗██╗░██████╗░█████╗░██████╗░██████╗░██╗░░░██╗██╗███╗░░██╗░█████╗░
██║░░░██║██╔══██╗██╔════╝░██╔════╝██║░░░██║██║░░░░░██║░░░░░██╔════╝██║██╔════╝██╔══██╗██╔══██╗██╔══██╗██║░░░██║██║████╗░██║██╔══██╗
╚██╗░██╔╝███████║██║░░██╗░█████╗░░██║░░░██║██║░░░░░██║░░░░░█████╗░░██║╚█████╗░███████║██████╔╝██║░░██║██║░░░██║██║██╔██╗██║██║░░██║
░╚████╔╝░██╔══██║██║░░╚██╗██╔══╝░░██║░░░██║██║░░░░░██║░░░░░██╔══╝░░██║░╚═══██╗██╔══██║██╔══██╗██║░░██║██║░░░██║██║██║╚████║██║░░██║
░░╚██╔╝░░██║░░██║╚██████╔╝██║░░░░░╚██████╔╝███████╗███████╗██║░░░░░██║██████╔╝██║░░██║██║░░██║██████╔╝╚██████╔╝██║██║░╚███║╚█████╔╝
░░░╚═╝░░░╚═╝░░╚═╝░╚═════╝░╚═╝░░░░░░╚═════╝░╚══════╝╚══════╝╚═╝░░░░░╚═╝╚═════╝░╚═╝░░╚═╝╚═╝░░╚═╝╚═════╝░░╚═════╝░╚═╝╚═╝░░╚══╝░╚════╝░
------------------------------------------------
AUTHOR:
@BILL1389 https://github.com/bill1389
@Tomáš Kováčik https://github.com/tomaskovacik
@Arildlangseid 
@Domnulvlad https://github.com/domnulvlad
@Alexander Grau https://grauonline.de/wordpress/?author=3
OBD2 and especially kw1281 https://www.blafusel.de/obd/obd.html 
------------------------------------------------
*/

/* РАБОТА КНОПКИ:
-ОДИН КЛИК-ДАЛЬШЕ
-ЗАЖАТЬ В РАЗДЕЛЕ С ОШИБКАМИ, СТЕРЕТЬ ОШИБКИ
-ЗАЖАТЬ В ЛЮБОМ ОКНЕ НЕ С ОШИБКАМИ ВТОРОЙ ЕКРАН ИЛИ БЛОК С ЕБУ ГРУППАМИ
КНОПКА ИНОГДА РАБОТАЕТ НЕ КОРЕКТНО*/

#include <avr/eeprom.h>
#include "NewSoftwareSerial.h"
#include <SPI.h>
#include "bitmaps.h"
#include "EncButton.h"
#include "VAGFISWriter.h"

#include <avr/sleep.h>
#include <avr/power.h>

// Пины для подключения
#define pinButton 2
#define pinKLineTX 3
#define pinKLineRX 4

EncButton<EB_TICK, pinButton> btnUp(INPUT);
//#define DEBUG;
//#define LOGS;
// avr
#define FIS_ENA 5
#define FIS_CLK 6
#define FIS_DATA 7

// CLK,DATA,ENA pin, forcemode=0/1
VAGFISWriter fisWriter(FIS_CLK, FIS_DATA, FIS_ENA, 1);

#define ADR_Engine 0x01
#define ADR_Gears 0x02
#define ADR_ABS_Brakes 0x03
#define ADR_Airbag 0x15
#define ADR_Dashboard 0x17
#define ADR_Immobilizer 0x25
#define ADR_Central_locking 0x35

int ADR_Engine_Speed = 10400; // Скорость обмена данных для двигателя (бодрейт)
int ADR_Dashboard_Speed = 10400; // Скорость обмена данных для приборной панели (бодрейт)
int ErrorArray[10]; // Массив для хранения ошибок
NewSoftwareSerial obd(pinKLineRX, pinKLineTX, false); // Инициализация программного UART для OBD (RX, TX, инверсная логика)
bool readErrorflag = 0; // Флаг для индикации ошибки чтения
int errorErrorCount = 1;  // Счетчик ошибок


int currentMenu = 0;
bool inSubMenu = false;
int currentMenuSelection = 0; // Переменная для отслеживания текущего пункта меню


uint8_t currAddr = 0; // Текущий адрес устройства
uint8_t blockCounter = 0;  // Счетчик блоков данных
uint8_t errorTimeout = 0; // Таймаут ошибки
uint8_t errorData = 0; // Ошибочные данные
bool connected = false;  // Флаг соединения
int sensorCounter = 0;  // Счетчик сенсоров
int pageUpdateCounter = 0;  // Счетчик обновлений страницы
int alarmCounter = 0;  // Счетчик тревог
bool fullDash = true; // Основной экран должен запускаться первым
bool fullDash = false;  // Флаг для полного отображения приборной панели
uint8_t currPage = 1;  // Текущая страница
uint8_t currPageOld;  // Предыдущая страница

int readVagGroup = 0;  // Номер группы VAG для чтения

int param0;  // Параметры для чтения данных
int param1;
int param2;
int param3;
String mess;
String turboZapros;  // Строки для сообщений и данных о турбонаддуве
String turboPress;
float turboPressMax; // Максимальное давление турбины
int turboPressCount = 1;  // Счетчик турбонадува

int8_t ambientTemperature = 0;  // Температура окружающей среды 
int8_t coolantTemp = 0;  // Температура охлаждающей жидкости,
int8_t oilTemp = 0;  // Температура масла, впускного воздуха
int8_t intakeAirTemp = 0;  // Температура впускного воздуха
int8_t oilPressure = 0;  // Давление масла
float engineLoad = 0;  // Нагрузка на двигатель
int engineSpeed = 0;  // Скорость двигателя (обороты в минуту)
float throttleValve = 0;  // Положение дроссельной заслонки
float supplyVoltage = 0;  // Напряжение бортовой сети
uint8_t vehicleSpeed = 0;  // Скорость автомобиля
uint8_t fuelConsumption = 0; //Расход топлива
uint8_t fuelLevel = 0; //Уровень топлива
unsigned long odometer = 0; // Пробег

// Время впрыска, массовый расход воздуха, расход топлива (текущий и средний)
float injektTime = 0;
float MAF = 0;
float Lhour = 0;
float LhourAVGtmp = 0;
float LhourAVG = 0;
float L100Current = 0;
float L100;

float L100tmp; // Временная переменная для расчета расхода топлива
int ix = 1;  // Индексы для циклов
int countLoop = 1;// Счетчик циклов

int iy = 1;
float L100Move = 0;
float L100Movetmp = 0;

float vehicleSpeedAVGtmp;  // Средняя скорость
float vehicleSpeedAVG;  // Средняя скорость
float L100AVG;  // Средний расход топлива
//  float L100tmp =0;
//  int ix = 1;
bool SaveL100Flag = false;  // Флаг сохранения расхода топлива

bool welcomeScreenShown = false;  // Флаг, отображался ли экран приветствия

String floatToString(float v)
{
  String res;
  char buf[16];
  dtostrf(v, 4, 2, buf);
  res = String(buf);
  return res;
}

// Функция для отключения от OBD
void disconnect(){
  connected = false;  // Устанавливаем флаг соединения в false
  currAddr = 0;  // Сбрасываем текущий адрес
}

// Функция для записи данных в OBD
void obdWrite(uint8_t data){
#ifdef DEBUG
// Отладочный вывод в Serial (если включен DEBUG режим)
//  Serial.print("uC:");
//  Serial.println(data, HEX);
#endif
  obd.write(data);  // Отправляем данные через OBD
}

// Функция для чтения данных с OBD
uint8_t obdRead(){
  unsigned long timeout = millis() + 2000;    // Устанавливаем таймаут в 2 секунду
  while (!obd.available()){  // Ждем, пока данные не станут доступны
    if (millis() >= timeout) {
		// Если вышло время ожидания, обрабатываем ошибку
		// Serial.println(F("ERROR: obdRead timeout"));
      disconnect();      
      errorTimeout++;  // Увеличиваем счетчик таймаутов
      return 0;
    }
  }
  uint8_t data = obd.read();  // Читаем данные из OBD
#ifdef DEBUG  
//  Serial.print("ECU:");
//  Serial.println(data, HEX);
#endif  
  return data;  // Возвращаем считанные данные
}



// Отправка данных с использованием 5 бод для инициализации соединения, 7O1
void send5baud(uint8_t data){
  // // 1 start bit, 7 data bits, 1 parity, 1 stop bit
  #define bitcount 10
  byte bits[bitcount];
  byte even=1;
  byte bit;
  
// Формируем биты для передачи (1 стартовый бит, 7 информационных, 1 бит четности, 1 стоп-бит)
  for (int i=0; i < bitcount; i++){
    bit=0;
    if (i == 0)  bit = 0;  // Стартовый бит
      else if (i == 8) bit = even;  // Бит четности
      else if (i == 9) bit = 1;  // Стоп-бит
      else {
        bit = (byte) ((data & (1 << (i-1))) != 0);
        even = even ^ bit;  // Подсчет четности
      }
 //   Serial.print(F("bit"));      
 //   Serial.print(i);          
 //   Serial.print(F("="));              
 //   Serial.print(bit);
 //  if (i == 0) Serial.print(F(" startbit"));
 //     else if (i == 8) Serial.print(F(" parity"));    
 //    else if (i == 9) Serial.print(F(" stopbit"));              
 //   Serial.println();      
    bits[i]=bit;
  }
  
 // Отправляем битстрим с задержкой для реализации скорости 5 бод  
  for (int i=0; i < bitcount+1; i++){
    if (i != 0){
        // Ожидаем 200 мс между битами (=5 baud), adjusted by latency correction
      delay(200);
      if (i == bitcount) break;
    }
    if (bits[i] == 1){ 
      // high
      digitalWrite(pinKLineTX, HIGH);
    } else {
      // low
      digitalWrite(pinKLineTX, LOW);
    }
  }
  obd.flush();  // Очистка буфера
}

// Инициализация соединения по протоколу KWP с использованием 5 бод
bool KWP5BaudInit(uint8_t addr){
  Serial.println(F("---KWP 5 baud init"));
  //delay(3000);
 // digitalWrite(pinKLineTX, LOW);
  //delay(2400);
  send5baud(addr);  // Отправляем адрес устройства
  return true;
}

// Функция для отправки блока данных через протокол KWP
bool KWPSendBlock(char *s, int size){
// Вывод информации о размере блока и счетчике блоков
  Serial.print(F("---KWPSend sz="));
  Serial.print(size);
  Serial.print(F(" blockCounter="));
  Serial.println(blockCounter);    

// Вывод данных, которые будут отправлены
  Serial.print(F("OUT:"));
  for (int i=0; i < size; i++){    
    uint8_t data = s[i];
    Serial.print(data, HEX);
    Serial.print(" ");    
  }  
  Serial.println();
// Отправка каждого байта данных  
  for (int i=0; i < size; i++){
    uint8_t data = s[i];    
    obdWrite(data); // Запись данных в шину OBD
    /*uint8_t echo = obdRead();  
    if (data != echo){
      Serial.println(F("ERROR: invalid echo"));
      disconnect();
      errorData++;
      return false;
    }*/
	
    // Если это не последний байт, проверяем комплементарный байт	
    if (i < size-1){
      uint8_t complement = obdRead();  // Чтение байта из шины OBD       
      if (complement != (data ^ 0xFF)){ // Проверка, что байт является комплементарным
        Serial.println(F("ERROR: invalid complement"));
        disconnect();
        errorData++;
        return false;
      }
    }
  }
  blockCounter++; // Увеличиваем счетчик блоков
  return true; // Возвращаем успех
}

// Функция для получения блока данных через протокол KWP
// Если передан размер 0, первый байт данных содержит длину блока
// Поддержка скоростей 4800, 9600 и 10400 бод, 8N1
bool KWPReceiveBlock(char s[], int maxsize, int &size){  
  bool ackeachbyte = false;
  uint8_t data = 0;
  int recvcount = 0;
  
  // Если размер передан 0, подтверждение каждого байта включено
  if (size == 0) ackeachbyte = true;
  
  // Вывод информации о размере блока и счетчике блоков
  Serial.print(F("---KWPReceive sz="));
  Serial.print(size);
  Serial.print(F(" blockCounter="));
  Serial.println(blockCounter);
  
  // Проверка, что размер блока не превышает допустимый максимум
  if (size > maxsize) {
    Serial.println("ERROR: invalid maxsize");
    return false;
  }  
  
  // Устанавливаем таймаут в 2000 мс
  unsigned long timeout = millis() + 2000;  
  while ((recvcount == 0) || (recvcount != size)) {
    while (obd.available()){      
      data = obdRead(); // Чтение байта данных
      delay(5); // Задержка для стабильности

           s[recvcount] = data; // Сохранение принятого байта в массив 
      recvcount++; // Увеличение счетчика принятых байтов 
          
       // Если размер блока изначально не был известен, устанавливаем его   
      if ((size == 0) && (recvcount == 1)) {
        size = data + 1; // Первый байт содержит длину блока
        if (size > maxsize) {
          Serial.println("ERROR: invalid maxsize");
          return false;
        }  
      }
	  
	  // Проверка счетчика блоков после получения второго байта
      if ((ackeachbyte) && (recvcount == 2)) {
        if (data != blockCounter){
          Serial.println(F("ERROR: invalid blockCounter"));
          disconnect();
          errorData++;
          return false;
        }
      }
	  
	  // Отправка подтверждения комплементарного байта
      if ( ((!ackeachbyte) && (recvcount == size)) ||  ((ackeachbyte) && (recvcount < size)) ){
        obdWrite(data ^ 0xFF); // Отправляем комплементарный байт-ACK        
        /*uint8_t echo = obdRead();        
        if (echo != (data ^ 0xFF)){
          Serial.print(F("ERROR: invalid echo "));
          Serial.println(echo, HEX);
          disconnect();
          errorData++;
          return false;
        }*/
      }
      timeout = millis() + 2000;  // Сброс таймаута после успешного чтения байта        
    } 
	
	// Если время ожидания истекло, сообщаем об ошибке
    if (millis() >= timeout){
      Serial.println(F("ERROR: timeout"));
      disconnect();
      errorTimeout++;
      return false;
    }
  }

 // Вывод принятого блока данных
  Serial.print(F("IN: sz="));  
  Serial.print(size);  
  Serial.print(F(" data="));  
  
  // Цикл для вывода всех принятых данных в шестнадцатеричном формате
  for (int i=0; i < size; i++){
    uint8_t data = s[i];
    Serial.print(data, HEX);
    Serial.print(F(" "));    
  }  
  Serial.println(); // Переход на новую строку
  blockCounter++; // Увеличиваем счетчик блоков
  return true; // Возвращаем успех
}

// Функция для отправки подтверждения блока
bool KWPSendAckBlock(){
  // Выводим информацию о текущем счетчике блоков
  Serial.print(F("---KWPSendAckBlock blockCounter="));
  Serial.println(blockCounter);  
  char buf[32];  
  // Формируем строку подтверждения с текущим значением счетчика блока
  sprintf(buf, "\x03%c\x09\x03", blockCounter);  
  return (KWPSendBlock(buf, 4));// Отправляем подтверждение и возвращаем результат
}
 
 // Функция для чтения блоков соединения
bool readConnectBlocks(){  
   // Выводим информацию о процессе чтения блоков соединения
  Serial.println(F("------readconnectblocks"));
//  lcdPrint(0,0, F("KW1281 label"), 20); // Oтображение на экране (например, LCD)
  String info;  
  while (true){
    int size = 0; // Инициализация размера блока
    char s[64]; // Буфер для получаемых данных
	// Получаем блок данных, если не удалось, возвращаем ложь
    if (!(KWPReceiveBlock(s, 64, size))) return false;
	
	// Если размер равен 0, возвращаем ложь
    if (size == 0) return false;
	
	// Проверяем, достигнут ли конец данных
    if (s[2] == '\x09') break; 
	
	// Проверяем, соответствует ли ответ ожидаемому значению
    if (s[2] != '\xF6') {
      Serial.println(F("ERROR: unexpected answer")); // Выводим сообщение об ошибке
      disconnect();
      errorData++;
      return false;
    }
	
	// Получаем текст из принятого блока данных
    String text = String(s);
    info += text.substring(3, size-2); // Добавляем текст в общую информацию
    
	// Отправляем подтверждение блока, если не удалось, возвращаем ложь
	if (!KWPSendAckBlock()) return false;
  }
  
  // Выводим собранную информацию
  Serial.print("label=");
  Serial.println(info);
  //lcd.setCursor(0, 1); //перемещение курсора на LCD
  //lcd.print(info); //отображение информации на LCD       
  return true; // Возвращаем истину, сигнализируя о успешном завершении
}

// Функция для установки соединения
bool connect(uint8_t addr, int baudrate){  
  Serial.print(F("------connect addr=")); // Выводим адрес соединения
  Serial.print(addr);
  Serial.print(F(" baud="));  
  Serial.println(baudrate);  
 
  blockCounter = 0;  // Сброс счетчика блоков
  currAddr = 0; // Сброс текущего адреса
  obd.begin(baudrate);  // Инициализация OBD с заданной скоростью передачи      
  KWP5BaudInit(addr); // Инициализация KWP с указанным адресом
  // Ожидаем ответ: 0x55, 0x01, 0x8A          
  char s[3];

  int size = 3; // Размер ожидаемого ответа
  // Получаем блок ответа, если не удалось, возвращаем ложь
  if (!KWPReceiveBlock(s, 3, size)) return false;
  
  // Проверяем, соответствует ли полученный ответ ожидаемому значению
  if (    (((uint8_t)s[0]) != 0x55) 
     ||   (((uint8_t)s[1]) != 0x01) 
     ||   (((uint8_t)s[2]) != 0x8A)   ){
    Serial.println(F("ERROR: invalid magic")); // Выводим сообщение об ошибке
    disconnect(); // Отключаем соединение
    errorData++; // Увеличиваем счетчик ошибок данных
    return false; // Возвращаем ложь
  }
  currAddr = addr; // Устанавливаем текущий адрес
  connected = true;  // Устанавливаем статус подключения
  if (!readConnectBlocks()) return false;
  return true;
}

// Функция для чтения датчиков  
bool readSensors(int group){
  Serial.print(F("------readSensors ")); // Выводим информацию о чтении датчиков
  Serial.println(group);
//  lcdPrint(0,0, F("KW1281 sensor"), 20); // Отображение на экране (например, LCD)    
  char s[64];
  sprintf(s, "\x04%c\x29%c\x03", blockCounter, group); // Формируем строку запроса на чтение датчиков
  if (!KWPSendBlock(s, 5)) return false;
  int size = 0; // Инициализация размера блока
  KWPReceiveBlock(s, 64, size);
  
   // Проверяем, соответствует ли ответ ожидаемому значению
  if (s[2] != '\xe7') {
    Serial.println(F("ERROR: invalid answer"));
    disconnect();
    errorData++;
    return false;
  }
  
  int count = (size-4) / 3;// Вычисляем количество датчиков
  Serial.print(F("count="));
  Serial.println(count);
  
   // Цикл для обработки каждого датчика
  for (int idx=0; idx < count; idx++){
    byte k=s[3 + idx*3]; // Получаем тип датчика
    byte a=s[3 + idx*3+1]; // Получаем значение а
    byte b=s[3 + idx*3+2]; // Получаем значение b
    String n; // Переменная для хранения имени датчика
    float v = 0; // Переменная для хранения значения
    Serial.print(F("type="));
    Serial.print(k); // Выводим тип датчика
    Serial.print(F("  a="));
    Serial.print(a); // Выводим значение а
    Serial.print(F("  b="));
    Serial.print(b); // Выводим значение b
    Serial.print(F("  text="));
    String t = ""; // Переменная для хранения текста
    String units = ""; // Переменная для хранения единиц измерения
    char buf[32]; // Буфер для форматирования строки    
    switch (k){
      case 1:  v=0.2*a*b;             units=F("rpm"); break;
      case 2:  v=a*0.002*b;           units=F("%%"); break;
      case 3:  v=0.002*a*b;           units=F("Deg"); break;
      case 4:  v=abs(b-127)*0.01*a;   units=F("ATDC"); break;
      case 5:  v=a*(b-100)*0.1;       units=F("°C");break;
      case 6:  v=0.001*a*b;           units=F("V");break;
      case 7:  v=0.01*a*b;            units=F("km/h");break;
      case 8:  v=0.1*a*b;             units=F(" ");break;
      case 9:  v=(b-127)*0.02*a;      units=F("Deg");break;
      case 10: if (b == 0) t=F("COLD"); else t=F("WARM");break;
      case 11: v=0.0001*a*(b-128)+1;  units = F(" ");break;
      case 12: v=0.001*a*b;           units =F("Ohm");break;
      case 13: v=(b-127)*0.001*a;     units =F("mm");break;
      case 14: v=0.005*a*b;           units=F("bar");break;
      case 15: v=0.01*a*b;            units=F("ms");break;
      case 18: v=0.04*a*b/1000;            units=F("mbar");break;
      case 19: v=a*b*0.01;            units=F("l");break;
      case 20: v=a*(b-128)/128;       units=F("%%");break;
      case 21: v=0.001*a*b;           units=F("V");break;
      case 22: v=0.001*a*b;           units=F("ms");break;
      case 23: v=b/256*a;             units=F("%%");break;
      case 24: v=0.001*a*b;           units=F("A");break;
      case 25: v=(b*1.421)+(a/182);   units=F("g/s");break;
      case 26: v=float(b-a);          units=F("C");break;
      case 27: v=abs(b-128)*0.01*a;   units=F("°");break;
      case 28: v=float(b-a);          units=F(" ");break;
      case 30: v=b/12*a;              units=F("Deg k/w");break;
      case 31: v=b/2560*a;            units=F("°C");break;
      case 33: v=100*b/a;             units=F("%%");break;
      case 34: v=(b-128)*0.01*a;      units=F("kW");break;
      case 35: v=0.01*a*b;            units=F("l/h");break;
      case 36: v=((unsigned long)a)*2560+((unsigned long)b)*10;  units=F("km");break;
      case 37: v=b; break; // oil pressure ?!
      // ADP: FIXME!
      /*case 37: switch(b){
             case 0: sprintf(buf, F("ADP OK (%d,%d)"), a,b); t=String(buf); break;
             case 1: sprintf(buf, F("ADP RUN (%d,%d)"), a,b); t=String(buf); break;
             case 0x10: sprintf(buf, F("ADP ERR (%d,%d)"), a,b); t=String(buf); break;
             default: sprintf(buf, F("ADP (%d,%d)"), a,b); t=String(buf); break;
          }*/
      case 38: v=(b-128)*0.001*a;        units=F("Deg k/w"); break;
      case 39: v=b/256*a;                units=F("mg/h"); break;
      case 40: v=b*0.1+(25.5*a)-400;     units=F("A"); break;
      case 41: v=b+a*255;                units=F("Ah"); break;
      case 42: v=b*0.1+(25.5*a)-400;     units=F("Kw"); break;
      case 43: v=b*0.1+(25.5*a);         units=F("V"); break;
      case 44: sprintf(buf, "%2d:%2d", a,b); t=String(buf); break;
      case 45: v=0.1*a*b/100;            units=F(" "); break;
      case 46: v=(a*b-3200)*0.0027;      units=F("Deg k/w"); break;
      case 47: v=(b-128)*a;              units=F("ms"); break;
      case 48: v=b+a*255;                units=F(" "); break;
      case 49: v=(b/4)*a*0.1;            units=F("mg/h"); break;
      case 50: v=(b-128)/(0.01*a);       units=F("mbar"); break;
      case 51: v=((b-128)/255)*a;        units=F("mg/h"); break;
      case 52: v=b*0.02*a-a;             units=F("Nm"); break;
      case 53: v=(b-128)*1.4222+0.006*a;  units=F("g/s"); break;
      case 54: v=a*256+b;                units=F("count"); break;
      case 55: v=a*b/200;                units=F("s"); break;
      case 56: v=a*256+b;                units=F("WSC"); break;
      case 57: v=a*256+b+65536;          units=F("WSC"); break;
      case 59: v=(a*256+b)/32768;        units=F("g/s"); break;
      case 60: v=(a*256+b)*0.01;         units=F("sec"); break;
      case 62: v=0.256*a*b;              units=F("S"); break;
      case 64: v=float(a+b);             units=F("Ohm"); break;
      case 65: v=0.01*a*(b-127);         units=F("mm"); break;
      case 66: v=(a*b)/511.12;          units=F("V"); break;
      case 67: v=(640*a)+b*2.5;         units=F("Deg"); break;
      case 68: v=(256*a+b)/7.365;       units=F("deg/s");break;
      case 69: v=(256*a +b)*0.3254;     units=F("Bar");break;
      case 70: v=(256*a +b)*0.192;      units=F("m/s^2");break;
      default: sprintf(buf, "%2x, %2x      ", a, b); break;
    }
      if (units.length() != 0){
      dtostrf(v,4, 2, buf); 
      t=String(buf);
    } 

// Блоки и датчики которые ты можеш отобразить на панеле (экране)
    switch (currAddr){
      case ADR_Engine: // Блок двигателя
        switch(group){
          case 4: 
            switch (idx){
              case 0: engineSpeed = v; break;
              case 1: supplyVoltage=v; break;
              case 2: coolantTemp =v; break;
              case 3: intakeAirTemp=v; break;
            }              
            break;
          case 11: 
            switch (idx){
              case 1: engineLoad=v; break;
              case 2: vehicleSpeed =v; break;
              case 3: fuelConsumption=v; break;
            }              
            break;
          case 2: 
            switch (idx){
              case 0: engineSpeed = v; break;
              case 2: injektTime =v; break;
              case 3: MAF=v; break;
            }              
            break;
          case 5: 
            switch (idx){
              case 0: engineSpeed = v; break;
              case 2: vehicleSpeed =v; break;
            }              
            break;
          case 115:
            switch (idx){
              case 0: engineSpeed = v; break;
              case 1: engineLoad=v; break;
              case 2: turboZapros =v; break;
              case 3: turboPress=v-1; break;

            }
            break;
          default:  
            switch (idx){
              case 0: param0 = v; break;
              case 1: param1 = v; break;
              case 2: param2 = v; break;
              case 3: param3 = v; break;    
            }
          break;

        }
        break;
      case ADR_Dashboard: // Блок панели приборов
        switch (group){ 
          case 1:  
            switch (idx){
              case 0: vehicleSpeed = v; break;
              case 1: engineSpeed = v; break;
              case 2: oilPressure = v; break;
            }
            break;
          case 2:
            switch (idx){
              case 0: odometer = v; break;
              case 1: fuelLevel = v; break; 
              case 3: ambientTemperature = v; break;          
            }
            break;
          case 50:
            switch (idx){
              case 1: engineSpeed = v; break;
              case 2: oilTemp = v; break;
              case 3: coolantTemp = v; break;
            }
            break;
         
        }
        break;
    }
    

     if (units.length() != 0){
      dtostrf(v,4, 2, buf); 
      t=String(buf) + " " + units;
    }     
    Serial.println(t);
    
    
  }
  sensorCounter++;
  return true;
}

// Сброс ошибок ЕБУ
bool clearErrors(){
char s[64];
sprintf(s, "\x03%c\x05\x03", blockCounter);
#ifdef DEBUG
Serial.println(F("TUTTUTTUTTUTTUT"));
#endif
if (!KWPSendBlock(s, 4)) return false;
int size = 0;
KWPReceiveBlock(s, 64, size);
if (s[3] == '\xFF' && s[4] == '\xFF') {
#ifdef DEBUG
Serial.println(F("No errors in ECU"));
#endif
return false;
}
if (s[2] != '\xFC') {
#ifdef DEBUG
Serial.println(F("Invalid answer from ECU"));
#endif
return false;
}
#ifdef LOGS
Serial.println(F("------readErrors------"));
#endif
int count = (size-4) / 3;
#ifdef DEBUG
Serial.print(F("count="));
Serial.println(count);
#endif
return true;
}
// Конец сброса ошибок ЕБУ

// Чтение ошибок. Пишется в массив ErrorArray - 0 элемент - количество ошибок, дальше их номера
bool readErrors(){
  // uint8_t ErrorArray[10];
memset(ErrorArray, 0, sizeof(ErrorArray));
char s[64];
sprintf(s, "\x03%c\x07\x03", blockCounter);
#ifdef DEBUG
Serial.println(F("TUTTUTTUTTUTTUT"));
#endif
if (!KWPSendBlock(s, 4)) return false;
int size = 0;
KWPReceiveBlock(s, 64, size);
if (s[3] == '\xFF' && s[4] == '\xFF') {
#ifdef DEBUG
Serial.println(F("No errors in ECU"));
#endif
return false;
}
if (s[2] != '\xFC') {
#ifdef DEBUG
Serial.println(F("Invalid answer from ECU"));
#endif
return false;
}
#ifdef LOGS
Serial.println(F("------readErrors------"));
#endif
int count = (size-4) / 3;
#ifdef DEBUG
Serial.print(F("count="));
Serial.println(count);
#endif
for (int idx=0; idx < count; idx++){
byte a=s[3 + idx*3];
byte b=s[3 + idx*3+1];
byte k=s[3 + idx*3+2];
int v = a*256+b;
ErrorArray[0] = count;
ErrorArray[idx+1] = v;
#ifdef LOGS
//Serial.println(v);
#endif
}
return true;
}
// Конец чтения ошибок

// Функция для получения данных с ЕБУ для группы датчиков
void getECUSensor(int groupNum){
  if (currAddr != ADR_Engine) // Если текущий адрес не равен адресу двигателя
  {
    connect(ADR_Engine, ADR_Engine_Speed); // Подключаемся к ЕБУ двигателя
  } else {
    readSensors(groupNum); // Если уже подключены, читаем данные датчиков
  }
}

// Функция для получения данных с панели приборов
void getDashboardSensor(int groupNum){
  if (currAddr != ADR_Dashboard)
  {
    connect(ADR_Dashboard, ADR_Dashboard_Speed);
  } else {
    readSensors(groupNum);
  }
}

// Прерывание для кнопки
void btnInterrupt() {
  btnUp.tickISR(); // Вызываем функцию обработки нажатия кнопки
}

// Функция для рисования интерфейса главного меню
void drawMainMenu(int currentSelection) {
    fisWriter.initFullScreen();
    fisWriter.sendStringFS(0, 0, 0x21, "MAIN MENU");

    String menuItems[] = {"SUMMARY", "READ ERROR", "ABOUT", "EXIT", "UFIS OFF"};

    for (int i = 0; i < 5; i++) {
        int yPos = 16 + (i * 10); // Позиция для каждого пункта
        
        if (i == currentSelection) {
            // Отображаем выбранный пункт с выделением
            fisWriter.sendMsgFS(0, yPos, 0x20, menuItems[i].length(), menuItems[i].c_str());
        } else {
            // Отображаем обычный текст для остальных пунктов
            fisWriter.sendStringFS(0, yPos, 0x21, menuItems[i]);
        }
    }
}




// Настройка начальных параметров
void setup()
{
  currPage = eeprom_read_byte(0); // Читаем текущую страницу из EEPROM
  pinMode(pinKLineTX, OUTPUT); // Устанавливаем режим для TX линии
  digitalWrite(pinKLineTX, HIGH); // Устанавливаем высокий уровень на TX линии
  pinMode(pinButton, INPUT_PULLUP); // Устанавливаем режим для кнопки с подтяжкой к питанию

  attachInterrupt(0, btnInterrupt, FALLING); // Прерывание для кнопки на падение сигнала

  Serial.begin(9600); // Инициализация последовательного порта со скоростью 9600
  Serial.println(F("SETUP")); // Вывод сообщения в Serial Monitor
  fisWriter.begin(); // Инициализация fisWriter
  fisWriter.reset(); // Сброс fisWriter
  Serial.println(F("START"));
}

// Основной цикл программы
void loop() {
    delay(500); 

    // Display welcome screen once
    if (!welcomeScreenShown) {
        fisWriter.initFullScreen();
        fisWriter.sendStringFS(0, 5 * 8, 0x21, F("WELCOME"));
        delay(2000);
        fisWriter.initFullScreen();
        fisWriter.GraphicFromArray(0, 0, 64, 88, vwlogo, 2);
        delay(2000);
        fisWriter.reset();
        welcomeScreenShown = true;
    }

    btnUp.tick(); // Update button state

    // Handle button hold
    if (btnUp.held()) {
        if (currPage == 10) {
            clearErrors(); // Clear errors for specific page
        } else {
            fullDash = !fullDash; // Toggle display mode
            countLoop = 1;
            fisWriter.reset();
        }
    }

    // Навигация по главному меню и меню
    if (!fullDash) {
			drawMainMenu(currentMenuSelection); // Update the displayed menu
		
        if (btnUp.hasClicks(1)) {
            currPage++;
            if (currPage > 8) currPage = 1;
            eeprom_update_byte(0, currPage);
			
			currentMenuSelection++;
            if (currentMenuSelection >= 5) // Предполагаем 5 пунктов меню
            currentMenuSelection = 0;
        }

        if (btnUp.hasClicks(2)) {
            currPage--;
            if (currPage < 1) currPage = 8;
            eeprom_update_byte(0, currPage);
        }

        // Page actions based on `currPage`
        switch (currPage) {
            case 1:
                getECUSensor(5);
                fisWriter.sendString("SPEED", String(vehicleSpeed) + " km/h");
                break;
            case 2:
                getECUSensor(4);
                fisWriter.sendString("COOLANT", String(coolantTemp) + "C");
                break;
            case 3:
                getECUSensor(4);
                fisWriter.sendString("VOLTAGE", String(supplyVoltage) + "V");
                break;
            case 4:
                getDashboardSensor(2);
                fisWriter.sendString("FUEL", String(fuelLevel) + "L");
                break;
            case 5:
                getECUSensor(5);
                fisWriter.sendString("ENGINE", String(engineSpeed) + " rpm");
                break;
            case 6:
                getECUSensor(4);
                fisWriter.sendString("AIR TEMP", String(intakeAirTemp) + "C");
                break;
            case 7:
                getECUSensor(2);
                fisWriter.sendString("INJ TIME", String(injektTime) + "m/s");
                break;
            case 8:
                getECUSensor(5);
                fisWriter.sendString("MAF", String(MAF) + "g/s");
                break;
        }
    } 
	else {

            // Long button press to confirm menu selection
    btnUp.tick(); // Обновление состояния кнопки

if (btnUp.held()) { // Проверка удержания кнопки
    inSubMenu = true; // Установка флага, что мы зашли в подменю
    switch (currentMenuSelection) {
		
            case 0:
                    {
      {
        getECUSensor(2); // Читаем данные с группы 2
        getECUSensor(4); // Читаем данные с группы 4
        getECUSensor(5); // Читаем данные с группы 5
      } 
      
      {
        getDashboardSensor(2); // Читаем данные ПП с группы 2
      }

      // Инициализация полного экрана для отображения информации
      fisWriter.initFullScreen();
    fisWriter.sendStringFS(0, 0, 0x21, "SUMMARY");
	fisWriter.sendMsgFS(0,10,0x24,13,"INFO ECU"); // 8пикселей занимает текст в висоту 
    // Добавьте код для отображения данных, например:
      fisWriter.sendStringFS(0, 20, 0x05, "ENGINE");
	  fisWriter.sendStringFS(40, 20, 0x05, " " + String(engineSpeed));
      fisWriter.sendStringFS(0, 28, 0x05, "SPEED");
      fisWriter.sendStringFS(40, 28, 0x05, " " + String(vehicleSpeed));
	  fisWriter.sendStringFS(0, 36, 0x05, "AIR TEMP");
      fisWriter.sendStringFS(40, 36, 0x05, " " + String(intakeAirTemp));
	  fisWriter.sendStringFS(0, 44, 0x05, "COOLANT");
      fisWriter.sendStringFS(40, 44, 0x05, " " + String(coolantTemp));
	  fisWriter.sendStringFS(0, 52, 0x05, "FUEL LVL");
      fisWriter.sendStringFS(40, 52, 0x05, " " + String(fuelLevel));	  
	  fisWriter.sendStringFS(0, 60, 0x05, "VOLTAGE");
      fisWriter.sendStringFS(40, 60, 0x05, " " + String(supplyVoltage));
	  fisWriter.sendStringFS(0, 68, 0x05, "INJ TIME");
      fisWriter.sendStringFS(40, 68, 0x05, " " + String(injektTime));
	  fisWriter.sendStringFS(0, 76, 0x05, "MAF");
      fisWriter.sendStringFS(40, 76, 0x05, " " + String(MAF));
	  fisWriter.sendStringFS(0, 84, 0x05, "RET.");
      fisWriter.sendStringFS(40, 84, 0x05, " " + String(param0) + ":" + String(param1) + ":" + String(param2) + ":" + String(param3));
    }
                break;
            case 1:
      fisWriter.sendStringFS(0, 0, 0x21, "READ ERROR");
      fisWriter.sendMsgFS(0,10,0x24,15,"two clicks del"); // 8пикселей занимает текст в висоту 
    
	if(!readErrorflag){
                if (currAddr != ADR_Engine)
  {
    connect(ADR_Engine, ADR_Engine_Speed);
    
  }
          readErrors();
          !readErrorflag;
        //  disconnect();
        }
        if(ErrorArray[errorErrorCount]){

        } else {
          errorErrorCount = 1;
        }
		
        fisWriter.sendStringFS(0, 30, 0x21, "CODE " + String(ErrorArray[0])); //код
		fisWriter.sendStringFS(0, 50, 0x21, "amount " + String(ErrorArray[errorErrorCount])); //количество
        
		Serial.println(ErrorArray[errorErrorCount]);
        errorErrorCount++;
        delay(1000);
                break;
            case 2:
    fisWriter.initFullScreen();
    fisWriter.sendStringFS(0, 0, 0x21, "ABOUT");
	
    fisWriter.sendStringFS(0, 20, 0x05, "UFIS V1");
	fisWriter.sendStringFS(0, 30, 0x05, "TRIPCOMPUTER");
	fisWriter.sendStringFS(0, 40, 0x05, "FOR PQ34 PLATFORM");
	fisWriter.sendStringFS(0, 50, 0x05, "MADE FOR YOURSELF");
	fisWriter.sendStringFS(0, 60, 0x05, "ON FREE LIBRARIES");
                break;
            case 3:
				fullDash = true; // Возврат к основному экрану
                break;
            case 4:
                //Код для выключения устройства

    // Отключение периферийных устройств на пинах 1 и 2
    pinMode(4, OUTPUT);
    pinMode(3, OUTPUT);
    digitalWrite(4, LOW); // Отключить периферийное устройство на пине 1
    digitalWrite(3, LOW); // Отключить периферийное устройство на пине 2

    // Настройка и переход в режим сна
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);
    sleep_enable();
    
    // Отключение ненужных периферийных устройств контроллера
    power_adc_disable();
    power_spi_disable();
    power_timer0_disable();
    power_timer1_disable();
    power_twi_disable();
    
    sleep_mode(); // Переход в спящий режим (устройство засыпает и ждет пробуждения)

    sleep_disable(); // Код сюда не дойдет, так как контроллер проснется по внешнему событию
                break;
        }
	  }
    }
  }
