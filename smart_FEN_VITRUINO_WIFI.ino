#include <ArduinoJson.h>
#include <AvtoFotaNew.h>
const char* MANIFEST_URL = "https://raw.githubusercontent.com/eu1abg/Smart_Fen/main/firmware/firmware.json";
AvtoFotaNew fota("5.00");
//bool otaStarted = false;
 #include <WiFi.h>
//=========================================================================================

#define ST7789_DRIVER 
#define TFT_RGB_ORDER TFT_BGR
#define TFT_WIDTH  240
#define TFT_HEIGHT 320
#define TFT_INVERSION_ON
#define TFT_MOSI 23
#define TFT_SCLK 18
#define TFT_CS   15
#define TFT_DC   2
#define TFT_RST  4
#define LOAD_GLCD
#define SPI_FREQUENCY  27000000
#define SPI_READ_FREQUENCY  20000000
#define SPI_TOUCH_FREQUENCY  2500000

 #include <TFT_eSPI.h>
 #include <SPI.h>
TFT_eSPI tft = TFT_eSPI();
//==========================================================================================
#include <SimplePortal.h>
#include <EEPROM.h>
//==================================================================
#include <PubSubClient.h>
#include <WiFiClient.h>
//=========================================================================================================================================== 
//#include "esp_task_wdt.h"
#define WDT_TIMEOUT 24 // часа
//===========================================================================================================================================
 #include <NTPClient_Generic.h>          // https://github.com/khoih-prog/NTPClient_Generic
#include <WiFiUdp.h>
WiFiUDP ntpUDP;
 #define TIME_ZONE_OFFSET_HRS            (3)
#define SECS_IN_HR                (3600L)
NTPClient timeClient(ntpUDP);
//=========================================================================================================================================== 
#include <DS18B20.h>
DS18B20 sensor1(26); // температура батареи

#include <GyverBME280.h>                            // Подключение библиотеки
GyverBME280 bme;       

//=========================================================================================================================================== 
//#include "GyverRelay.h"
//GyverRelay regulator(REVERSE);
//=========================================================================================================================================== 

#define BTN_PIN 32       // кнопка подключена сюда (BTN_PIN --- КНОПКА --- GND)
#include "GyverButton.h"
GButton butt1(BTN_PIN);

//===========================================================================================================================================
#include <TimerMs.h>
TimerMs tmr1(3000, 1, 0);   // отправляем топики и делаем измерения
TimerMs tmr2(30000, 1, 0);   // запись
TimerMs tmr3(1500, 0, 1);
TimerMs tmr4(60000, 0, 1);   // дубль ;
TimerMs tmr5(1000, 1, 0); 
TimerMs tmr6(500, 0, 1);
TimerMs tmr7(5000, 1, 0);   // делаем измерения CO2 co
TimerMs tmr8(15000, 0, 1);   // прогрев СО
TimerMs tmr9(15000, 1, 0);   //  возвр экр
TimerMs tmr10(7000, 0, 1); // переключ экр
TimerMs tmr11(180000, 1, 0); //  переподкл вайфай
TimerMs tmr12(2000, 1, 0); // измеряем батарею

TimerMs tmr13(300000, 1, 0); // обновление
TimerMs tmr14(1500, 1, 0); //читаем температуру комнаты
TimerMs tmr15(2500, 1, 0); //читаем влажность комнаты
TimerMs tmr16(3000, 1, 0); //читаем давление комнаты
TimerMs tmr17(1000, 1, 0); // вывод инфы
TimerMs tmr18(WDT_TIMEOUT*3600*1000, 0, 1);; // вочдог
//=====================================================
#include <MQUnifiedsensor.h>

//Definitions
//#define placa "esp-32"
//#define Voltage_Resolution 5
//#define pin 34 //Analog input 0 of your arduino
//#define type "MQ-135" //MQ135
//#define ADC_Bit_Resolution 12 // For arduino UNO/MEGA/NANO
#define RatioMQ135CleanAir 3.6//RS / R0 = 3.6 ppm 
#define RatioMQ7CleanAir 27.5 //RS / R0 = 27.5 ppm  
//#define PWMPin 13 // Pin connected to mosfet
MQUnifiedsensor MQ7("esp-32", 3.3, 12, 34, "MQ-7");
MQUnifiedsensor MQ135("esp-32", 3.3, 12, 35, "MQ-135");
//=========================================================
#include "GyverPID.h"
GyverPID regulator1(3, 1, 0.5);  // регулятор охлаждения коэф. П, коэф. И, коэф. Д, период дискретизации dt (мс)
//=========================================================

// const char* ssid = "EPSminsk.by";
// const char* password = "13051973";

const char* mqtt_server = "m6.wqtt.ru"; // replace with your broker url
const char* mqtt_username = "u_XBRH9C";  // you don't need the username and password for public connection
 const char* mqtt_password = "lOzEXIn0";
 const int mqtt_port = 14516;



WiFiClient espClient;
PubSubClient client(espClient);
//==============================================================================================================================
 unsigned long lastMsg = 0;



  int tonePin = 14; 
  //int gist;   // гистерезис нагрева
 // Меняются в прерывании кнопки
volatile bool portal = 0;        // меняется в myIsr() через butt1

// Меняются в MQTT callback
volatile int edit1 = 0;          // меняется в callback()
volatile bool switch1 = 0;       // меняется в callback()
volatile int switch12 = 0;       // меняется в callback()

// Флаги между задачами (Core0 <-> Core1)
volatile bool otaStarted = false;  // читается в Task0, меняется в Task1
volatile bool ekr1 = 0;            // флаг для обновления экрана

// Сигналы от датчиков (если читаешь в одной задаче, а используешь в другой)
volatile float co = 0;             // читается в Task1
volatile float co2 = 0;            // читается в Task1
volatile float t = 0;              // читается в Task1
volatile float h = 0;              // читается в Task1
volatile int   p = 0;              // читается в Task1
volatile float tb = 0;             // читается в Task1

// Таймеры и флаги
volatile bool x3 = 0;              // флаг прогрева CO
volatile bool vent = 0;            // состояние вентилятора


 uint32_t sec, timer, minutes, seconds, hours,timerwifi,timercikl;  
 unsigned long startTime = 0, currentTime = 0;
 String str, str1, fen_time; 
 bool x, x1=0, x2=0,on=0,tx,tmw;
 float vs,dT=0; unsigned long millis_int1=0; 
//======================================================================
 int n=0,flag=0,x6=0,x4=0, x5=0, ekr = 0; 
 char buffer[100];
 
//--------------------------------ТОПИКИ и АйДи -----------------------------------------------------------------------------------------------------
 String incommingMessage="";   uint32_t chipId = 0; 
 String vent_top;              const char* vent_topic; 
 String tb_top;                const char* tb_topic; 
 String t_top;                 const char* t_topic;                        
 String  h_top;                const char*  h_topic;                         
 String co2_top;               const char* co2_topic;                    
 String co_top;                const char* co_topic;                      
String p_top;                const char* p_topic;                     
 String tust_top;             const char* tust_topic;
 String time_top;              const char* time_topic;
 String switch1_top;           const char* switch1_topic; 
String switch11_top;          const char* switch11_topic;
 String edit1_top;             const char* edit1_topic; 
 String edit11_top;            const char* edit11_topic;


void preSetupChipId() {
  uint64_t mac = ESP.getEfuseMac();
  chipId = (uint32_t)(mac >> 24);  // Берем 4 байта MAC-адреса

 vent_top = String(chipId)+"/"+"fen_vent";            vent_topic= vent_top.c_str();  
 tb_top = String(chipId)+"/"+ "fen_tempb";            tb_topic= tb_top.c_str();
 t_top = String(chipId)+"/"+  "fen_temp";             t_topic= t_top.c_str();                       
  h_top = String(chipId)+"/"+ "fen_hum";              h_topic= h_top.c_str();                      
 co2_top = String(chipId)+"/"+ "fen_co2";             co2_topic= co2_top.c_str();                  
 co_top = String(chipId)+"/"+ "fen_co";               co_topic= co_top.c_str();                   
 p_top = String(chipId)+"/"+   "fen_p";               p_topic=  p_top.c_str();                 
 tust_top = String(chipId)+"/"+  "fen_tust";          tust_topic= tust_top.c_str(); 
 time_top = String(chipId)+"/"+     "fen_time";       time_topic= time_top.c_str();
 switch1_top = String(chipId)+"/"+  "fen_vkl";        switch1_topic= switch1_top.c_str();
 switch11_top = String(chipId)+"/"+ "fen_vkl1";       switch11_topic= switch11_top.c_str();
 edit1_top = String(chipId)+"/"+ "fen_edit1";         edit1_topic= edit1_top.c_str(); 
 edit11_top = String(chipId)+"/"+  "fen_edit11";      edit11_topic= edit11_top.c_str();

  


// //-----------------------------------------------------------------------------------------------------------------
// String incommingMessage="";
// const char* vent_topic = "fen_vent";   
// const char* tb_topic = "fen_tempb";     
// const char* t_topic = "fen_temp";                              
// const char*  h_topic="fen_hum";                              
// const char* co2_topic = "fen_co2";                          
// const char* co_topic = "fen_co";                             
// const char* p_topic = "fen_p";                               
// const char* tust_topic = "fen_tust";  
// const char* time_topic = "fen_time";                        
// //-----------------------------------------------------------------------------------------------------------------
// const char* switch1_topic="fen_vkl"; const char* switch11_topic="fen_vkl1";
// const char* edit1_topic="fen_edit1"; const char* edit11_topic="fen_edit11";

}

//==============================================================================================================================
void __attribute__((constructor)) beforeSetup() {preSetupChipId();}

 
 //======================= ВНЕШНЕЕ ПРЕРЫВАНИЕ апаратное по 32 ноге ================================================

IRAM_ATTR void myIsr() {butt1.tick();} 
//==========================================================================================================================
TaskHandle_t TaskOnCore0;
TaskHandle_t TaskOnCore1;
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// --- Задача для первого ядра (Core 0) ---
void Task0Code(void *pvParameters) {for (;;) { 
client.loop(); timeClient.update();
if (!client.connected()) reconnect();


if (tmr1.tick() && otaStarted==0) {

 publishMessage(vent_topic,String(vent),true); 
 publishMessage(tb_topic,String(tb),true); 
 publishMessage(t_topic,String(t),true);    
 publishMessage(h_topic,String(h),true);
 publishMessage(co2_topic,String(co2),true);
 publishMessage(co_topic,String(co),true);
 publishMessage(p_topic,String(p),true);
 publishMessage(tust_topic,String(edit1),true);
 publishMessage(time_topic,fen_time.c_str(),true); 
 publishMessage(switch11_topic,String(switch12),true);
 publishMessage(edit11_topic,String(edit1),true);

} 

 vTaskDelay(100 / portTICK_PERIOD_MS); // Правильная задержка в RTOS


  }}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// --- Задача для второго ядра (Core 1) ---


void Task1Code(void *pvParameters) {for (;;) {

  if (otaStarted) {vTaskDelay(pdMS_TO_TICKS(100));  // отдыхаем, пока идет OTA
      continue;  // пропускаем всю остальную логику
    }
//=========================================================================================================================================== 
if (tmr8.tick()) { 
 // Serial.println("делаем прробел......... ");
  x3=1; tft.setCursor(120, 170);tft.setTextSize(2);tft.setTextColor(TFT_NAVY, TFT_BLACK); tft.print("         ");}

//esp_task_wdt_reset();
 if(x3==0) { tft.setCursor(120, 170);tft.setTextSize(2);tft.setTextColor(TFT_NAVY, TFT_GREEN); tft.print(" Init СО "); co=0;co2=0; }
// Serial.println("проверяем СО......... ");
 if(co > 35 && x3==1) { switch1 = 0;
     if (tmr5.tick()) { tft.setCursor(120, 170);tft.setTextSize(2);tft.setTextColor(TFT_WHITE, TFT_RED);   tft.print("Угар.газ!");tone(tonePin, 1000, 500);tmr6.start();} 
     if (tmr6.tick()) { tft.setCursor(120, 170);tft.setTextSize(2);tft.setTextColor(TFT_WHITE, TFT_BLACK); tft.print("         ");}
    } 
  if(co < 35 && switch1==0 && x3==1 ) {  
  tft.setCursor(120, 170);tft.setTextSize(2);tft.setTextColor(TFT_WHITE, TFT_BLACK); tft.print("         ");}

//=========================================================================================================================================== 



if (tmr7.tick()) { //Serial.println("читаем датчики......... ");
//----------------------------- УПРАВЛЕНИЕ вентиль БАТАРЕЕ ----------------------------------------------------------------------------
 //ledcWriteChannel(1, 100); // шим вентиль ledcWriteTone(1, 10); ledcWrite(1,255);  
//====================================================== CO CO2 T H P ===================================================================================== 
   MQ7.update();  co = MQ7.readSensor(); if(co < 10000) {  co =co; } else co = 9999;//esp_task_wdt_reset();
   MQ135.update(); co2=MQ135.readSensor(); if(co2 < 10000) { co2 = co2; } else co2 = 9999;

  // MQ135.setA(605.18); MQ135.setB(-3.937);  co = MQ135.readSensor(); if(co < 10000) {  co =co; } else co = 9999;
  //  MQ135.setA(110.47); MQ135.setB(-2.862); int co2=400+MQ135.readSensor(); if(co2 < 10000) { co2 = co2; } else co2 = 9999;                                                                          
  //   MQ135.setA(77.255); MQ135.setB(-3.18);  float Alcohol = MQ135.readSensor(); 
  //    MQ135.setA(44.947); MQ135.setB(-3.445); float Toluen = MQ135.readSensor(); 
  //     MQ135.setA(102.2 ); MQ135.setB(-2.473); float NH4 = MQ135.readSensor(); 
  //      MQ135.setA(34.668); MQ135.setB(-3.369); float Aceton = MQ135.readSensor(); 
 // Serial.println("прочитали датчики......... ");
//esp_task_wdt_reset();

 }
 
  //==================================ДИСПЛЕЙ и измерения ========================================================================================================= 
   if (tmr12.tick()) {  //esp_task_wdt_reset(); //Serial.println("читаем батарею......... ");
   tb = sensor1.getTempC();regulator1.input = t; regulator1.setpoint = edit1;
   
    // esp_task_wdt_reset();

if(tb>(edit1+3)) { // esp_task_wdt_reset(); 
//Serial.println("управляем вентилятором......... ");
ledcWrite(25,regulator1.getResultTimer()); vent=1;} 
  else {ledcWrite(25,0);vent=0;}
    ekr1=1;
     }

  if (tmr14.tick()) {   t =  bme.readTemperature() - 3.5; // погрешность
 //Serial.println("читаем температуру комнаты......... ");
   }
  if (tmr15.tick())  { h = bme.readHumidity();}  
  if (tmr16.tick())  {  p = pressureToMmHg(bme.readPressure());}


  //=//===========================================================================================================================================
 if(switch1==1 && otaStarted==0) {   //esp_task_wdt_reset();
//Serial.println("обработк вкл......... ");
   tft.drawCircle(130, 165, 5, TFT_RED);tft.fillCircle(130, 165, 4, TFT_ORANGE);

 if(x1==0){ x1=1; x2=0; timer = millis();tone(tonePin, 1000, 100);}

  sec = (millis() - timer) / 1000ul; seconds = (sec % 3600ul) % 60ul; minutes = (sec % 3600ul) / 60ul; hours = (sec / 3600ul);
  sprintf (buffer, "%02d:%02d:%02d", hours, minutes,seconds ); fen_time = buffer;
  //Serial.println("обработк буфера прошла......... ");
 //esp_task_wdt_reset();
   if( hours == 10 && switch1==1) {digitalWrite(27, LOW); switch1=0; n=0; tone(tonePin, 500, 3000); Serial.println("таймер......... "); }
    //Serial.print("regulator.getResultTimer() = ");Serial.println(regulator.getResultTimer());
//Serial.println("проверка таймера прошла......... ");
//  bool ResultTimer= regulator.getResultTimer();
// Serial.println("Индик прошел...");
//    if(ResultTimer==1 && flag==0) {Serial.println("Нагр..."); str=" Нагр.!"; ekr= 0xF800; x=1; digitalWrite(33, LOW);tmr3.start();flag=1;n=0;tone(tonePin, 1000, 100);}
//    if(ResultTimer==0 && flag==1) {Serial.println("Охл..."); str=" Охл. !"; ekr= 0x001F;x=0; digitalWrite(27, LOW); tmr3.start();flag=0;n=0;tone(tonePin, 2000, 100); }
// Serial.println("стираем...");

     if (tmr5.tick()) { tft.setCursor(145, 170);tft.setTextSize(2);tft.setTextColor(TFT_WHITE, ekr); tft.print(str);tmr6.start(); } 
  //Serial.println("tmr5 прошла......... ");
     if (tmr6.tick()) { tft.setCursor(145, 170);tft.setTextSize(2);tft.setTextColor(TFT_WHITE, TFT_BLACK); tft.print("       "); }
    //  Serial.println("tmr6 прошла......... ");
      // else if(switch1==0){   }
      // Serial.println("прошли else if..."); 
//=========================================================================================================================================== 
} else 
{ //Serial.println("если выкл кнопку......... ");
 tft.drawCircle(130, 165, 5, TFT_BLACK);tft.fillCircle(130, 165, 5, TFT_BLACK); 
if(x2==0){   x2=1; x1 = 0; tmr3.start();tone(tonePin, 2000, 100); digitalWrite(27, LOW);
tft.setCursor(145, 170);tft.setTextSize(2);tft.setTextColor(TFT_WHITE, TFT_BLACK); tft.print("       "); }
sprintf (buffer, "%02d:%02d:%02d", 0, 0, 0 ); fen_time = buffer;
//Serial.println(" выкл кнопку......... ");
}

//===================================== Кнопка ==============================================================================================
//Serial.println(" опрос кнопки......... ");
if (butt1.isClick()) { tone(tonePin, 2000, 100); switch1=1; switch12=333; }

if (butt1.isHolded()) {switch1=0; flag=0; switch12=0;
tft.setCursor(145, 170);tft.setTextSize(2);tft.setTextColor(TFT_WHITE, TFT_BLACK); tft.print("       ");}
//Serial.println(" прошли опрос кнопку......... ");
//=========================================================================================================================================== 
//Serial.println(" обн......... ");
 // if(tmr13.tick()) obnovl();   // обновление
  //Serial.println("экран......... ");
  ekran();
 //if(n < 5) dublclic(); //Serial.println(digitalRead(32));
if (tmr3.tick()) {digitalWrite(33, HIGH); digitalWrite(27, HIGH); tmr4.start(); n=1; Serial.println("tmr3 ...");}
//Serial.println("вклвыкл......... ");
 onoff();
 //if(tmw==1) {Serial.print("timercikl = "); Serial.println( (millis()-timercikl)/1000); tmw=0; delay(3000);}
//esp_task_wdt_reset();  
   if (tmr11.tick()) { // Serial.println("проверка вайфай......... ");
  if (WiFi.status() != WL_CONNECTED) { 
     WiFi.disconnect();   EEPROM.get(0, portalCfg.SSID); EEPROM.get(150, portalCfg.pass); WiFi.mode(WIFI_STA); WiFi.begin(portalCfg.SSID, portalCfg.pass); 
     }}
if (WiFi.status() == 3)  { tft.drawCircle(30, 100, 8, TFT_BLUE );tft.fillCircle(30, 100, 7, TFT_GREENYELLOW);  }
   else {  tft.drawCircle(30, 100, 8, TFT_BLUE );tft.fillCircle(30, 100, 7, TFT_BLACK);}
 //Serial.println("рисуем кружек......... ");
 //if (tmr2.tick()) {EEPROM.put(100,edit1); EEPROM.put(110,switch1); EEPROM.commit(); }
 if(tmr13.tick()) obnovl();   // обновление   
   
    vTaskDelay(100 / portTICK_PERIOD_MS);
 }}




//==============================================================================================================================
void setup() { 
  Serial.begin(115200); EEPROM.begin(500);

  
//=========================================================================================
  wifisel();
fota.setManifestURL(MANIFEST_URL);
 fota.setDebug(true);
// 🔥 ВОТ КЛЮЧЕВОЕ
  fota.setProgressCallback([](uint8_t p) {
    Serial.printf("OTA progress: %d%%\n", p);
    tft.setCursor(100, 250);tft.setTextSize(2);tft.setTextColor(TFT_WHITE, TFT_BLACK); tft.print(p); tft.print("%"); // esp_task_wdt_reset();
  // oled.setCursor(55, 2);oled.print(p);oled.print("%");oled.update();
  //  if(p == 0)   str1 = "Download Udate.    ";
  //     if(p<100) stroka = " "+ str1 + p +"  %";
  //    if (p == 100) stroka = " Installing !!! ";   
  
 
  //  publishMessage(stroka_topic,stroka.c_str(),true);
  });
  //==============================================================================================================================
  attachInterrupt(32, myIsr, CHANGE);// внешнее прерывание
  pinMode(14, OUTPUT); // пищалка  
  pinMode(34, INPUT);  // напряжение вход СО2
  pinMode(35, INPUT);  // напряжение вход СО
  pinMode(33, OUTPUT);  // ON
  pinMode(25, OUTPUT);   //  шим вентиль
 // ledcAttachPin(25, 1); ledcSetup(1, 400, 8);
  ledcAttach(25, 6000, 8);
  pinMode(27, OUTPUT);   // OFF
  pinMode(32, INPUT_PULLUP);  //  кнопка
  //pinMode(3, OUTPUT);  // 
  digitalWrite(33, HIGH);digitalWrite(27, HIGH); //digitalWrite(PWMPin, HIGH); //Serial.print("PWMPin = 0 "); delay(3000);

  edit1 = EEPROM.read(100); switch1= EEPROM.read(110); tx=EEPROM.read(400);
//==================================================================================================================
     bme.setFilter(FILTER_COEF_8);                     // Настраиваем коофициент фильтрации
     bme.setTempOversampling(OVERSAMPLING_8);          // Настраиваем передискретизацию для датчика температуры
     bme.setPressOversampling(OVERSAMPLING_16);        // Настраиваем передискретизацию для датчика давления
     bme.setStandbyTime(STANDBY_500MS);                // Устанавливаем время сна между измерениями (у нас обычный циклический режим)
     bme.begin();                                      // Если на этом настройки окончены - инициализир


   //dht.begin();
//==============================================================================================================================
tft.init(); tft.setRotation(0);tft.fillScreen(TFT_BLACK);
 tft.setTextColor(TFT_YELLOW,TFT_BLUE );tft.setTextSize(4);
  tft.setCursor(40, 100);tft.print("<Смарт>");tft.setCursor(50, 140);tft.print("<ФЕН!>"); tft.setCursor(75, 180);tft.print(fota.getVER()); tft.setTextColor(TFT_YELLOW,TFT_BLACK );
  tft.setCursor(0,200);tft.setTextSize(1);delay(3000);
//=======================================WIFI=======================================================================================


   
//========================================================================================
  tone(tonePin, 783, 165.441);delay(183.823333333);tone(tonePin, 659, 165.441);delay(183.823333333);tone(tonePin, 880, 165.441);delay(3000);
  tft.fillScreen(TFT_BLACK);
//==============================================================================================================================
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);
  
  
  butt1.setDebounce(90);        // настройка антидребезга (по умолчанию 80 мс)
  butt1.setTimeout(1000);
  butt1.setTickMode(AUTO);
  //==============================================================================================================================
   MQ135.setRegressionMethod(1); //_PPM =  a*ratio^b
   MQ135.setA(102.2); MQ135.setB(-2.473);
   MQ135.setRL(1); MQ135.init(); 
   MQ7.setRegressionMethod(1); //_PPM =  a*ratio^b
   MQ7.setA(99.042); MQ7.setB(-1.518); // Configure the equation to calculate CO concentration value
   MQ7.setRL(1);
   MQ7.init();
   initco2();


//===========================================================================================================================================          
   timeClient.begin();
   timeClient.setTimeOffset(3600 * TIME_ZONE_OFFSET_HRS);
   timeClient.setUpdateInterval(SECS_IN_HR);
//================== Регулятор охлаждения ==========================================================================================
   regulator1.setDirection(NORMAL); // направление регулирования (NORMAL/REVERSE). ПО УМОЛЧАНИЮ СТОИТ NORMAL
   regulator1.setLimits(0, 1024);    // пределы (ставим для 8 битного ШИМ). ПО УМОЛЧАНИЮ СТОЯТ 0 И 255
   //regulator1.setpoint = 33;        // сообщаем регулятору температуру радиатора
  
  //regulator1.k = 0.5;          // коэффициент обратной связи (подбирается по факту)
  regulator1.setDt(2000);      // установить время итерации для getResultTimer
//============================================================================================================================================
//=========================================================================================================================================   
 digitalWrite(27, LOW); delay(500); digitalWrite(27, HIGH);
  //===========================================================================================================================
   tmr8.start();  tmr18.start();  
   //=========================================================================================================================================
xTaskCreatePinnedToCore(Task0Code,"Task0",10000,NULL,1,&TaskOnCore0,0 );
xTaskCreatePinnedToCore(Task1Code,"Task1",10000,NULL,1,&TaskOnCore1,1 );
/////////////// Настройка конфигурации watchdog////////////////////////
//     esp_task_wdt_config_t wdt_config = {
//     .timeout_ms = WDT_TIMEOUT * 1000, // конвертируем секунды в миллисекунды
//     .idle_core_mask = (1 << 1),                                                       //idle_core_mask = 0, // Следим за всеми ядрами (битовая маска) .idle_core_mask = (1 << 1), // Только ядро 0  
//     .trigger_panic = true, // Заменить на true для перезагрузки при срабатывании
// };

//     esp_task_wdt_init(&wdt_config);
//     esp_task_wdt_add(NULL);
  
}



//================================================
void loop() { 

 
}



 //====================================================================================================co = analogRead(35);=======================================
// void dublclic(){ esp_task_wdt_reset(); 
// if(tmr3.active()) return;

//   if (flag==1) { if(tmr4.tick()){ digitalWrite(33, LOW); tmr3.start(); n=n+1; esp_task_wdt_reset(); yield(); } }
//   if (flag==0 or switch1==0) { if(tmr4.tick()){ digitalWrite(27, LOW); tmr3.start(); n=n+1;esp_task_wdt_reset(); yield(); } } 
// }
//==============================================================================================================================
void onoff(){
  if(!switch1) return;
 //Serial.println("вошли  в выкл......... ");
   if( t < edit1-dT && flag==0) {dT=0.7;Serial.println("Нагр..."); str=" Нагр.!"; ekr= 0xF800; x=1; digitalWrite(33, LOW);tmr3.start();flag=1;n=0;tone(tonePin, 1000, 100);}
      else if( t > edit1+dT && flag==1) {dT=1.5;Serial.println("Охл..."); str=" Охл. !"; ekr= 0x001F;x=0; digitalWrite(27, LOW); tmr3.start();flag=0;tone(tonePin, 2000, 100); }
//Serial.println("прошли условия темп......... ");
if(flag==1 && n==1) {if(tmr4.tick()){ digitalWrite(33, LOW);tmr3.start();Serial.println("Дубль на нагрев..."); }}
if(flag==0 && n==1) {if(tmr4.tick()){ digitalWrite(27, LOW);tmr3.start();Serial.println("Дубль на охл..."); }}
//Serial.println("прошли условия дубль......... ");
if(tmr17.tick()){
 Serial.print("flag---"); Serial.println(flag); 
 Serial.print("dT---"); Serial.println(dT); 
 Serial.print("n---"); Serial.println(n);

}
//Serial.println("выходим оноф......... ");
}
//==============================================================================================================================
void ekran(){
  if(!ekr1) return;
if(otaStarted) return;

 //esp_task_wdt_reset();
tft.drawRoundRect(1, 1, 238, 70, 20, TFT_RED); tft.setCursor(100, 6);tft.setTextSize(2);tft.setTextColor(TFT_PURPLE, TFT_BLACK);tft.print("Дата.");
   tft.setCursor(5, 30);tft.setTextSize(3);tft.setTextColor(TFT_YELLOW, TFT_BLACK);
   tft.print(String(" ") + timeClient.getDay() + " " + timeClient.getMonthStr() + " " + timeClient.getYear()); 
     tft.drawRoundRect(1, 75, 238, 70, 20, TFT_RED);tft.setCursor(100, 81);tft.setTextSize(2);tft.setTextColor(TFT_PURPLE, TFT_BLACK);tft.print("Время.");
     tft.setCursor(60, 105);tft.setTextSize(3);tft.setTextColor(TFT_YELLOW, TFT_BLACK);
     tft.print(timeClient.getFormattedTime()); 
      tft.drawRoundRect(1, 152, 238, 50, 20, TFT_CYAN);
        tft.setCursor(17, 170);tft.setTextSize(2);tft.setTextColor(TFT_WHITE, TFT_BLACK); tft.print("Туст. ");tft.print(edit1);
  //esp_task_wdt_reset();
if(tmr9.tick()) {tmr10.start(); 
        tft.drawRoundRect(1, 207, 238, 113, 20, TFT_GREENYELLOW);
        tft.setCursor(15, 220);tft.setTextSize(3);tft.setTextColor(TFT_MAGENTA, TFT_BLACK); tft.print("Тбат. ");tft.setCursor(125, 220);tft.print("     ");tft.setCursor(125, 220);tft.print(tb);//tft.setCursor(125, 220);tft.print("     ");
        //tft.setCursor(15, 220);tft.setTextSize(3);tft.setTextColor(TFT_MAGENTA, TFT_BLACK); tft.print(String("")+"Темп."+" "+t+" ");
        tft.setCursor(15, 250);tft.setTextSize(3);tft.setTextColor(TFT_GREENYELLOW, TFT_BLACK); tft.print("Влаж. ");tft.setCursor(127, 250);tft.print("     ");tft.setCursor(127, 250);tft.print(h);//tft.setCursor(127, 250);tft.print("     ");
        //tft.setCursor(15, 250);tft.setTextSize(3);tft.setTextColor(TFT_GREENYELLOW, TFT_BLACK); tft.print(String("")+"Влаж."+" "+h+" ");
        tft.setCursor(15, 280);tft.setTextSize(3);tft.setTextColor(TFT_ORANGE, TFT_BLACK); tft.print("CO ppm ");tft.setCursor(135, 280);tft.print("     ");tft.setCursor(135, 280);tft.print(co);//tft.setCursor(135, 280);tft.print("     ");
        //tft.setCursor(15, 280);tft.setTextSize(3);tft.setTextColor(TFT_ORANGE, TFT_BLACK); tft.print(String("")+"CO ppm"+" "+co+" ");  
           } 
if(tmr10.tick()) {
        tft.drawRoundRect(1, 207, 238, 113, 20, TFT_MAGENTA);
        tft.setCursor(15, 220);tft.setTextSize(3);tft.setTextColor(TFT_YELLOW, TFT_BLACK); tft.print("Темп. ");tft.setCursor(125, 220);tft.print("     ");tft.setCursor(125, 220);tft.print(t);//tft.setCursor(125, 220);tft.print("     ");
        //tft.setCursor(15, 220);tft.setTextSize(3);tft.setTextColor(TFT_YELLOW, TFT_BLACK); tft.print(String("")+"Темп."+" "+t+" ");
        tft.setCursor(15, 250);tft.setTextSize(3);tft.setTextColor(TFT_RED, TFT_BLACK); tft.print("Давл. ");tft.setCursor(127, 250);tft.print("     ");tft.setCursor(127, 250);tft.print(p);//tft.setCursor(127, 250);tft.print("     ");
        //tft.setCursor(15, 250);tft.setTextSize(3);tft.setTextColor(TFT_RED, TFT_BLACK); tft.print(String("")+"Давл. "+" "+p+" ");
        tft.setCursor(15, 280);tft.setTextSize(3);tft.setTextColor(TFT_WHITE, TFT_BLACK); tft.print("CO2ppm ");tft.setCursor(135, 280);tft.print("     ");tft.setCursor(135, 280);tft.print(co2); //tft.setCursor(135, 280);tft.print("     ");
        //tft.setCursor(15, 280);tft.setTextSize(3);tft.setTextColor(TFT_WHITE, TFT_BLACK); tft.print(String("")+"CO2ppm"+" "+co2+" ");
           }


ekr1=0; //esp_task_wdt_reset(); 
}
//==============================================================================================================================
 void callback(char* topic, byte* payload, unsigned int length) {   String incommingMessage = ""; bool swit;
      for (int i = 0; i < length; i++) incommingMessage+=(char)payload[i]; Serial.println("Message arrived ["+String(topic)+"]"+incommingMessage);
    
     if( strcmp(topic,switch1_topic) == 0){ if (incommingMessage.equals("333")) {swit = 1; switch12=333; } 
        else { if (incommingMessage.equals("0")){ if(switch1==1)  dT=0; swit = 0;  switch12=0; }}  if(swit!=switch1) { switch1=swit; EEPROM.put(110,switch1); EEPROM.commit();}  }  //  включаем нагрев
      //esp_task_wdt_reset();   
     int editVREM;
     if (strcmp(topic,edit1_topic) == 0) {editVREM = incommingMessage.toInt(); if(editVREM!=edit1) {edit1=editVREM; EEPROM.put(100,edit1);EEPROM.commit();incommingMessage = "";}}

  //Serial.print("switch1 =  "); Serial.println(switch1);
  //Serial.print("edit1 =  "); Serial.println(edit1);
  
  }

//==============================================================================================================================
 void obnovl() {  //Serial.println(" вошли в обн......... ");
 //esp_task_wdt_delete(NULL); // ⛔ отключаем WDT ДО OTA 
 // Serial.println(" отключили воч дог......... ");
 if (fota.getchekupdate()){ tone(tonePin, 2000, 1000); 
 otaStarted = 1;
 String ver, notes;
 fota.getupdate(ver, notes); 
    Serial.println("=== UPDATE AVAILABLE ===");
    Serial.print("New version : "); Serial.println(ver);
    Serial.print("Notes       : "); Serial.println(notes);
 tft.fillScreen(TFT_BLACK);
   tft.drawRoundRect(1, 152, 238, 50, 20, TFT_CYAN);
        tft.setCursor(15, 170);tft.setTextSize(2);tft.setTextColor(TFT_WHITE, TFT_BLACK); tft.print("Update Ver. ");tft.print(ver);
  tft.drawRoundRect(1, 207, 238, 113, 20, TFT_MAGENTA);
        tft.setCursor(3, 220);tft.setTextSize(2);tft.setTextColor(TFT_YELLOW, TFT_BLACK); tft.println(" Notes:  ");tft.print(notes);
 delay(3000); tft.fillScreen(TFT_BLACK);
 tft.drawRoundRect(1, 152, 238, 50, 20, TFT_CYAN);
         tft.setCursor(15, 170);tft.setTextSize(2);tft.setTextColor(TFT_WHITE, TFT_BLACK); tft.print("Update Begin !!!");tone(tonePin, 800, 3000);
         fota.updateNOW(true);
          }
 // esp_task_wdt_add(NULL);
 //Serial.println(" обн нет......... ");
  // }   
// //----------------------------------------------------------------------------------------------------
}
//==============================================================================================================================
void publishMessage(const char* topic, String payload , boolean retained){
  if (client.publish(topic, payload.c_str(), retained))
      Serial.println("Message publised ["+String(topic)+"]: "+payload);
}
//==============================================================================================================================
void reconnect() { 
 
  static uint32_t lastTry = 0;
  if (millis() - lastTry < 5000) return;
  lastTry = millis();
  
  //if (WiFi.status() == 3) { while (!client.connected()) {Serial.print("Attempting MQTT connection...");String clientId = "ESP32Client-";
 //clientId += String(random(0xffff), HEX);
String clientId = "ESP32Client-";
    if (client.connect(clientId.c_str(), mqtt_username, mqtt_password)) {Serial.println("connected"); tft.drawCircle(30, 123, 8, TFT_RED);tft.fillCircle(30, 123, 7, TFT_BLUE);
    client.subscribe(switch1_topic); 
    client.subscribe(edit1_topic);
    //subscribe the topics here
      //client.subscribe(command2_topic);   
      } else {
      Serial.print("failed, rc=");Serial.print(client.state());Serial.println(" try again in 5 seconds"); 
      tft.drawCircle(30, 123, 8, TFT_RED); //tft.fillCircle(30, 123, 7, TFT_BLACK);
    //client.subscribe(switch1_topic);    delay(5000);
      } 
 // }
//  } 

}
//==============================================================================================================================

 
//===========================================================================================================================================
 void initco2() { 
   float calcR0 = 0;
  for(int i = 1; i<=10; i ++)
   {MQ135.update(); calcR0 += MQ135.calibrate(RatioMQ135CleanAir);Serial.print(".");}
   MQ135.setR0(calcR0/10);
     if(isinf(calcR0)) { tft.setCursor(15, 10);tft.setTextSize(2);tft.setTextColor(TFT_GREENYELLOW, TFT_BLACK); 
     tft.print("Warning MQ135: Conection issue, R0 is infinite (Open circuit detected) please check your wiring and supply");
     Serial.println("Warning MQ135: Conection issue, R0 is infinite (Open circuit detected) please check your wiring and supply"); return;
     }
   if(calcR0 == 0){ tft.setCursor(15, 10);tft.setTextSize(2);tft.setTextColor(TFT_GREENYELLOW, TFT_BLACK); 
    tft.print("Warning MQ135: Conection issue found, R0 is zero (Analog pin shorts to ground) please check your wiring and supply");
    Serial.println("Warning MQ135: Conection issue found, R0 is zero (Analog pin shorts to ground) please check your wiring and supply"); return;
   }
  calcR0 = 0;
  for(int i = 1; i<=10; i ++)
   { MQ7.update(); calcR0 += MQ7.calibrate(RatioMQ7CleanAir);Serial.print("."); }
   MQ7.setR0(calcR0/10);
   Serial.println("  done!.");
  
   if(isinf(calcR0)) { tft.setCursor(15, 200);tft.setTextSize(2);tft.setTextColor(TFT_ORANGE, TFT_BLACK); 
    tft.print("Warning MQ7: Проблема с подключением, R0 бесконечен (обнаружен обрыв цепи) пожалуйста, проверьте вашу проводку и источник питания");
    Serial.println("Warning MQ7: Проблема с подключением, R0 бесконечен (обнаружен обрыв цепи) пожалуйста, проверьте вашу проводку и источник питания"); return;
   }
   if(calcR0 == 0){  tft.setCursor(15, 200);tft.setTextSize(2);tft.setTextColor(TFT_ORANGE, TFT_BLACK); 
    tft.print("Warning MQ7: Обнаружена проблема с подключением, R0 равен нулю (аналоговый вывод замыкается на землю) пожалуйста, проверьте проводку и источник питания");
    Serial.println("Warning MQ7: Обнаружена проблема с подключением, R0 равен нулю (аналоговый вывод замыкается на землю) пожалуйста, проверьте проводку и источник питания");return;
  }

   MQ7.serialDebug(true);
 
}
//==============================================================================================================================
void wifisel(){ 
label0:
 
 if(digitalRead(32)==0) { portal=1;}
 //if(digitalRead(13)==0) { portal=1;}
  else { if(portal==0){
   EEPROM.get(0, portalCfg.SSID); EEPROM.get(150, portalCfg.pass); WiFi.mode(WIFI_STA); WiFi.begin(portalCfg.SSID, portalCfg.pass); 
    timerwifi = millis(); tft.fillScreen(TFT_BLACK);tft.setTextSize(3);tft.setCursor(0, 50);tft.print(" Подключение ");tft.println(portalCfg.SSID);tft.println(portalCfg.pass);
  while (WiFi.status() != WL_CONNECTED) {tft.print("."); delay(200);
    if((millis()-timerwifi) > 15000) { portal=1; WiFi.disconnect(); tft.fillScreen(TFT_BLACK);tft.setTextSize(3);tft.setCursor(0, 50);tft.print("Портал старт."); goto label0;} }
    }}
if (portal==1) {portalRun(180000); 
   switch (portalStatus()) {
        case SP_SUBMIT: portal=0; 
  EEPROM.put(0,portalCfg.SSID);
  EEPROM.put(150,portalCfg.pass); EEPROM.commit();
   char SSI[32];
    EEPROM.get(0, SSI);tft.setCursor(0, 200);tft.print(SSI);
    EEPROM.get(150, SSI);tft.setCursor(0, 250);tft.print(SSI); delay(5000);
   goto label0;  break;

        case SP_SWITCH_AP: portal=2;WiFi.mode(WIFI_AP); WiFi.softAP("SMARTfen", "12345678"); break;  
        case SP_SWITCH_LOCAL: portal=0; break;
        case SP_EXIT:  portal=0; goto label0;    break;
        case SP_TIMEOUT: portal=2;  goto label0;    break;                    /// WiFi.mode(WIFI_AP); WiFi.softAP("SMARTfen", "12345678");   break;
        case SP_ERROR:   portal=1; goto label0;  break;
 }
}
 
    
}

//==============================================================================================================================


