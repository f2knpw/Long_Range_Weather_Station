
#include <WiFi.h>
//#include <HTTPClient.h>
#include <WiFiClient.h>
#include <TelnetStream.h>
#include "driver/adc.h"
#include <esp_wifi.h>
#include <esp_bt.h>

#define PIN_LED 22
//FDRS
#include "fdrs_gateway_config.h"  //https://github.com/timmbogner/Farm-Data-Relay-System
#include <fdrs_gateway.h>
extern DataReading myCtrlData[256];
extern uint8_t myCtrl;

float sensorValues[100][100][1];  //sensorValues[GTW, sensorIndex, value];
boolean gtwHasSentStatus[100];
int forceSleep = 0;

// ThingSpeak settings
char thingserver[] = "api.thingspeak.com";
String writeAPIKey = "YOUR_API_WRITE_KEY";


long timeOut;
long telnetTimeOut ;
#ifdef USE_MQTT
long sendTimeTimeOut = 10000; //timeout if no wifi or no connection MQTT
#else
long sendTimeTimeOut = 10000; //timeout if no wifi only
#endif
int nbSent = 0;
#define LED_PIN 22

enum {idleStatus, sendingTime, waitingSensor, sleeping};        // 0: idleStatus, 1: sendingTime, 2: waitingSensor, 3: sleeping
int gtwStatus = idleStatus;
int currentGTWindex = 1;
int nbGtw = 1;
boolean hasReceivedSensors = false;
int retry = 0;



String ssid = "";
String password = "";
boolean hasWifiCredentials = false;
boolean hasNtpTime = false;                 //UTC time not acquired from NTP
int timeZone = 0;   //set to UTC
const int dst = 0;
//SUN
//PUT YOUR LATITUDE, LONGITUDE, AND TIME ZONE HERE
double latitude = 43.6;        //Toulouse
double longitude = 1.433333;
double transit, sunrise, sunset;  //transit is time at "noon" -not used-


//these variable remain in RTC memory even in deep sleep or after software reset (https://github.com/espressif/esp-idf/issues/7718)(https://www.esp32.com/viewtopic.php?t=4931)
//RTC_NOINIT_ATTR boolean hasRtcTime = false;   //UTC time not acquired from smartphone
RTC_DATA_ATTR boolean hasRtcTime = false;   //will only survive to deepsleep reset... not software reset
RTC_NOINIT_ATTR int hours;                  //will survive to software reset and deepsleep reset
RTC_NOINIT_ATTR int seconds;
RTC_NOINIT_ATTR int tvsec;
RTC_NOINIT_ATTR int minutes;
RTC_NOINIT_ATTR int days;
RTC_NOINIT_ATTR int months;
RTC_NOINIT_ATTR int years;
RTC_NOINIT_ATTR long thingspeakTimeOut ;

//time
#include <TimeLib.h>
int timeToSleep = 5;     //in minutes
int marginToWakeUp = 10;  //in seconds
#define uS_TO_S_FACTOR 1000000ULL  /* Conversion factor for micro seconds to seconds */



//RTC DS1302
#include <Ds1302.h>
#define ENA_PIN 18
#define CLK_PIN 19
#define DAT_PIN 23
// DS1302 RTC instance
Ds1302 rtc(ENA_PIN, CLK_PIN, DAT_PIN);


const static char* WeekDays[] =
{
  "Monday",
  "Tuesday",
  "Wednesday",
  "Thursday",
  "Friday",
  "Saturday",
  "Sunday"
};
#include <SolarCalculator.h> //https://github.com/jpb10/SolarCalculator
int  margin = 10;

boolean touchWake = false;
boolean resetWake = false;
touch_pad_t touchPin;
int threshold = 40; //Threshold value for touchpads pins

//Preferences
#include <Preferences.h>
Preferences preferences;


//WifiManager
#include <DNSServer.h>
#include <WebServer.h>
#include <WiFiManager.h>          //https://github.com/tzapu/WiFiManager

//define your default values here, if there are different values in config.json, they are overwritten.
char ascMargin[3];  //should contain 2 char "20" margin in minutes
String strAscMargin; //same in String


//flag for saving data
bool shouldSaveConfig = false;
bool touch9detected = false;  //
bool touch3detected = false;  //touch3 used to launch WifiManager (hold it while reseting)

//callback notifying us of the need to save config
void saveConfigCallback () {
  Serial.println("Should save config");
  shouldSaveConfig = true;

}



#define DEBUG_WIFI   //debug Wifi 
#define DEBUG_SLEEP
#define DEBUG_UDP    //broadcast info over UDP
#define DEBUG_PREFS  //debug preferences
#define DEBUG_VCC
#define DEBUG_THINGSPEAK
#define DEBUG_TELNET
//#define DEBUG

#include "rom/rtc.h"
void print_reset_reason(int reason) //Print last reset reason of ESP32
{
  switch ( reason)
  {
    case 1 :                                                    //Vbat power on reset
      Serial.println ("POWERON_RESET");
      resetWake = true;
      hasRtcTime = false;                                       //this is the only reset case where RTC memory persistant variables are wiped
      break;
    case 3 : Serial.println ("SW_RESET"); break;                //Software reset digital core
    case 4 : Serial.println ("OWDT_RESET"); break;              //Legacy watch dog reset digital core
    case 5 :                                                    //Deep Sleep reset digital core
      Serial.println ("DEEPSLEEP_RESET");
      print_wakeup_reason();
      break;
    case 6 : Serial.println ("SDIO_RESET"); break;              //Reset by SLC module, reset digital core
    case 7 : Serial.println ("TG0WDT_SYS_RESET"); break;        //Timer Group0 Watch dog reset digital core
    case 8 : Serial.println ("TG1WDT_SYS_RESET"); break;        //Timer Group1 Watch dog reset digital core
    case 9 : Serial.println ("RTCWDT_SYS_RESET"); break;        //RTC Watch dog Reset digital core
    case 10 : Serial.println ("INTRUSION_RESET"); break;        //Instrusion tested to reset CPU
    case 11 : Serial.println ("TGWDT_CPU_RESET"); break;        //Time Group reset CPU
    case 12 : Serial.println ("SW_CPU_RESET"); break;           //Software reset CPU
    case 13 : Serial.println ("RTCWDT_CPU_RESET"); break;       //RTC Watch dog Reset CPU
    case 14 : Serial.println ("EXT_CPU_RESET"); break;          //for APP CPU, reseted by PRO CPU
    case 15 : Serial.println ("RTCWDT_BROWN_OUT_RESET"); break; //Reset when the vdd voltage is not stable
    case 16 : Serial.println ("RTCWDT_RTC_RESET"); break;       //RTC Watch dog reset digital core and rtc module
    default : Serial.println ("NO_MEAN");
  }
}

void print_wakeup_reason()  //deepSleep wake up reason
{
  esp_sleep_wakeup_cause_t wakeup_reason;
  wakeup_reason = esp_sleep_get_wakeup_cause();
  switch (wakeup_reason)
  {
    case ESP_SLEEP_WAKEUP_EXT0 : Serial.println("Wakeup caused by external signal using RTC_IO"); break;
    case ESP_SLEEP_WAKEUP_EXT1 : Serial.println("Wakeup caused by external signal using RTC_CNTL"); break;
    case ESP_SLEEP_WAKEUP_TIMER : Serial.println("Wakeup caused by timer"); break;
    case ESP_SLEEP_WAKEUP_TOUCHPAD :
      Serial.println("Wakeup caused by touchpad");
      touchWake = true;
      print_wakeup_touchpad();
      break;
    case ESP_SLEEP_WAKEUP_ULP : Serial.println("Wakeup caused by ULP program"); break;
    default : Serial.printf("Wakeup was not caused by deep sleep: %d\n", wakeup_reason); break;
  }
}

void print_wakeup_touchpad() {
  touchPin = esp_sleep_get_touchpad_wakeup_status();

  switch (touchPin)
  {
    case 0  : Serial.println("Touch detected on GPIO 4"); break;
    case 1  : Serial.println("Touch detected on GPIO 0"); break;
    case 2  : Serial.println("Touch detected on GPIO 2"); break;
    case 3  : Serial.println("Touch detected on GPIO 15"); break;
    case 4  : Serial.println("Touch detected on GPIO 13"); break;
    case 5  : Serial.println("Touch detected on GPIO 12"); break;
    case 6  : Serial.println("Touch detected on GPIO 14"); break;
    case 7  : Serial.println("Touch detected on GPIO 27"); break;
    case 8  : Serial.println("T9 detected "); break;  //GPIO32
    case 9  : Serial.println("T8 detected "); break;  //GPIO33
    default : Serial.println("Wakeup not by touchpad"); break;
  }
}

void display_time(void)
{
  Serial.print(year());
  Serial.print("-");
  Serial.print(month());
  Serial.print("-");
  Serial.print(day());
  Serial.print(" at ");
  Serial.print(hour());
  Serial.print(":");
  Serial.print(minute());
  Serial.print(":");
  Serial.println(second());
}

/* ===CODE_STARTS_HERE========================================== */

void setup() {
  pinMode(PIN_LED, OUTPUT);     // initialize digital pin 22 as an output.(LED and power on for sensors via P mosfet)
  digitalWrite(PIN_LED, HIGH);  //sensors all off
  delay(200);
  Serial.begin(115200);


  DBG("Weather Station full Gateway 0 : Lora + Wifi");

  Serial.println(" ");
  Serial.println("*****************************************");
  Serial.print("CPU0 reset reason: ");
  print_reset_reason(rtc_get_reset_reason(0));
  Serial.println("*****************************************");

  if (resetWake)
  {
    Serial.print( "touch3 ") ;
    Serial.print( touchRead(T3));
    Serial.print ("/");
    Serial.println(threshold);
    if (touchRead(T9) < threshold) touch9detected = true; 
    if (touchRead(T3) < threshold) touch3detected = true; //detect touchpad for CONFIG_PIN
  }
  //Preferences
  preferences.begin("Rezodo", false);
  //preferences.clear();              // Remove all preferences under the opened namespace
  //preferences.remove("counter");   // remove the counter key only
  
  timeToSleep = preferences.getInt("timeToSleep", 4);
  ssid = preferences.getString("ssid", "");         // Get the ssid  value, if the key does not exist, return a default value of ""
  password = preferences.getString("password", "");


  //preferences.end();  // Close the Preferences

#if defined DEBUG_PREFS
  Serial.println("_______prefs after boot_______");
  Serial.print("timeToSleep : ");
  Serial.println(timeToSleep);
  Serial.println("______________________________");
#endif

 // initialize the RTC
  rtc.init();					   
  //  //connect to WiFi
  // The extra parameters to be configured (can be either global or just in the setup)
  // After connecting, parameter.getValue() will get you the configured value
  // id/name placeholder/prompt default length
  WiFiManagerParameter custom_ascMargin("margin", "sleep time", ascMargin, 3);
  //WiFiManagerParameter custom_TZ("time zone", "time zone", TZ, 2); //no need... done with call to OpenWeather Map

  //WiFiManager
  //Local intialization. Once its business is done, there is no need to keep it around
  WiFiManager wifiManager;

  //set config save notify callback
  wifiManager.setSaveConfigCallback(saveConfigCallback);

  //add all your parameters here
  const char* z2 = "<p>sleep (min)</p>";
  WiFiManagerParameter custom_text2(z2);
  wifiManager.addParameter(&custom_ascMargin);
  wifiManager.addParameter(&custom_text2);
  //  const char* z3 = "<p>time zone</p>";      //no need... done with call to OpenWeather Map
  //  WiFiManagerParameter custom_text3(z3);
  //  wifiManager.addParameter(&custom_TZ);
  //  wifiManager.addParameter(&custom_text3);

  //reset settings - for testing
  //wifiManager.resetSettings();

  //sets timeout until configuration portal gets turned off
  //useful to make it all retry or go to sleep
  //in seconds
  wifiManager.setTimeout(300);

  if (resetWake && touch3detected)  //then launch WifiManager
  {
    //fetches ssid and pass and tries to connect
    //if it does not connect it starts an access point with the specified name
    //here  "AutoConnectAP"
    //and goes into a blocking loop awaiting configuration
    if (!wifiManager.startConfigPortal("JP Rezodo"))
    {
      Serial.println("failed to connect and hit timeout");
      delay(3000);
      //reset and try again, or maybe put it to deep sleep
      ESP.restart();
      delay(5000);
    }
  }
  delay(2000);
  //  //save the custom WifiManager's parameters if needed
  if (shouldSaveConfig)
  {
    Serial.println("saving Wifi credentials ");
    //read updated parameters
    // strcpy(OWM_APIkey, custom_OWM_APIkey.getValue());
    strcpy(ascMargin, custom_ascMargin.getValue());
    timeToSleep = atoi(ascMargin);
    preferences.putInt("timeToSleep", timeToSleep);                 // reset time to sleep to 10 minutes (usefull after night)
    Serial.print("time to sleep : ");
    Serial.println(timeToSleep);
if (timeToSleep < 2) timeToSleep = 2;
    preferences.putString("password", WiFi.psk());
    preferences.putString("ssid", WiFi.SSID());
    delay(2000);
    ESP.restart();
    delay(5000);
  }




  //connect to WiFi
  WiFi.begin(ssid.c_str(), password.c_str());
  long start = millis();
  hasWifiCredentials = false;
  hasNtpTime = false;
  while ((WiFi.status() != WL_CONNECTED) && (millis() - start < 10000))
  {
    delay(500);
    Serial.print(".");
  }
  if (WiFi.status() == WL_CONNECTED) hasWifiCredentials = true;

  //if you get here you may be connected to the WiFi
  Serial.print("connected to Wifi: ");
  Serial.println(hasWifiCredentials);



  if (hasWifiCredentials)
  {
    TelnetStream.begin(); //used to debug over telnet
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());

    //init and get the time
    Serial.println("trying to get time 1");
    configTime(timeZone * 3600, dst * 0, "pool.ntp.org");
    printLocalTime();

    //init and get the time
    Serial.println("trying to get time 2");   //call it twice to have a well synchronized time on soft reset... Why ? bex=caus eit works...
    delay(2000);
    configTime(timeZone * 3600, dst * 0, "pool.ntp.org");
    printLocalTime();

    //disconnect WiFi as it's no longer needed
    //  WiFi.disconnect(true);
    //  WiFi.mode(WIFI_OFF);


    if (hasNtpTime)   //set the time with NTP info
    {
      time_t now;
      struct tm * timeinfo;
      time(&now);
      timeinfo = localtime(&now);

      years = timeinfo->tm_year + 1900;   //https://mikaelpatel.github.io/Arduino-RTC/d8/d5a/structtm.html
      months = timeinfo->tm_mon + 1;
      days = timeinfo->tm_mday;
      hours = timeinfo->tm_hour - timeZone;
      minutes = timeinfo->tm_min;
      seconds = timeinfo->tm_sec;

      //set ESP32 time manually (hr, min, sec, day, mo, yr)
      setTime(hours, minutes, seconds, days, months, years);
      Serial.print("time after ntp: ");
      display_time();

      struct timeval current_time;        //get ESP32 RTC time and save it
      gettimeofday(&current_time, NULL);
      tvsec  = current_time.tv_sec ;      //seconds since reboot
      hasRtcTime = true;                  //now ESP32 RTC time is also initialized

      //            //set DS1302 RTC time
      Ds1302::DateTime dt;
      dt.year = year() % 2000;
      dt.month = month();
      dt.day = day();
      dt.hour = hour();
      dt.minute = minute();
      dt.second = second();
      //dt.dow = weekday() - 1; // Day of the week (1-7), Sunday is day 1
      if (rtc.isHalted()) Serial.println("RTC is halted...");
      rtc.setDateTime(&dt);
    }
  }

  // test if clock is halted and set a date-time (see example 2) to start it
  if (rtc.isHalted())
  {
    DBG("RTC is halted...");
    if (hasRtcTime)
    {
      Serial.print("use time from ESP32 RTC: ");
      struct timeval current_time;
      gettimeofday(&current_time, NULL);
      // Serial.printf("seconds : %ld\nmicro seconds : %ld", current_time.tv_sec, current_time.tv_usec);
      //Serial.printf("seconds stored : %ld\nnow seconds : %ld\n", tvsec, current_time.tv_sec);
      int sec  = seconds - tvsec + current_time.tv_sec ;
      sec = hours * 3600 + minutes * 60 + sec;
      int ss = sec % 60;
      sec = sec / 60;
      int mm = sec % 60;
      sec = sec / 60;
      int hh = sec % 24;
      int dd = days + sec / 24;
      //set time manually (hr, min, sec, day, mo, yr)
      setTime(hh, mm, ss, dd, months, years);
      display_time();
    }
  }
  else
  {
    // get the current time
    Ds1302::DateTime now;
    rtc.getDateTime(&now);
    years = now.year + 2000;
    months = now.month;
    days = now.day;
    hours = now.hour;
    minutes = now.minute ;
    seconds = now.second;
    setTime(hours, minutes, seconds, days, months, years); //set ESP32 time manually
    DBG("use time from DS1302 RTC : ");
    display_time();
    struct timeval current_time;       //get ESP32 RTC time and save it
    gettimeofday(&current_time, NULL);
    tvsec  = current_time.tv_sec ;      //seconds since reboot (now stored into RTC RAM
    hasRtcTime = true;                  //now ESP32 RTC time is also initialized
  }

  // Calculate the sunset and sunrise
  calcSunriseSunset(year(), month(), day(), latitude, longitude, transit, sunrise, sunset);
  char str[6];
  Serial.print("Sunrise : ");
  Serial.println(sunrise);
  Serial.print("Sunset : ");
  Serial.println(sunset);

  float hourDec = hour() + float(minute()) / 60. + float(second()) / 3600.;
  Serial.print("current hour : ");
  Serial.println(hourDec);

  if ((hourDec < sunrise) || (hourDec > sunset))  //night sleeping time
  {
    timeToSleep = 60;
  }
  delay(1000);
 
  //FDRS
  for (int i = nbGtw; i < 1; i--) gtwHasSentStatus[i] = false;
  beginFDRS();


  timeOut = millis();     //arm software watchdog
  thingspeakTimeOut = millis();
}

void printLocalTime() //check if ntp time is acquired and print it
{
  struct tm timeinfo;
  hasNtpTime = true;
  if (!getLocalTime(&timeinfo)) {
    Serial.println("Failed to obtain time");
    hasNtpTime = false;
    return;
  }
  Serial.println(&timeinfo, "%A, %B %d %Y %H:%M:%S"); //https://www.ibm.com/docs/en/workload-automation/9.5.0?topic=troubleshooting-date-time-format-reference-strftime
}

void loop() {
  loopFDRS();
  if (myCtrl > 0) //we have received at least a control message for this gateway
  {
    for (int i = 0 ; i < myCtrl; i++) //for each control message
    {
      int id = myCtrlData[i].id >> 8;                 //extract the GTW id
      int index = myCtrlData[i].id & 0xFF;            //extract the sensor index

      sensorValues[id][index][0] = myCtrlData[i].d;   //extract the data field
      myCtrlData[i].id = 0x8000;                      //eat the message

      timeOut = millis();   //reset the timeout
      sendTimeTimeOut = 7000;
      if (index == 0) //if it is a sensor value message
      {
        Serial.print("has received sensors for GTW");
        Serial.print(id);

        Serial.print("   data ");
        Serial.println(sensorValues[id][index][0]);
#ifdef DEBUG_TELNET
        TelnetStream.print("has received sensors for GTW");
        TelnetStream.println(id);
#endif
        gtwHasSentStatus[id] = true;
        getNextGtw();
      }
    }
    myCtrl = 0;                                       //Ctrl message has been read, clear it
  }


  if (((millis() - timeOut) > sendTimeTimeOut) || (gtwStatus == sendingTime))
  {
    Serial.print("send time timeout with status ");
    Serial.println(gtwStatus);
    timeOut = millis();
    switch (gtwStatus)    //enum {idleStatus, sendingTime, waitingSensor, sleeping};
    {
      case idleStatus:            //at startup wait for the first timeout to occur before sending time
        gtwStatus = sendingTime;
        break;
      case sendingTime:
        getNextGtw();         //will send time to current GTW
        break;
      case waitingSensor:                      //bad situation where the current GTW does not send its sensor values (timeout)
        Serial.println("timeout waitingSensor");
        nbSent++;
        getNextGtw();
        //beginFDRS();  //test
        break;
      case sleeping:
        break;
      default:
        break;
    }
  }
  if (nbSent > (2 * nbGtw + 2))
  {
    Serial.println("escape from eternal lock...");
	timeToSleep = 2; //reset to minimal value				
    gotoSleep(); //escape from eternal lock... will allow 2 retries per GTW +2
  }

  if ((((millis() - timeOut) > sendTimeTimeOut) && (hasRtcTime)) || (gtwStatus == sleeping))  //if (hasReceivedSensors) ThingSpeakPost();
  {
    hasReceivedSensors = true;
    for (int i = 1; i < nbGtw + 1; i++) hasReceivedSensors = hasReceivedSensors & gtwHasSentStatus[i];
    if (hasReceivedSensors)
    {
      ThingSpeakPost();
#ifdef DEBUG_TELNET
      TelnetStream.print("temperature = ");
      TelnetStream.print(sensorValues[1][0][0]);
      TelnetStream.print(" °C   humidity = ");
      TelnetStream.print(sensorValues[1][1][0]);
      TelnetStream.print(" %   pressure = ");
      TelnetStream.print( sensorValues[1][2][0]);
      TelnetStream.println(" hPa");

      TelnetStream.print("rot speed = ");
      TelnetStream.print(sensorValues[1][3][0]);
      TelnetStream.print(" rpm   wind direction = ");
      TelnetStream.println(sensorValues[1][4][0]);

      TelnetStream.print("rain weight = ");
      TelnetStream.print(sensorValues[1][5][0]);
      TelnetStream.println(" g");
#endif
      delay(1000);
      gotoSleep();
    }
    else            //problem when receiving sensors
    {
      //      Serial.print("timeout but all sensors not received...");
      //      Serial.println(sendTimeTimeOut);
      //      Serial.print("gtwStatus ");
      //      Serial.println(gtwStatus);
      //      Serial.print("for each GTW: ");
      //      for (int i = 1; i < nbGtw + 1; i++) Serial.print (gtwHasSentStatus[i]);
      //      Serial.println(" ");
      Serial.println("**** no contact with GTW: try again"); //try again
      Serial.println("==> restart REZODO Lora gateway");														
#ifdef DEBUG_TELNET
      TelnetStream.println("**** no contact with GTW: try again");
#endif
      delay(100);
      beginFDRS();				 
      gtwStatus = idleStatus;
      timeOut = millis();
      nbSent++;
    }
  }
#ifdef DEBUG_TELNET
  if ((((millis() - telnetTimeOut) > 3000)) && hasWifiCredentials)  //debug with telnet (Termius on Android port 23)
  {
    telnetTimeOut = millis();
    TelnetStream.println(".");
  }
#endif

  //reboot @3 o'clock
  //if ((hours == 03) && (minutes == 0) && (seconds == 0)) ESP.restart();
}

void getNextGtw(void)
{
  gtwStatus = sleeping;
  sendTimeTimeOut = 12000;
  timeOut = millis();
  for (int i = nbGtw; i > 0; i--)
  {
    if (gtwHasSentStatus[i] == false)
    {
      sendTimeTimeOut = 8000;
      sendTime(i);
      gtwStatus = waitingSensor;  // at least one GTW has not yet sent its sensors values
      break;
    }
  }
}

void sendTime(int GTW)
{
  uint16_t id;
  id = (GTW << 8) | 255;
  float myTime;
  myTime = hour() * 3600 + minute() * 60 + second();
  //loadFDRS(float data, uint8_t type, uint16_t id);

  loadFDRS(myTime, timeToSleep, id); //will send time and timeToSleep (id 255) using INTERNAL_ACT event.
  sendFDRS();
}

void ThingSpeakPost(void)
{
    Serial.print("temperature = ");
      Serial.print(sensorValues[1][0][0]);
      Serial.print(" °C   humidity = ");
      Serial.print(sensorValues[1][1][0]);
      Serial.print(" %   pressure = ");
      Serial.print( sensorValues[1][2][0]);
      Serial.println(" hPa");

      Serial.print("rot speed = ");
      Serial.print(sensorValues[1][3][0]);
      Serial.print(" km/h   wind direction = ");
      Serial.println(sensorValues[1][4][0]);

      Serial.print("rain weight = ");
      Serial.print(sensorValues[1][5][0]);
      Serial.println(" g");
  WiFiClient client;
  if (!client.connect(thingserver, 80))
  {
    Serial.println("Connection failed");
    client.stop();
    return;
  }
  else
  {
    // Create data string to send to ThingSpeak.
    float scaledValue = mapf(sensorValues[2][0][0], 50, 370, 100, 0); //remap values from 100 to 0
    String data =  "field1=" + String(sensorValues[1][0][0]) + "&field2=" + String(sensorValues[2][0][0]) + "&field3=" + String(scaledValue) + "&field4=" + String(sensorValues[1][1][0]) + "&field5=" + String(sensorValues[1][2][0]) + "&field6=" + String(sensorValues[1][3][0]) + "&field7=" + String(sensorValues[1][4][0]) + "&field8=" + String(sensorValues[1][5][0])  ; //shows how to include additional field data in http post

    // POST data to ThingSpeak.
    if (client.connect(thingserver, 80)) {
      client.println("POST /update HTTP/1.1");
      client.println("Host: api.thingspeak.com");
      client.println("Connection: close");
      client.println("User-Agent: ESP32WiFi/1.1");
      client.println("X-THINGSPEAKAPIKEY: " + writeAPIKey);
      client.println("Content-Type: application/x-www-form-urlencoded");
      client.print("Content-Length: ");
      client.print(data.length());
      client.print("\n\n");
      client.print(data);
      delay(300);
#ifdef DEBUG_TELNET
      TelnetStream.print("temperature = ");
      TelnetStream.print(sensorValues[1][0][0]);
      TelnetStream.print(" °C   humidity = ");
      TelnetStream.print(sensorValues[1][1][0]);
      TelnetStream.print(" %   pressure = ");
      TelnetStream.print( sensorValues[1][2][0]);
      TelnetStream.println(" hPa");

      TelnetStream.print("rot speed = ");
      TelnetStream.print(sensorValues[1][3][0]);
      TelnetStream.print(" km/h   wind direction = ");
      TelnetStream.println(sensorValues[1][4][0]);

      TelnetStream.print("rain weight = ");
      TelnetStream.print(sensorValues[1][5][0]);
      TelnetStream.println(" g");
#endif
    }
    else
    {
#ifdef DEBUG_THINGSPEAK
      Serial.println("==> ThingSpeak: failed to connect ");
#endif
    }
#ifdef DEBUG_THINGSPEAK
    // Just for testing, print out the message on serial port
    Serial.println("==> ThingSpeak POST message ");
    Serial.println(data);
#endif
    client.stop();
  }
}

void gotoSleep()
{
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
  btStop();
  adc_power_off();
  esp_wifi_stop();
  esp_bt_controller_disable();
  Serial.println("Entering DeepSleep");
  int mm = (int)(floor((minute() * 60 + second()) / (60 * timeToSleep)) * timeToSleep + timeToSleep);
  if (mm > 60) mm = 60;
  Serial.print ("next integer minutes " );
  Serial.println (mm);
#ifdef DEBUG_TELNET
  TelnetStream.println("Entering DeepSleep");
  TelnetStream.print ("next integer minutes " );
  TelnetStream.println(mm);
  delay(1000);
#endif
  long tt;                                                 //time to sleep
  tt = mm * 60 - (minute() * 60 + second()) + 10 ;         //10s margin to wake up after slaves and to take into account lora delay...
  display_time();
  esp_sleep_enable_timer_wakeup(tt * uS_TO_S_FACTOR);
  esp_deep_sleep_start();                                  //enter deep sleep mode
  delay(1000);
  abort();
}

float mapf(float value, float fromLow, float fromHigh, float toLow, float toHigh)
{
  float result;
  result = (value - fromLow) * (toHigh - toLow) / (fromHigh - fromLow) + toLow;
  return result;
}
