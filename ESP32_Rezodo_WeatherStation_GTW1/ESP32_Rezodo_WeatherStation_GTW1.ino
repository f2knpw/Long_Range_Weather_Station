#include <WiFi.h>
//#include <HTTPClient.h>
#include <WiFiClient.h>
#include <TelnetStream.h>

#define LED_PIN 22
#define PWR_PIN 2     //will  be used to power all the sensors when HIGH

//servo
// Values for TowerPro SG90 small servos; adjust if needed
#define COUNT_LOW 2200
#define COUNT_HIGH 7100
#define TIMER_WIDTH 16
#include "esp32-hal-ledc.h "
#define SERVO_PIN 33

//sensors selection
#define HAS_HX711       //uncomment for weighing rain gauge
#define HAS_ANEMOMETER  //uncomment for anemometer
#define HAS_AS5600      //uncomment for Wind Direction sensor
#define HAS_BME280      //uncomment for temperature + pressure + humidity BME280 sensor
#define HAS_DS18B20   //uncomment for DS18B20 temperature sensor
//#define HAS_DHT22     //uncomment for DTH22 temperature + humidity sensor
//#define ENABLE_WIFI   //uncomment to enable wifi debugging (avoid this when ESPNow in use)

float humidity = 0 ;
float temperature = 0 ;
float outsideTemperature = 0 ;
float pressure = 0 ;
float windAngle = 0;
float windSpeed = 0;
int smooth = 1000;        //acquire smooth*values for each ADC
float Vin = 0.;           //input Voltage (solar panel voltage)
int sensorsGetTime = 15000; //ms to acquire sensors in worst case

//DHT sensor (temp + humidity)
#define DHTPIN 5             // Digital pin connected to the DHT sensor 
#ifdef HAS_DHT22
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>
#define DHTTYPE    DHT22     // DHT 22 (AM2302) --> https://learn.adafruit.com/dht/overview
DHT dht(DHTPIN, DHTTYPE);
#endif

//temperature sensor
#define ONE_WIRE_BUS 13       // Data wire is plugged into pin 13 on the ESP32
#define TEMP_OFFSET -0.3
#ifdef HAS_DS18B20
#include <OneWire.h>
#include <DallasTemperature.h>
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature tempScale1(&oneWire); // Pass our oneWire reference to Dallas Temperature.
#endif

//scale
#define PIN_CLOCK  32        //output to generate clock on Hx711
#define PIN_DOUT   34        //input Dout from Hx711

long calibZero = 0;  //No load sensor Output
long calib = 130968;          //sensor output - calibZero for Weight calibration --> will be auto calibrated later
int calibWeight = 335;         //weight at which calinration is done --> expressed in gramsx10. eg 335 means 33.5g
float AverageWeight = 0;
float CurrentRawWeight = 0;
float RealTimeWeight = 0;
int iWeight;
float rainWeight;
float rain;
boolean hasEmptiedBucket = false;



#define FILTER_SAMPLES   50              // filterSamples should  be an odd number, no smaller than 3
#define REJECT_RATIO     25               //points to reject % left and right before averaging
float weightSmoothArray [FILTER_SAMPLES];   // array for holding raw sensor values for sensor1

//anemometer
#define HALL_OUT_PIN 4
int tops = 0;       //nb tops when anemometer rotates
long hallTimeout;   //to debounce
int anemometerMeasuringTime;

//WindDirection
#define SDA_PIN 16
#define SCL_PIN 17
#include "Wire.h"
#ifdef HAS_AS5600
#include "AS5600.h"

AS5600 as5600;   //  use default Wire
#endif

float calibAngle = 0; //raw value when pointing to North

//BME280 (temperature, pressure, humidity)
//uses the wire library already included
#ifdef HAS_BME280
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#define SEALEVELPRESSURE_HPA (1013.25)
Adafruit_BME280 bme; // I2C
#endif

#define VIN_PIN 36    //ADC pin for solar panel voltage measurement


//FDRS
#include "fdrs_gateway_config.h"
#include <fdrs_gateway.h>


extern DataReading myCtrlData[256];
extern uint8_t myCtrl;
//end FDRS

long timeOut = 0;
long telnetTimeOut ;
int sendStateTimeOut = 80000;
int counter;
int forceSleep = 0;
boolean hasReceivedCmd = false;
boolean hasReceivedTime = false;

enum {idle, waitingTime, sendingSensor, sleeping};        // 0: idle, 1: sendingTime, 2: waitingSensor, 3: sleeping
int GtwStatus = idle;

//float M1;                           // Moisture sensors 1 (note that M2 on the PCB is used to calibrate sensors)

#define uS_TO_S_FACTOR 1000000ULL  /* Conversion factor for micro seconds to seconds */
int timeToSleep;                    /* Time ESP32 will go to sleep (in seconds) */



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


String ssid = "";
String password = "";
boolean hasWifiCredentials = false;
boolean hasNtpTime = false;                 //UTC time not acquired from NTP
int timeZone = 0;   //set to UTC
const int dst = 0;

//these variable remain in RTC memory even in deep sleep or after software reset (https://github.com/espressif/esp-idf/issues/7718)(https://www.esp32.com/viewtopic.php?t=4931)
RTC_NOINIT_ATTR boolean hasRtcTime = false;   //UTC time not acquired from smartphone

RTC_NOINIT_ATTR int hours;
RTC_NOINIT_ATTR int seconds;
RTC_NOINIT_ATTR int tvsec;
RTC_NOINIT_ATTR int minutes;
RTC_NOINIT_ATTR int days;
RTC_NOINIT_ATTR int months;
RTC_NOINIT_ATTR int years;

//reseted after software reset
//RTC_DATA_ATTR boolean hasRtcTime = false;   //will only survice to deepsleep reset... not software reset
RTC_DATA_ATTR float previousRainWeight = 0;


//time
#include <TimeLib.h>


boolean touchWake = false;
boolean resetWake = false;
touch_pad_t touchPin;
int threshold = 45; //Threshold value for touchpads pins
bool touch9detected = false;  //touch9
bool touch3detected = false;  //touch3 used to calibrate sensors (hold it while reseting)
bool touch0detected = false;  //touch0 used to launch WifiManager (hold it while reseting)

//Preferences
#include <Preferences.h>
Preferences preferences;

#define DEBUG_WIFI   //debug Wifi 
#define DEBUG_SLEEP
#define DEBUG_UDP    //broadcast info over UDP
#define DEBUG_PREFS  //debug preferences
#define DEBUG_VIN
#define DEBUG
#define RAW_WEIGHT_DEBUG
#define PREFERENCES_DEBUG


//WifiManager
#ifdef ENABLE_WIFI
#include <DNSServer.h>
#include <WebServer.h>
#include <WiFiManager.h>          //https://github.com/tzapu/WiFiManager

//define your default values here, if there are different values in config.json, they are overwritten.
char ascMargin[3];  //should contain 2 char "20" margin in minutes
String strAscMargin; //same in String


//flag for saving data
bool shouldSaveConfig = false;

//callback notifying us of the need to save config
void saveConfigCallback ()
{
  Serial.println("Should save config");
  shouldSaveConfig = true;
}
#endif


//********************
//code starts here
//********************

#ifdef HAS_ANEMOMETER
void IRAM_ATTR hall_ISR()    //hall sensor interrupt routine
{
  if ((millis() - hallTimeout) > 10)
  {
    hallTimeout = millis();
    tops++;
    //Serial.println( tops);  //should comment this line to avoid crashes
  }
}
#endif

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



void setup() {
  pinMode(LED_PIN, OUTPUT);      // initialize digital pin 22 as an output.
  digitalWrite(LED_PIN, HIGH);
  pinMode(PWR_PIN, OUTPUT);      // initialize digital pin 0 as an output
  digitalWrite(PWR_PIN, LOW);   //all sensors are Off
  sensorsGetTime = millis();

  Serial.begin(115200);
  Serial.println(" ");
  Serial.println("*****************************************");
  Serial.print("CPU0 reset reason: ");
  print_reset_reason(rtc_get_reset_reason(0));
  Serial.println("*****************************************");

  if (resetWake)
  {
    Serial.print("touch3 : ");
    Serial.print(touchRead(T3));
    Serial.print("/");
    Serial.println(threshold);
    if (touchRead(T9) < threshold) touch9detected = true; //detect touchpad for T9
    if (touchRead(T3) < threshold) touch3detected = true; //detect touchpad for CONFIG_PIN
    //if (touchRead(T2) < threshold) touch0detected = true; //detect touchpad for wifiManager
    if (touchRead(T0) < threshold) touch0detected = true; //detect touchpad for wifiManager
  }

  //Preferences
  preferences.begin("Rezodo", false);
  //preferences.clear();              // Remove all preferences under the opened namespace
  //preferences.remove("counter");   // remove the counter key only
  calibZero = preferences.getLong("calibZero", 0);
  calib = preferences.getLong("calib", 0);
  calibAngle = preferences.getFloat("calibAngle", 0);   //raw value of AS5600 when pointing to North

  timeToSleep = preferences.getInt("timeToSleep", 4);
  ssid = preferences.getString("ssid", "");         // Get the ssid  value, if the key does not exist, return a default value of ""
  password = preferences.getString("password", "");

#ifdef PREFERENCES_DEBUG
  Serial.println("_________________");
  Serial.print("calib0 HX711 : ");
  Serial.println(calibZero);
  Serial.print("calib HX711 : ");
  Serial.println(calib);
  Serial.print("timeToSleep : ");
  Serial.println(timeToSleep);
  Serial.println("_________________");
#endif

  //enable deepsleep for ESP32
  //  esp_sleep_enable_ext1_wakeup(PIR_PIN_BITMASK, ESP_EXT1_WAKEUP_ANY_HIGH); //this will be the code to enter deep sleep and wakeup with pin GPIO2 high
  esp_sleep_enable_timer_wakeup(timeToSleep * uS_TO_S_FACTOR);                 //allow timer deepsleep
  //esp_sleep_enable_touchpad_wakeup();                                           //allow to wake up with touchpads

  //connect to WiFi
#ifdef ENABLE_WIFI
  // The extra parameters to be configured (can be either global or just in the setup)
  // After connecting, parameter.getValue() will get you the configured value
  // id/name placeholder/prompt default length
  //WiFiManagerParameter custom_ascMargin("margin", "margin", ascMargin, 3);


  //WiFiManager
  //Local intialization. Once its business is done, there is no need to keep it around
  WiFiManager wifiManager;

  //set config save notify callback
  wifiManager.setSaveConfigCallback(saveConfigCallback);

  //add all your parameters here
  //  const char* z2 = "<p>calib (gx10)</p>";
  //  WiFiManagerParameter custom_text2(z2);
  //  wifiManager.addParameter(&custom_ascMargin);
  //  wifiManager.addParameter(&custom_text2);

  //reset settings - for testing
  //wifiManager.resetSettings();

  //sets timeout until configuration portal gets turned off
  //useful to make it all retry or go to sleep
  //in seconds
  wifiManager.setTimeout(300);


  if (resetWake && touch0detected)  //then launch WifiManager
  {
    //fetches ssid and pass and tries to connect
    //if it does not connect it starts an access point with the specified name
    //here  "AutoConnectAP"
    //and goes into a blocking loop awaiting configuration
    if (!wifiManager.startConfigPortal("JP WeatherStation"))
    {
      Serial.println("failed to connect and hit timeout");
      delay(3000);
      //reset and try again, or maybe put it to deep sleep
      ESP.restart();
      delay(5000);
    }
  }

  //  //save the custom WifiManager's parameters if needed
  if (shouldSaveConfig)
  {
    Serial.println("saving Wifi credentials ");
    //read updated parameters
    //    strcpy(ascMargin, custom_ascMargin.getValue());
    //    calibWeight = atoi(ascMargin);
    //    preferences.putInt("calibWeight", calibWeight);
    //Serial.println(ascMargin);
    preferences.putString("password", WiFi.psk());
    preferences.putString("ssid", WiFi.SSID());
    ESP.restart();
    delay(5000);
  }
#endif // ENABLE_WIFI


#ifdef DEBUG
  Serial.println(" == > acquiring sensors");
#endif
  digitalWrite(LED_PIN, HIGH);  //led off to save juice
  digitalWrite(PWR_PIN, HIGH);  //all sensors are On

  //panel voltage
  //ADC
  //analogSetClockDiv(255);
  //analogReadResolution(12);             // Sets the sample bits and read resolution, default is 12-bit (0 - 4095), range is 9 - 12 bits
  analogSetWidth(12);                   // Sets the sample bits and read resolution, default is 12-bit (0 - 4095), range is 9 - 12 bits
  analogSetAttenuation(ADC_11db);        //Sets the input attenuation for ALL ADC inputs, default is ADC_11db, range is ADC_0db=0, ADC_2_5db=1, ADC_6db=2, ADC_11db=3

  for (int i = 0; i < smooth; i++) Vin += analogRead(VIN_PIN);
  Vin = Vin / smooth ;
#ifdef DEBUG_VIN
  Serial.print("Vin ");
  Serial.print(Vin);
  Serial.print(" / ");
#endif
  Vin = volts(Vin);
#ifdef DEBUG_VIN
  Serial.println(Vin);
#endif

  //anemometer
#ifdef HAS_ANEMOMETER
#ifdef DEBUG
  Serial.println("anemometer enabled");
#endif
  pinMode(HALL_OUT_PIN, INPUT_PULLUP);
  attachInterrupt(HALL_OUT_PIN, hall_ISR, FALLING);   //will count tops on Anemometer hall Sensor
  hallTimeout = millis();
  tops = 0;
  anemometerMeasuringTime = millis();
#endif

  //WindDirection
  Wire.begin(SDA_PIN, SCL_PIN);
#ifdef HAS_AS5600

  //  as5600.begin(4);  //  set direction pin.
  //  as5600.setDirection(AS5600_CLOCK_WISE);  // default, just be explicit.
  int b = as5600.isConnected();
  if (b)  //if connection OK
  {
    Serial.print("wind Direction: ");
    windAngle = getWindAngle();
    Serial.println(windAngle);
  }

#endif

  //BME280
#ifdef HAS_BME280

  unsigned status;

  // default settings
  status = bme.begin(0x76);
  // You can also pass in a Wire library object like &Wire2
  // status = bme.begin(0x76, &Wire2)
  if (!status)
  {
    Serial.println("Could not find a valid BME280 sensor, check wiring, address, sensor ID!");
    Serial.print("SensorID was: 0x"); Serial.println(bme.sensorID(), 16);
    Serial.print("        ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n");
    Serial.print("   ID of 0x56-0x58 represents a BMP 280,\n");
    Serial.print("        ID of 0x60 represents a BME 280.\n");
    Serial.print("        ID of 0x61 represents a BME 680.\n");
    while (1) delay(10);
  }
  else
  {
    Serial.println("BME280 enabled");
    Serial.print("Temperature = ");
    temperature = bme.readTemperature();
    Serial.print(temperature);
    Serial.println(" °C");

    Serial.print("Pressure = ");
    pressure = bme.readPressure() / 100.0F;
    Serial.print(pressure);
    Serial.println(" hPa");

    //    Serial.print("Approx. Altitude = ");
    //    Serial.print(bme.readAltitude(SEALEVELPRESSURE_HPA));
    //    Serial.println(" m");

    Serial.print("Humidity = ");
    humidity = bme.readHumidity();
    Serial.print(humidity);
    Serial.println(" %");
  }
#endif

  //outside temperature sensor
#ifdef HAS_DS18B20
  tempScale1.begin(); // IC Default 9 bit. If you have troubles consider upping it 12. Ups the delay giving the IC more time to process the temperature measurement

  outsideTemperature = 0;
  int i;
  int kk;
  for (i = 0; i < 2; i++) {            //read twice to warm up
    tempScale1.requestTemperatures();   //needed as first reading may be stuck to 25°
    kk = tempScale1.getTempCByIndex(0);
  }
  for (i = 0; i < 20; i++) {
    tempScale1.requestTemperatures();
    outsideTemperature += tempScale1.getTempCByIndex(0);
  }
  outsideTemperature /= i;
  outsideTemperature += TEMP_OFFSET; //offset measured...
#ifdef DEBUG
  Serial.print("outside temperature = ");
  Serial.print(outsideTemperature);
  Serial.println(" °C");
#endif
#endif

  //scale init
#ifdef HAS_HX711
  pinMode(PIN_CLOCK, OUTPUT); // initialize digital pin 4 as an output.(clock)
  digitalWrite(PIN_CLOCK, HIGH);
  delayMicroseconds(100);   //be sure to go into sleep mode if > 60µs
  digitalWrite(PIN_CLOCK, LOW);     //exit sleep mode*/
  pinMode(PIN_DOUT, INPUT);  // initialize digital pin 5 as an input.(data Out)

  GetRawWeight();       //HX711 will sleep after weight acquisition
  if (touch3detected)
  {
    Serial.print ("calibration HX711... ");
    calib = CurrentRawWeight;
    Serial.println(CurrentRawWeight);
    preferences.putLong("calib", calib);
    emptyBucket();
  }

  AverageWeight = (calibZero - CurrentRawWeight) * calibWeight / (calibZero - calib);
  rainWeight = AverageWeight / 10;
  if ((rainWeight - previousRainWeight)  < -.2 ) //something not normal bucket has lost more than 0.2g !
  {
#ifdef DEBUG
    Serial.print ("bucket has lost more than 0.2g... ");
    Serial.println(rainWeight);
#endif
    emptyBucket();
    rain = 0;
  }
  else if (resetWake)
  {
#ifdef DEBUG
    Serial.println ("manual reset... ");
#endif
    emptyBucket();              //we have lost previousRainWeight... must empty the bucket
    rain = 0;
  }
  else                                  //possibly some rain
  {
    rain = rainWeight - previousRainWeight;
    if (rain < 0) rain = 0;
    if (rainWeight > 50)
    {
#ifdef DEBUG
      Serial.print ("bucket full... ");
      Serial.println(rainWeight);
#endif
      emptyBucket();              //bucket is almost full
    }
    else if ((hours == 12) && (minutes == 0))
    {
#ifdef DEBUG
      Serial.print ("it's noon... ");
      Serial.println(rainWeight);
#endif
      emptyBucket();
    }
  }
#ifdef DEBUG
  Serial.print("rain value : ");
  Serial.println(rain);
#endif
  previousRainWeight = rainWeight;
  digitalWrite(PIN_CLOCK, HIGH);  //go to sleep mode
#endif  //HAS_HX711

  //DHT22
#ifdef HAS_DHT22
  dht.begin();      // Reading temperature or humidity takes about 250 milliseconds!
  humidity = dht.readHumidity();
  temperature = dht.readTemperature();

  if (isnan(humidity) || isnan(temperature)) // Check if any reads failed and exit early (to try again).
  {
    Serial.println(F("Failed to read from DHT sensor!"));
  }
  else
  {
    Serial.print(F("Humidity : "));
    Serial.print(humidity);
    Serial.print(F(" %  Temperature : "));
    Serial.print(temperature);
    Serial.println(F("°C "));
  }
#endif  //HAS_DHT22

  //
  //
  //  //soil moisture probes initialization
  //  Serial.println("measuring moisture");
  //  touch_pad_init();
  //  touch_pad_set_voltage(TOUCH_HVOLT_2V7, TOUCH_LVOLT_0V5, TOUCH_HVOLT_ATTEN_1V);
  //  touch_pad_config(TOUCH_PAD_NUM1, 0); //T1
  //
  //  //M1 read touch output
  //  uint16_t output;
  //  touch_pad_read(TOUCH_PAD_NUM1, &output);  //T1 or M1
  //  M1 = float(output);


  Serial.println("==> end acquisition sensors");

  // initialize the RTC
  rtc.init();

#ifdef ENABLE_WIFI
  Serial.println("==> connect to Wifi");
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

    Serial.println("Ready");
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

      // set DS1302 RTC time
      Ds1302::DateTime dt;
      dt.year = years  % 2000;
      dt.month = months;
      dt.day = days;
      dt.hour = hours;
      dt.minute = minutes;
      dt.second = seconds;
      if (rtc.isHalted()) DBG("RTC is halted...");
      rtc.setDateTime(&dt);
    }
  }
#else
  //disconnect WiFi as it's not needed
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
#endif //ENABLE_WIFI


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

  //FDRS
  timeOut = millis();
  delay(10);                            //to avoid loosing serialPrint...

  beginFDRS();
  DBG("==> start Weather Station full GTW1");

  if (hasEmptiedBucket == false) delay(14000);  //14s is time to empty bucket
  sensorsGetTime = millis() - sensorsGetTime; //now we know how long it takes to acquire sensors
  sensorsGetTime = min(sensorsGetTime, 30000);
  Serial.print("time spent into setup : sensorsGetTime (ms) ");
  Serial.println(sensorsGetTime);
}

#ifdef HAS_HX711
void emptyBucket(void)
{

  //servo to empty the bucket
  ledcSetup(1, 50, TIMER_WIDTH); // channel 1, 50 Hz, 16-bit width
  ledcAttachPin(SERVO_PIN, 1);   // SERVO_PIN assigned to channel 1
  //move servo
  Serial.println("empty bucket");
  ledcWrite(1, COUNT_LOW);
  delay(2000);
  ledcWrite(1, COUNT_HIGH);
  delay(1000);
  ledcDetachPin(SERVO_PIN);
  pinMode(SERVO_PIN, INPUT);
  delay(1000);
  GetRawWeight();                 //recalibrate zero value after emptying the bucket
  calibZero = CurrentRawWeight;
  preferences.putLong("calibZero", calibZero);
  Serial.print("calibZero = ");
  Serial.println(calibZero);
  previousRainWeight = 0;
  hasEmptiedBucket = true;
}
#endif

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

void loop()
{
  loopFDRS();
  if (myCtrl > 0) //we have received at least a control message for this gateway
  {
    timeOut = millis();
    for (int i = 0 ; i < myCtrl; i++) //for each control message
    {
      int id = myCtrlData[i].id >> 8;                 //extract the GTW id
      int index = myCtrlData[i].id & 0xFF;            //extract the sensor index
      myCtrlData[i].id = 0x8000;                      //reset the id to unused value (eat the message)
      if (id == UNIT_MAC)
      {
        switch (index)
        {
          case 1:  //irrigation coil
          case 2:
          case 3:
          case 4:
          case 5:
          case 6:
            if (myCtrlData[i].t == 1) //activate coil
            {
              GtwStatus = waitingTime;
              hasReceivedCmd = true;        //this will disable security shutdown during deepsleep
              Serial.print ("hasReceivedCmd ");
              Serial.print (index);
            }
            break;

          case 0xFF:  //time synchro
            sendStateTimeOut = 15000;  //decrease the timeout value
            timeOut = millis();   //reset the timeout
            GtwStatus = sendingSensor;  //ok to send sensor values
            hasReceivedTime = true;        //this will disable security shutdown during deepsleep
            timeToSleep =   myCtrlData[i].t ;
            preferences.putInt("timeToSleep", timeToSleep);                 // set time to sleep
            Serial.print ("timeToSleep set to ");
            Serial.print(timeToSleep);
            Serial.println (" minutes");

            hours =  floor( myCtrlData[i].d / 3600);
            minutes = floor((myCtrlData[i].d - hours * 3600) / 60);
            seconds = myCtrlData[i].d - hours * 3600 - minutes * 60;
            days = 1;
            months = 6;
            years = 2023;

            //set ESP32 time manually (hr, min, sec, day, mo, yr)
            setTime(hours, minutes, seconds, days, months, years);
            Serial.print("time from GTW0: ");
            display_time();

            struct timeval current_time;        //get ESP32 RTC time and save it
            gettimeofday(&current_time, NULL);
            tvsec  = current_time.tv_sec ;      //seconds since reboot
            hasRtcTime = true;                  //now ESP32 RTC time is also initialized

            //            //set DS1302 RTC time
            Ds1302::DateTime dt;
            dt.year = 23;
            dt.month = 6;
            dt.day = 1;
            dt.hour = hours;
            dt.minute = minutes;
            dt.second = seconds;
            if (rtc.isHalted()) DBG("RTC is halted...");
            rtc.setDateTime(&dt);
#ifdef HAS_ANEMOMETER
            windSpeed = getAnemometer();
#endif
            sendSensorsValues();              //now answer with the sensors values
            break;
          default:
            //do nothing
            break;
        }
      }
    }
    myCtrl = 0;                                       //Ctrl message has been read, clear it
  }

  if ((((millis() - telnetTimeOut) > 3000)) && hasWifiCredentials)  //debug with telnet (Termius on Android port 23)
  {
    telnetTimeOut = millis();
    if (touch3detected)
    {
      TelnetStream.println("M3 was touched ==> sensors calibration");
      Serial.println("M3 was touched ==> sensors calibration");
    }
#ifdef HAS_HX711
    TelnetStream.print("rain weight: ");
    TelnetStream.print(rainWeight);
    TelnetStream.println(" g");
#endif
#ifdef HAS_ANEMOMETER
    TelnetStream.print("hall sensor: ");
    TelnetStream.print(digitalRead(HALL_OUT_PIN));
    TelnetStream.print(" rot speed (km/h): ");
    TelnetStream.println(windSpeed);
#endif
#ifdef HAS_AS5600
    TelnetStream.print("wind direction: ");
    TelnetStream.print(windAngle);
    TelnetStream.print("   now: ");
    TelnetStream.println( getWindAngle());
#endif
#ifdef HAS_BME280
    TelnetStream.print("temperature = ");
    TelnetStream.print(temperature);

    TelnetStream.print(" °C   pressure = ");
    TelnetStream.print( pressure);
    TelnetStream.print(" hPa   humidity = ");
    TelnetStream.print(humidity);
    TelnetStream.println(" %");
#endif
#ifdef HAS_DS18B20
    TelnetStream.print("outside temperature = ");
    TelnetStream.print(outsideTemperature);
    TelnetStream.println(" °C");
#endif

    TelnetStream.print("Vin = ");
    TelnetStream.print(Vin);
    TelnetStream.println( "V");
  }



  if (((millis() - timeOut) > (sendStateTimeOut - 1000)) && (hasRtcTime)) gotoSleep(); //if no RTC time then wait for Time sync message

  if (millis() > 100000)
  {
    Serial.println ("===> wake up too long...");
    gotoSleep();
  }
}

#ifdef HAS_AS5600
float getWindAngle(void)
{
  float angleValue = as5600.rawAngle() * AS5600_RAW_TO_DEGREES;
  if (touch3detected) //calibrate sensors
  {
    Serial.println ("calibrate anemometer");
    calibAngle = angleValue;
    preferences.putFloat("calibAngle", calibAngle);
  }
  else
  {
    angleValue = as5600.rawAngle() * AS5600_RAW_TO_DEGREES - calibAngle;
    angleValue += 360;
    if (angleValue > 360) angleValue -= 360;
  }
  angleValue = mapf(angleValue, 0., 360., 360., 0.);
  return angleValue;
}
#endif


#ifdef HAS_ANEMOMETER
float getAnemometer(void)
{
  //float anemometerValue = ((float)tops ) ;
  //float anemometerValue = ((float)tops *60 * 1000.) / (millis() - anemometerMeasuringTime); //expressed in RPM
  float anemometerValue = (((float)tops * 60 * 1000.) / (millis() - anemometerMeasuringTime)) * 0.105; //expressed in km/h after calibration : https://hackaday.io/project/190577-rezodo-long-range-irrigation-and-weather-station/log/218476-calibration
  tops = 0;
  anemometerMeasuringTime = millis();
  return anemometerValue;
}
#endif

void sendSensorsValues(void)
{
  timeOut = millis();
  //loadFDRS(float data, uint8_t type, uint16_t id);
  uint16_t id;
  id = (UNIT_MAC << 8) | 0;
  loadFDRS(outsideTemperature, TEMP_T, id); //will send back these data readings using INTERNAL_ACT event. Id 0x100 = GTW1 sensor0
  id = (UNIT_MAC << 8) | 1;
  loadFDRS(humidity, HUMIDITY_T, id); //will send back these data readings using INTERNAL_ACT event. Id 0x101 = GTW1 sensor1
  id = (UNIT_MAC << 8) | 2;
  loadFDRS(pressure, PRESSURE_T, id); //will send back these data readings using INTERNAL_ACT event. Id 0x102 = GTW1 sensor2
  id = (UNIT_MAC << 8) | 3;
  //loadFDRS(windSpeed, WINDSPD_T, id); //will send back these data readings using INTERNAL_ACT event. Id 0x103 = GTW1 sensor3
  loadFDRS(Vin, WINDSPD_T, id); //will send back these data readings using INTERNAL_ACT event. Id 0x103 = GTW1 sensor3
  id = (UNIT_MAC << 8) | 4;
  loadFDRS(windAngle, WINDHDG_T, id); //will send back these data readings using INTERNAL_ACT event. Id 0x104 = GTW1 sensor4
  id = (UNIT_MAC << 8) | 5;
  loadFDRS(rain / 10, RAINFALL_T, id); //will send back these data readings using INTERNAL_ACT event. Id 0x105 = GTW1 sensor5 //10g = 1mm of rain ==> needs to divide by 10 !
  sendFDRS();
  forceSleep ++;
  if (forceSleep > 3) gotoSleep();  //escape from eternal lock...
}


#ifdef HAS_HX711
void GetRawWeight(void)
{
  digitalWrite(PIN_CLOCK, HIGH);
  delayMicroseconds(100);   //be sure to go into sleep mode if > 60µs
  digitalWrite(PIN_CLOCK, LOW);     //exit sleep mode*/
  pinMode(PIN_DOUT, INPUT);  // initialize digital pin 5 as an input.(data Out)

  unsigned long RawWeight;
  // wait for the chip to become ready
  long startTime;
  delay(5000);             //let the HX711 warm up
  AverageWeight = 0;
  for (int j = 0; j < FILTER_SAMPLES; j++)
  {
    startTime = millis();

    while ((digitalRead(PIN_DOUT) == HIGH) && ((millis() - startTime) < 1000)); //wait for data conversion ready

    if ((millis() - startTime) > 1000)                                          //or time out...
    {
      Serial.println("weight error");
    }
    RawWeight = 0;
    // pulse the clock pin 24 times to read the data
    for (char i = 0; i < 24; i++)
    {
      digitalWrite(PIN_CLOCK, HIGH);
      delayMicroseconds(2);
      RawWeight = RawWeight << 1;
      if (digitalRead(PIN_DOUT) == HIGH) RawWeight++;
      digitalWrite(PIN_CLOCK, LOW);
    }
    // set the channel and the gain factor (A 128) for the next reading using the clock pin (one pulse)
    digitalWrite(PIN_CLOCK, HIGH);
    delayMicroseconds(2);
    RawWeight = RawWeight ^ 0x800000;
    digitalWrite(PIN_CLOCK, LOW);

    weightSmoothArray[j] = RawWeight;
#ifdef xxRAW_WEIGHT_DEBUG
    Serial.print("Raw weight : \t");
    Serial.println(RawWeight);
#endif
    delayMicroseconds(60);
  }
  //digitalWrite(PIN_CLOCK, HIGH);    //to enter into power saving mode
  //median filter
  boolean done;
  float temp;
  int k, top, bottom;
  done = 0;                // flag to know when we're done sorting
  while (done != 1)
  { // simple swap sort, sorts numbers from lowest to highest
    done = 1;
    for (int j = 0; j < (FILTER_SAMPLES - 1); j++) {
      if (weightSmoothArray[j] > weightSmoothArray[j + 1]) {    // numbers are out of order - swap
        temp = weightSmoothArray[j + 1];
        weightSmoothArray [j + 1] =  weightSmoothArray[j] ;
        weightSmoothArray [j] = temp;
        done = 0;
      }
    }
  }
  // throw out top and bottom REJECT_RATIO % of samples - limit to throw out at least one from top and bottom
  bottom = max(((FILTER_SAMPLES * REJECT_RATIO)  / 100), 1);
  top = min((((FILTER_SAMPLES * (100 - REJECT_RATIO)) / 100) + 1  ), (FILTER_SAMPLES - 1)); // the + 1 is to make up for asymmetry caused by integer rounding
  k = 0;
  CurrentRawWeight = 0;
  for ( int j = bottom; j < top; j++) {
    CurrentRawWeight += weightSmoothArray[j];  // total remaining indices
    k++;
  }
  CurrentRawWeight = CurrentRawWeight / k;    // divide by number of samples and return the value
  //end median filter

#ifdef RAW_WEIGHT_DEBUG
  Serial.print("Raw average weight : ");
  Serial.println(CurrentRawWeight);
#endif
}
#endif

float mapf(float value, float fromLow, float fromHigh, float toLow, float toHigh)
{
  float result;
  result = (value - fromLow) * (toHigh - toLow) / (fromHigh - fromLow) + toLow;
  return result;
}
float volts(float raw)  //simple linear calibration...
{
  //return raw * 4.23 / 2440.14 ;  // carte 10k/10k
  return raw * 4.7 / 2485.59 ;  //calib carte Deyme (12k/10k)
}

void gotoSleep()
{
  digitalWrite(LED_PIN, HIGH);   // power off sensors
  digitalWrite(PWR_PIN, LOW);    //all sensors are Off
  Serial.println("Entering DeepSleep");
  pinMode(DHTPIN, INPUT);
  pinMode(ONE_WIRE_BUS, INPUT);
  pinMode(PIN_CLOCK, INPUT);
  pinMode(PIN_DOUT, INPUT);
  pinMode(SDA_PIN, INPUT);
  pinMode(SCL_PIN, INPUT);

  if (!hasReceivedCmd)
  {
    Serial.println("No Cmd received... ");
  }
  if (!hasReceivedTime)
  {
    Serial.println("No Time received... ");
    timeToSleep = 2;                        //reset to lowest value
  }
  int mm = (int)(floor((minute() * 60 + second()) / (60 * timeToSleep)) * timeToSleep + timeToSleep);
  if (mm > 60) mm = 60;
  Serial.print ("next integer minutes " );
  Serial.println (mm);
  long tt;                                              //time to sleep
  tt = mm * 60 - (minute() * 60 + second()) - sensorsGetTime / 1000;
  display_time();
  esp_sleep_enable_timer_wakeup(tt * uS_TO_S_FACTOR);
  esp_deep_sleep_start();                               //enter deep sleep mode
  delay(1000);
  abort();
}
