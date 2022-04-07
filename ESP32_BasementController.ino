/*
  Uses libraries and example code from ArDewpoint Ardunio based ventilation system under BSD License ( Author Michael Niemi. https://urldefense.proofpoint.com/v2/url?u=https-3A__github.com_mvniemi&d=DwIGaQ&c=Qznq1V5e4u04CfMRj920aPtDqN4RUEToMeZ6oK6t9iY&r=252_MRi3ygNlE59oTPrit4zJLEUvc2St2grwJ8AKdYY&m=edow_P6JMKI4yvio5AWbdg7zEQ8A_vByl07OKu_55_Y&s=4xq6x6_EGhFSizIcnB1MilF71gbkyjB_kLf1enRckug&e=  )
  
  Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, but not to use it commercially for profit making or to sub-license and/or to sell copies of the Software or to permit persons to whom the Software is furnished to do so, subject to the following conditions:
   The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/


//################# LIBRARIES ################

#include <WebServer_WT32_ETH01.h>                                                         // Standard ESP32 Arduino library used for MQTT
#include <PubSubClient.h>                                                                 // Standard ESP32 Arduino library used for MQTT
#include "EEPROM.h"                                                                       // Standard ESP32 Arduino library for EEPROM storage
#include <Wire.h>                                                                         // Standard ESP32 Arduino library for TWE/I2C bus
#include <SPI.h>                                                                          // Standard ESP32 Arduino library for SPI bus
#include <OneWire.h>                                                                      // Standard ESP32 Arduino library for OneWire bus
#include <Time.h>                                                                         // Standard ESP32 Arduino library
#include <TimeAlarms.h>                                                                   // Standard ESP32 Arduino library
#include "MCP23S08.h"                                                                     // library from https://github.com/julianschuler/MCP23S08
#include "MS5525DSO.h"                                                                    // based on MS5525DSO sample code from: https://github.com/yubox-node-org/MS5525DSO-Arduino
#include "SparkFunBME280.h"                                                               // from Sparkfun under BSD License to be found in Arduino library install
#include <DallasTemperature.h>                                                            // library from https://github.com/milesburton/Arduino-Temperature-Control-Library to be found in Arduino library install

// i2c wire.h normally used i2c pins are not availeble on WT32-ETH01 so use other pins for i2c
#define I2C_SCL    32    // WT32-ETH01 CFG    = Gpio 32      non standard i2c adress 
#define I2C_SDA    33    // WT32-ETH01 485_EN = Gpio 33      non standard i2c adress

// Expander Pin Assignments
#define AIRPUMP_PIN 0
#define SPARE_PIN 1 
#define FAN_PIN 2
#define VALVE2_PIN 3
#define VALVE1_PIN 4

// SPI bus defines
#define SCLK 14
#define MISO 2 
#define MOSI 15
#define CS_PIN_MCP23S08  4
#define CS_PIN_MS5525DSO 12

// OneWire bus defines
#define OneWirePin 17

// Watermeter defines
#define WaterMeter1Pin 36
#define WaterMeter2Pin 39
#define WaterMeter1EEPROMAddr 0
#define WaterMeter2EEPROMAddr 4

// Ventialtion defines
#define VENTTHRESHOLD 1.5                                               //Set the ventilation threshold of the DEWPOINT in degrees celcius. I recccomend it being greater than 1, as that seems to be the margin of error for these sensors

// MCP23S08 IO expander
MCP23S08 expander(CS_PIN_MCP23S08);

// MS5525DSO pressure sensor
MS5525DSO pressuresensor(CS_PIN_MS5525DSO);
float pressurelosstubing = 0.014;


// DS18B20 Temperature sensors
OneWire oneWire(OneWirePin);
DallasTemperature DS18B20_sensors(&oneWire);
DeviceAddress bottomThermometer, topThermometer;

// BME280 sensors
BME280 sensorInside;
BME280 sensorOutside;
double RollingdewPointIn, RollingdewPointOut;
bool FanOnInhibit = true;
bool Ventilation = false;

// Select the IP address according to your local network
IPAddress myIP(192, 168, 1, 35);
IPAddress myGW(192, 168, 1, 1);
IPAddress mySN(255, 255, 255, 0);
IPAddress myDNS(8, 8, 8, 8);                      // Google DNS Server IP
const char* mqttServer = "192.168.1.50  ";        // Broker address
const char *ID        = "MQTTClient_SSL-Client";  // Name of our device, must be unique
const char *TOPIC     = "MQTT_Pub";               // Topic to subcribe to
const char *subTopic  = "MQTT_Sub";               // Topic to subcribe to

WebServer server(80);

void callback(char* topic, byte* payload, unsigned int length) 
{
}

WiFiClient    ethClient;
PubSubClient    client(mqttServer, 1883, callback, ethClient);

uint32_t WaterMeterCnt1, WaterMeterCnt2;


// NTP Server
const char* ntpServer = "pool.ntp.org";
const long  gmtOffset_sec = 1;
const int   daylightOffset_sec = 3600;
struct tm timeinfo;
  
void setup()
{
  // MQTT Client setup  
  // To be called before ETH.begin()
  WT32_ETH01_onEvent();

  //bool begin(uint8_t phy_addr=ETH_PHY_ADDR, int power=ETH_PHY_POWER, int mdc=ETH_PHY_MDC, int mdio=ETH_PHY_MDIO, 
  //           eth_phy_type_t type=ETH_PHY_TYPE, eth_clock_mode_t clk_mode=ETH_CLK_MODE);
  //ETH.begin(ETH_PHY_ADDR, ETH_PHY_POWER, ETH_PHY_MDC, ETH_PHY_MDIO, ETH_PHY_TYPE, ETH_CLK_MODE);
  ETH.begin(ETH_PHY_ADDR, ETH_PHY_POWER);

  // Static IP, leave without this line to get IP via DHCP
  //bool config(IPAddress local_ip, IPAddress gateway, IPAddress subnet, IPAddress dns1 = 0, IPAddress dns2 = 0);
  ETH.config(myIP, myGW, mySN, myDNS);

  WT32_ETH01_waitForConnect();

  client.setServer(mqttServer, 1883);
  client.setCallback(callback);

  // Read EEPROM for last watermeter counter values
  if (!EEPROM.begin(10)) {
    Serial.println("Failed to initialise EEPROM");
    Serial.println("Restarting...");
    delay(1000);
    ESP.restart();
  }

  WaterMeterCnt1 = EEPROM.readUInt(WaterMeter1EEPROMAddr);
  WaterMeterCnt2 = EEPROM.readUInt(WaterMeter2EEPROMAddr);

  // Watermeter inputs setup
  pinMode(WaterMeter1Pin, INPUT);
  pinMode(WaterMeter2Pin, INPUT);
  
// Set watermeters to zero
//  EEPROM.writeUInt(WaterMeter1EEPROMAddr, 0);
//  EEPROM.writeUInt(WaterMeter2EEPROMAddr, 0);
//  EEPROM.commit(); 
  
  Wire.begin(I2C_SDA, I2C_SCL);                                                           // start up the I2C bus (BME280 sensors)                
  SPI.begin(SCLK, MISO, MOSI);                                                            // start up the SPI bus (MCP23S08 and MS5525DSO)

  // Setup MCP23S08 IO Expander
  expander.begin();                                                                       // begin communication with the pin/O expander
  for (uint8_t pin = 0; pin < 8; pin++) {                                                 // pins 0 - 7 available
    expander.pinModeIO(pin, OUTPUT);                                                      // set all pins to output
  }

  // Setup MS5525DSO pressure sensor
  pressuresensor.begin();
  
  // Setup BME280 sensors
  sensorInside.setI2CAddress(0x77);
  sensorOutside.setI2CAddress(0x76);
  
  sensorInside.beginI2C();
  sensorOutside.beginI2C();
  
  sensorInside.setFilter(0);                                                              //0 to 4 is valid. Filter coefficient. See 3.4.4
  sensorInside.setStandbyTime(5);                                                         //0 to 7 valid. Time between readings. See table 27.

  sensorInside.setTempOverSample(16);                                                      //0 to 16 are valid. 0 disables temp sensing. See table 24.
  sensorInside.setPressureOverSample(16);                                                  //0 to 16 are valid. 0 disables pressure sensing. See table 23.
  sensorInside.setHumidityOverSample(16);                                                  //0 to 16 are valid. 0 disables humidity sensing. See table 19.
  
  sensorInside.setMode(MODE_NORMAL);                                                      //MODE_SLEEP, MODE_FORCED, MODE_NORMAL is valid. See 3.3
  
  sensorOutside.setFilter(0);                                                             //0 to 4 is valid. Filter coefficient. See 3.4.4
  sensorOutside.setStandbyTime(0);                                                        //0 to 7 valid. Time between readings. See table 27.

  sensorOutside.setTempOverSample(16);                                                     //0 to 16 are valid. 0 disables temp sensing. See table 24.
  sensorOutside.setPressureOverSample(16);                                                 //0 to 16 are valid. 0 disables pressure sensing. See table 23.
  sensorOutside.setHumidityOverSample(16);                                                 //0 to 16 are valid. 0 disables humidity sensing. See table 19.
  
  sensorOutside.setMode(MODE_NORMAL);                                                     //MODE_SLEEP, MODE_FORCED, MODE_NORMAL is valid. See 3.3

  RollingdewPointIn = sensorInside.dewPointC();
  RollingdewPointOut = sensorOutside.dewPointC();

  // Setup DS18B20 temperature sensors
  DS18B20_sensors.begin();

  //set real time clock

  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);  
  getLocalTime(&timeinfo);
  
  Alarm.timerRepeat(0,15,0, runalarm_15min);
  Alarm.timerRepeat(1,0,0, runalarm_hour);  // alarm for watertank level sensingde
  Alarm.timerRepeat(168,0,0, StoreWMCounters);

  // Allow the hardware to sort itself out
  delay(1500);
}

double MS5525DSO_pressure, MS5525DSO_temperature;
char tempString[8];
char timeStringBuff[50];
  
void loop() 
{
 
  if (!client.connected()) 
  {
    reconnect();
  }

  
  Alarm.delay(0);                                                                                 // call for timealarms to function

  Check_WaterMeters();
  
  client.loop();
}

void Check_WaterMeters() {
  long start = millis() ;
  float temp;
  long debouncing_time = 15; //Debouncing Time in Milliseconds
  boolean current_state1, current_state2 ;
  boolean previous_state1 = digitalRead(WaterMeter1Pin) ;
  boolean previous_state2 = digitalRead(WaterMeter2Pin) ;
  for( ; millis()-start <= debouncing_time ; )
  {
     current_state1 = digitalRead(WaterMeter1Pin) ;
     if (current_state1 && !previous_state1){
        WaterMeterCnt1++;
        dtostrf(WaterMeterCnt1, 1, 0, tempString); 
        client.publish("ESP32_Kelder/Watermeter/Counter1", tempString);
        dtostrf(((float)WaterMeterCnt1/1000), 1, 3, tempString); 
        client.publish("ESP32_Kelder/Watermeter/m3_1", tempString);
     }
     previous_state1 = current_state1 ;
     
     current_state2 = digitalRead(WaterMeter2Pin) ;
     if (current_state2 && !previous_state2) {
        WaterMeterCnt2++;
        dtostrf(WaterMeterCnt2, 1, 0, tempString); 
        client.publish("ESP32_Kelder/Watermeter/Counter2", tempString);
        dtostrf(((float)WaterMeterCnt2/1000), 1, 3, tempString); 
        client.publish("ESP32_Kelder/Watermeter/m3_2", tempString);
     }
     previous_state2 = current_state2 ;     
  }
}

void runalarm_15min(){
  getLocalTime(&timeinfo);
  strftime(timeStringBuff, sizeof(timeStringBuff), "%A, %B %d %Y %H:%M:%S", &timeinfo);
  client.publish("ESP32_Kelder/Debug/Time", timeStringBuff);
  
  DS18B20_sensors.requestTemperatures();
  dtostrf(DS18B20_sensors.getTempCByIndex(0), 1, 1, tempString); 
  client.publish("ESP32_Kelder/Vijver/Temperature1", tempString);
  dtostrf(DS18B20_sensors.getTempCByIndex(1), 1, 1, tempString); 
  client.publish("ESP32_Kelder/Vijver/Temperature2", tempString);
  
  dtostrf(sensorInside.readTempC(), 1, 1, tempString);
  client.publish("ESP32_Kelder/Ventilation/TemperatureInside", tempString);
  dtostrf(sensorInside.readFloatHumidity(), 1, 0, tempString);
  client.publish("ESP32_Kelder/Ventilation/HumidityInside", tempString);
  dtostrf((sensorInside.readFloatPressure() / 100.0F), 1, 0, tempString);
  client.publish("ESP32_Kelder/Ventilation/PressureInside", tempString);
  dtostrf(sensorInside.dewPointC(), 1, 1, tempString);
  RollingdewPointIn = (((RollingdewPointIn*5)-RollingdewPointIn)+sensorInside.dewPointC())/5;
  client.publish("ESP32_Kelder/Ventilation/DewpointInside", tempString);
  dtostrf(RollingdewPointIn, 1, 1, tempString);
  client.publish("ESP32_Kelder/Ventilation/RollingDewpointInside", tempString);
  dtostrf(sensorOutside.readTempC(), 1, 1, tempString);
  client.publish("ESP32_Kelder/Ventilation/TemperatureOutside", tempString);
  dtostrf(sensorOutside.readFloatHumidity(), 1, 0, tempString);
  client.publish("ESP32_Kelder/Ventilation/HumidityOutside", tempString);
  dtostrf((sensorOutside.readFloatPressure() / 100.0F), 1, 0, tempString);
  client.publish("ESP32_Kelder/Ventilation/PressureOutside", tempString); 
  dtostrf(sensorOutside.dewPointC(), 1, 1, tempString);
  client.publish("ESP32_Kelder/Ventilation/DewpointOutside", tempString);
  RollingdewPointOut = (((RollingdewPointOut*5)-RollingdewPointOut)+sensorOutside.dewPointC())/5;
  dtostrf(RollingdewPointOut, 1, 1, tempString);
  client.publish("ESP32_Kelder/Ventilation/RollingDewpointOutside", tempString);

  // if outside rolling average dewpoint + additional threshold is lower then inside rolling average dewpoint (no venting below 15 degrees is not valid here, ventilation can always be done in my case)
  if ( ( RollingdewPointOut+VENTTHRESHOLD<RollingdewPointIn ) && !Ventilation && FanOnInhibit)
    {
     // switch on fan 
     Ventilation=true;
     client.publish("ESP32_Kelder/Ventilation/fan", "ON");
     expander.digitalWriteIO(VALVE1_PIN, HIGH);
     expander.digitalWriteIO(VALVE2_PIN, HIGH);
     Alarm.timerOnce(60, FanOnMinute);
    }
    else if ( ( RollingdewPointOut+VENTTHRESHOLD>=RollingdewPointIn ) && Ventilation )
    {
     // switch off fan
     Ventilation=false;
     FanOnInhibit=false;
     client.publish("ESP32_Kelder/Ventilation/fan", "OFF");
     expander.digitalWriteIO(FAN_PIN, LOW);
     expander.digitalWriteIO(VALVE1_PIN, LOW);
     expander.digitalWriteIO(VALVE2_PIN, LOW);
     Alarm.timerOnce(3600, FanOffHour);
    }

  dtostrf(EEPROM.readUInt(WaterMeter1EEPROMAddr), 1, 0, tempString);
  client.publish("ESP32_Kelder/Debug/WaterMeter1StoredValue", tempString);
  dtostrf(EEPROM.readUInt(WaterMeter2EEPROMAddr), 1, 0, tempString);
  client.publish("ESP32_Kelder/Debug/WaterMeter2StoredValue", tempString);
}

void runalarm_hour(){
  expander.digitalWriteIO(AIRPUMP_PIN, HIGH);                                                    // turn airpump on
  Alarm.timerOnce(10, WatertankMeasurement);             
}

void FanOffHour(){
  FanOnInhibit=true;
  client.publish("ESP32_Kelder/Debug/VentilationOffOneHour", "One Hour Fan off");
}

void FanOnMinute(){
  expander.digitalWriteIO(FAN_PIN, HIGH);
  client.publish("ESP32_Kelder/Debug/VentilationOnOneMinute", "One Minute wait done");
}

void WatertankMeasurement(){
  float height, liters;
  float average_pressure = 0;
  float average_temp = 0;
  for (int i = 0; i <= 4; i++) {
    pressuresensor.readPressureAndTemperature(&MS5525DSO_pressure, &MS5525DSO_temperature);        // Read pressure and temperature from MS5525DSO pressure sensor
    average_pressure = average_pressure + MS5525DSO_pressure;
    average_temp = average_temp + MS5525DSO_temperature;
  }
  average_pressure = average_pressure/5;
  average_temp = average_temp/5;
  
  dtostrf(average_pressure, 1, 2, tempString);
  client.publish("ESP32_Kelder/Watertank/Pressure", tempString);
  dtostrf(average_temp, 1, 1, tempString);
  client.publish("ESP32_Kelder/Watertank/Temperature", tempString);

  height = ((average_pressure/14.504)-pressurelosstubing)/0.0980638 ;
  dtostrf(height, 1, 2, tempString);
  client.publish("ESP32_Kelder/Watertank/Height", tempString);

  liters = (height*(3.1415*0.9*1.175))*1000;
  dtostrf(liters, 1, 2, tempString);
  client.publish("ESP32_Kelder/Watertank/Liter", tempString);

  expander.digitalWriteIO(AIRPUMP_PIN, LOW);                                                     // turn airpump off
}

void reconnect()
{
  // Loop until we're reconnected
  while (!client.connected())
  {
    // Attempt to connect
    if (client.connect(ID, "try", "try"))
    {
      // This is a workaround to address https://github.com/OPEnSLab-OSU/SSLClient/issues/9
      //ethClientSSL.flush();
      // ... and resubscribe
      client.subscribe(subTopic);
      // for loopback testing
      client.subscribe(TOPIC);
      // This is a workaround to address https://github.com/OPEnSLab-OSU/SSLClient/issues/9
      //ethClientSSL.flush();
    }
    else
    {
      // Wait 5 seconds before retrying 
      delay(5000);
    }
  }
}

void StoreWMCounters()
{
  client.publish("ESP32_Kelder/Debug/DayAlarmTriggered", "Triggered");
  EEPROM.writeUInt(WaterMeter1EEPROMAddr, WaterMeterCnt1);
  EEPROM.writeUInt(WaterMeter2EEPROMAddr, WaterMeterCnt2);
  EEPROM.commit(); 
  client.publish("ESP32_Kelder/Debug/DayAlarmTriggeredEEPROM", "EEPROM saved");
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer); 
}
