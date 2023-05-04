#include <Arduino.h>
#include <IotWebConf.h>
#include <IotWebConfUsing.h>
#include <IotWebConfMultipleWifi.h>
#include <ModbusTCP.h>

#include <NTPClient.h>
#include <SolarCalculator.h>
#include <WiFiUdp.h>
#include <MQTT.h>
#include <ArduinoJson.h>
#include <WiFiClientSecure.h>

#define IOTWEBCONF_CONFIG_USE_MDNS

#if defined(ESP8266)
  #include <ESP8266WiFi.h> 
  #include <ESP8266HTTPUpdateServer.h>
  ESP8266HTTPUpdateServer httpUpdater;
#elif defined(ESP32)
  #include <WiFi.h>
  #include <IotWebConfESP32HTTPUpdateServer.h>
  HTTPUpdateServer httpUpdater;
#else
#error "This ain't a ESP8266 or ESP32, dumbo!"
#endif



// -- Initial name of the Thing. Used e.g. as SSID of the own Access Point.
const char thingName[] = "HeaterControl";

// -- Initial password to connect to the Thing, when it creates an own Access Point.
const char wifiInitialApPassword[] = "smrtTHNG8266";


#define STRING_LEN 69
#define NUMBER_LEN 5

// -- Configuration specific key. The value should be modified if config structure was changed.
#define CONFIG_VERSION "019"

// -- When CONFIG_PIN is pulled to ground on startup, the Thing will use the initial
//      password to buld an AP. (E.g. in case of lost password)


// -- Status indicator pin.
//      First it will light up (kept LOW), on Wifi connection it will blink,
//      when connected to the Wifi it will turn off (kept HIGH).
//#define STATUS_PIN LED_BUILTIN


#if defined(ESP8266)
  // D7 13
  #define PIN_HEATER_A1 13
  //D8 15
  #define PIN_HEATER_A2 15
  // D1 5
  #define PIN_HEATER_A3 5
  // D4 2
  #define PIN_HEATER_B1 2
  // D2 4
  #define PIN_HEATER_B2 4
  // D6 12
  #define PIN_HEATER_B3 12
  // D5 14
  #define ENABLE_PIN_A 14
  // D0 3
  #define CONFIG_PIN 3

#elif defined(ESP32)
  #define PIN_HEATER_A1 5
  #define PIN_HEATER_A2 18
  #define PIN_HEATER_A3 19
  #define PIN_HEATER_B1 21
  #define PIN_HEATER_B2 22
  #define PIN_HEATER_B3 23
  #define ENABLE_PIN_A 35
  #define CONFIG_PIN 34
#else
#error "This ain't a ESP8266 or ESP32, dumbo!"
#endif


#define REG_COUNT 4


union conv 
{
  uint16_t uint_val;
  int16_t int_val;
} conv;


uint8_t targetSID = 100;

void handleRoot();
String getDateTime();

// -- Callback methods.
void configSaved();
bool formValidator(iotwebconf::WebRequestWrapper* webRequestWrapper);
void wifiConnected();
bool connectMqtt();
bool connectMqttOptions();

void manEnableA1();
void manEnableA2();
void manEnableA3();
void manEnableB1();
void manEnableB2();
void manEnableB3();

String getJSONStatus();
void handleAPI();
void sendMqttStatus();

bool cb(Modbus::ResultCode event, uint16_t transactionId, void* data);

DNSServer dnsServer;
WebServer server(80);


double transit, sunrise, sunset;


char intParamValueminSOC[NUMBER_LEN];

char intParamValueCurrA1[NUMBER_LEN];
char intParamValueCurrA2[NUMBER_LEN];
char intParamValueCurrA3[NUMBER_LEN];
char intParamValueCurrB1[NUMBER_LEN];
char intParamValueCurrB2[NUMBER_LEN];
char intParamValueCurrB3[NUMBER_LEN];

char intParamValueTimezone[NUMBER_LEN];
char intParamValueSunsetOffset[NUMBER_LEN];

char intParamValueBufferSOC[NUMBER_LEN];
char intParamValueBufferCurrent[NUMBER_LEN];

char intParamValueIP1[NUMBER_LEN];
char intParamValueIP2[NUMBER_LEN];
char intParamValueIP3[NUMBER_LEN];
char intParamValueIP4[NUMBER_LEN];

char mqttServerValue[STRING_LEN];
char mqttUserNameValue[STRING_LEN];
char mqttUserPasswordValue[STRING_LEN];
char mqttTopicPrefixValue[STRING_LEN];


unsigned long lastModbusCall = 0;
unsigned long lastHeaterCall = 0;
unsigned long lastMqttCall = 0;
unsigned long modbusTimer = 10 * 1000;

unsigned long t_man_activate[6];


IPAddress remote;
Modbus::ResultCode modbusresult;


u_int16_t res[REG_COUNT];
u_int16_t res1;
u_int16_t res2;


int heaterCurrent[6];
bool heaterEnable[6];

bool bufferHeatEnable;

int sunsetOffset = -6;

int numberOfActiveHeater = 0;

int minBatSOC;

bool inTimerange = false;

IotWebConf iotWebConf(thingName, &dnsServer, &server, wifiInitialApPassword, CONFIG_VERSION);
// -- You can also use namespace formats e.g.: iotwebconf::TextParameter

iotwebconf::ChainedWifiParameterGroup chainedWifiParameterGroups[] = {
  iotwebconf::ChainedWifiParameterGroup("wifi1"),
  iotwebconf::ChainedWifiParameterGroup("wifi2"),
  iotwebconf::ChainedWifiParameterGroup("wifi3")
};

iotwebconf::MultipleWifiAddition multipleWifiAddition(
  &iotWebConf,
  chainedWifiParameterGroups,
  sizeof(chainedWifiParameterGroups)  / sizeof(chainedWifiParameterGroups[0]));

iotwebconf::OptionalGroupHtmlFormatProvider optionalGroupHtmlFormatProvider;

IotWebConfParameterGroup group1 = IotWebConfParameterGroup("group1", "Cerbo");
IotWebConfNumberParameter intParamIP1 = IotWebConfNumberParameter("IP Adress Part 1", "ipaddr1", intParamValueIP1, NUMBER_LEN, "192", "0..255", "min='1' max='255' step='1'");
IotWebConfNumberParameter intParamIP2 = IotWebConfNumberParameter("IP Adress Part 2", "ipaddr2", intParamValueIP2, NUMBER_LEN, "168", "0..255", "min='0' max='255' step='1'");
IotWebConfNumberParameter intParamIP3 = IotWebConfNumberParameter("IP Adress Part 3", "ipaddr3", intParamValueIP3, NUMBER_LEN, "1", "0..255", "min='0' max='255' step='1'");
IotWebConfNumberParameter intParamIP4 = IotWebConfNumberParameter("IP Adress Part 4", "ipaddr4", intParamValueIP4, NUMBER_LEN, "41", "0..255", "min='0' max='255' step='1'");


// -- We can add a legend to the separator
IotWebConfParameterGroup group2 = IotWebConfParameterGroup("group2", "Heater Config");
IotWebConfNumberParameter intParamMinSOC = IotWebConfNumberParameter("Min SOC", "minSOC", intParamValueminSOC, NUMBER_LEN, "20", "1..100", "min='1' max='100' step='1'");
IotWebConfNumberParameter intParamMinCurrA1 = IotWebConfNumberParameter("Current Heater A1", "ucA1", intParamValueCurrA1, NUMBER_LEN, "0", "0..100", "min='0' max='100' step='1'");
IotWebConfNumberParameter intParamMinCurrA2 = IotWebConfNumberParameter("Current Heater A2", "ucA2", intParamValueCurrA2, NUMBER_LEN, "0", "0..100", "min='0' max='100' step='1'");
IotWebConfNumberParameter intParamMinCurrA3 = IotWebConfNumberParameter("Current Heater A3", "ucA3", intParamValueCurrA3, NUMBER_LEN, "0", "0..100", "min='0' max='100' step='1'");
IotWebConfNumberParameter intParamMinCurrB1 = IotWebConfNumberParameter("Current Heater B1", "ucB1", intParamValueCurrB1, NUMBER_LEN, "0", "0..100", "min='0' max='100' step='1'");
IotWebConfNumberParameter intParamMinCurrB2 = IotWebConfNumberParameter("Current Heater B2", "ucB2", intParamValueCurrB2, NUMBER_LEN, "0", "0..100", "min='0' max='100' step='1'");
IotWebConfNumberParameter intParamMinCurrB3 = IotWebConfNumberParameter("Current Heater B3", "ucB3", intParamValueCurrB3, NUMBER_LEN, "0", "0..100", "min='0' max='100' step='1'");

IotWebConfNumberParameter intParamBufferSOC = IotWebConfNumberParameter("Buffer SOC", "bufsoc", intParamValueBufferSOC, NUMBER_LEN, "90", "0..100", "min='0' max='100' step='1'");
IotWebConfNumberParameter intParamBufferCurrent = IotWebConfNumberParameter("Buffer Current", "bufcur", intParamValueBufferCurrent, NUMBER_LEN, "24", "0..100", "min='0' max='100' step='1'");


IotWebConfParameterGroup group3 = IotWebConfParameterGroup("group3", "Time Config");

IotWebConfNumberParameter intParamSunsetOffset = IotWebConfNumberParameter("Sunset Offset", "sunoffset", intParamValueSunsetOffset, NUMBER_LEN, "-6", "-6..6", "min='-6' max='6' step='1'");
IotWebConfNumberParameter intParamTimezone = IotWebConfNumberParameter("Timezone Offset", "tzoffset", intParamValueTimezone, NUMBER_LEN, "2", "-12..12", "min='-12' max='12' step='1'");



IotWebConfParameterGroup mqttGroup = IotWebConfParameterGroup("mqtt", "MQTT configuration");
IotWebConfTextParameter mqttServerParam = IotWebConfTextParameter("MQTT server", "mqttServer", mqttServerValue, STRING_LEN);
IotWebConfTextParameter mqttUserNameParam = IotWebConfTextParameter("MQTT user", "mqttUser", mqttUserNameValue, STRING_LEN);
IotWebConfPasswordParameter mqttUserPasswordParam = IotWebConfPasswordParameter("MQTT password", "mqttPass", mqttUserPasswordValue, STRING_LEN);
IotWebConfTextParameter mqttTopicPrefix = IotWebConfTextParameter("MQTT Topic Prefix", "mqttPrefix", mqttTopicPrefixValue, STRING_LEN);



float batteryCurrent = 0;
float batterySOC = 0;
float estimatedCurrent = 0;
float pvCurrent = 0;
float vebusCurrent = 0;

float bufferSOC;
float bufferCurrent;

unsigned long modbusSuccessCounter = 0;
unsigned long modbusErrorCounter = 0;

bool enableA = false;
//bool enableB = false;

WiFiClientSecure net;
MQTTClient mqttClient(512);

unsigned long lastMqttConnectionAttempt = 0;

bool needMqttConnect = false;
bool needReset = false;

ModbusTCP mb;  //ModbusTCP object

WiFiUDP ntpUDP;

int utc_offset = 2;
NTPClient timeClient(ntpUDP, "de.pool.ntp.org", utc_offset * 3600, 60000);


bool isOnline = false;
bool needReconnect = false;

void setup() 
{
  bool validConfig;
  Serial.begin(115200);
  Serial.println();
  Serial.println("Starting up...");

  delay(250);

  pinMode(CONFIG_PIN, INPUT_PULLUP);

  pinMode(ENABLE_PIN_A, INPUT_PULLUP);
  //pinMode(ENABLE_PIN_B, INPUT_PULLUP);
  

  pinMode(PIN_HEATER_A1, OUTPUT);
  pinMode(PIN_HEATER_A2, OUTPUT);
  pinMode(PIN_HEATER_A3, OUTPUT);
  pinMode(PIN_HEATER_B1, OUTPUT);
  pinMode(PIN_HEATER_B2, OUTPUT);
  pinMode(PIN_HEATER_B3, OUTPUT);

  delay(250);

  group1.addItem(&intParamIP1);
  group1.addItem(&intParamIP2);
  group1.addItem(&intParamIP3);
  group1.addItem(&intParamIP4);

  Serial.println("Created Group1");

  group2.addItem(&intParamMinSOC);
  
  group2.addItem(&intParamMinCurrA1);
  group2.addItem(&intParamMinCurrA2);
  group2.addItem(&intParamMinCurrA3);
  group2.addItem(&intParamMinCurrB1);
  group2.addItem(&intParamMinCurrB2);
  group2.addItem(&intParamMinCurrB3);
  group2.addItem(&intParamBufferSOC);
  group2.addItem(&intParamBufferCurrent);

  Serial.println("Created Group3");
  
  group3.addItem(&intParamSunsetOffset);
  group3.addItem(&intParamTimezone);

  Serial.println("Created Group3");

  mqttGroup.addItem(&mqttServerParam);
  mqttGroup.addItem(&mqttUserNameParam);
  mqttGroup.addItem(&mqttUserPasswordParam);
  mqttGroup.addItem(&mqttTopicPrefix);

  //iotWebConf.setStatusPin(STATUS_PIN);
  iotWebConf.setConfigPin(CONFIG_PIN);

  Serial.println("IOTWC Pins configured");

  multipleWifiAddition.init();

  iotWebConf.addParameterGroup(&group1);
  iotWebConf.addParameterGroup(&group2);
  iotWebConf.addParameterGroup(&group3);
  iotWebConf.addParameterGroup(&mqttGroup);

  iotWebConf.setupUpdateServer(
    [](const char* updatePath) { httpUpdater.setup(&server, updatePath); },
    [](const char* userName, char* password) { httpUpdater.updateCredentials(userName, password); });

  Serial.println("IOTWC Config added");

  iotWebConf.setConfigSavedCallback(&configSaved);
  iotWebConf.setFormValidator(&formValidator);
  iotWebConf.setHtmlFormatProvider(&optionalGroupHtmlFormatProvider);
  //iotWebConf.getApTimeoutParameter()->visible = true;

  Serial.println("IOTWC Callbacks added");

  delay(250);
  // -- Initializing the configuration.
  validConfig = iotWebConf.init();

  if (!validConfig)
  {
    Serial.println("IOTWC Invalid Config");
    mqttServerValue[0] = '\0';
    mqttUserNameValue[0] = '\0';
    mqttUserPasswordValue[0] = '\0';
    mqttTopicPrefixValue[0] = '\0';
  } else {
    Serial.println("IOTWC Valid Config");
  }

  remote = IPAddress(atoi(intParamValueIP1), atoi(intParamValueIP2), atoi(intParamValueIP3), atoi(intParamValueIP4));

  heaterCurrent[0] = atoi(intParamValueCurrA1);
  heaterCurrent[1] = atoi(intParamValueCurrA2);
  heaterCurrent[2] = atoi(intParamValueCurrA3);
  heaterCurrent[3] = atoi(intParamValueCurrB1);
  heaterCurrent[4] = atoi(intParamValueCurrB2);
  heaterCurrent[5] = atoi(intParamValueCurrB3);

  sunsetOffset = atoi(intParamValueSunsetOffset);

  utc_offset = atoi(intParamValueTimezone);

  minBatSOC = atoi(intParamValueminSOC);
  bufferSOC = atoi(intParamValueBufferSOC);
  bufferCurrent = atoi(intParamValueBufferCurrent);


  // -- Set up required URL handlers on the web server.
  server.on("/", handleRoot);
  server.on("/api", handleAPI);
  server.on("/man/a1", manEnableA1);
  server.on("/man/a2", manEnableA2);
  server.on("/man/a3", manEnableA3);
  server.on("/man/b1", manEnableB1);
  server.on("/man/b2", manEnableB2);
  server.on("/man/b3", manEnableB3);
  server.on("/config", []{ iotWebConf.handleConfig(); });
  server.onNotFound([](){ iotWebConf.handleNotFound(); });

  net.setInsecure();
  mqttClient.begin(mqttServerValue, 8883, net);

  Serial.println("Ready.");
  Serial.println("Config Done");

  enableA = false;
  //enableB = false;

  bufferHeatEnable = false;

  for(int i=0; i<6; i++) {
    t_man_activate[i] = ULONG_MAX-1;
  }
  
}

void loop() 
{
  float remainingCurrent;
  unsigned long currentTime = millis();
  unsigned long timediff;
  unsigned long timediff2;
  unsigned long timediff3;
  float timeAct;

  
  // -- doLoop should be called as frequently as possible.
  iotWebConf.doLoop();
  mb.task();
  mqttClient.loop();
  delay(10);

  timediff = (currentTime > lastModbusCall) ? currentTime - lastModbusCall : lastModbusCall - currentTime;
  timediff2 = (currentTime > lastHeaterCall) ? currentTime - lastHeaterCall : lastHeaterCall - currentTime;
  timediff3 = (currentTime > lastMqttCall) ? currentTime - lastMqttCall : lastMqttCall - currentTime;

  
  if(millis() > ULONG_MAX-(300*1000)) {
    Serial.println("Need Reboot");
    needReset = true;
  }

  // Handle Time sync
  if(iotWebConf.getState() == iotwebconf::OnLine) {
    if(!isOnline) {
      timeClient.begin();
      timeClient.setTimeOffset(utc_offset * 3600);
      mb.client();
    }
    isOnline = true;
    timeClient.update();
  } else {
    isOnline = false;
  }

  // Handle MQTT and Reboot
  if (needMqttConnect)
  {
    if (connectMqtt())
    {
      needMqttConnect = false;
    }
  }
  else if ((iotWebConf.getState() == iotwebconf::OnLine) && (!mqttClient.connected()))
  {
    connectMqtt();
  }

  if (needReset)
  {
    Serial.println("Rebooting after 1 second.");
    iotWebConf.delay(1000);
    ESP.restart();
  }



  if(timediff > modbusTimer) {
    lastModbusCall = currentTime;

    if(iotWebConf.getState() == iotwebconf::OnLine) {
      Serial.println("Modbus read input register");
      
      if(mb.isConnected(remote)) {
        mb.readIreg(remote, 841, res, REG_COUNT, cb, targetSID);
        mb.readIreg(remote, 851, &res1, 1, cb, targetSID);
        mb.readIreg(remote, 865, &res2, 1, cb, targetSID);
      } else {
        Serial.println("Reconnect to Modbus Server");
        
        mb.connect(remote);
      }
    }

    
    time_t epochTime = timeClient.getEpochTime();
    struct tm *ptm = gmtime ((time_t *)&epochTime); 
    // Calculate the times of sunrise, transit, and sunset, in hours (UTC)
    calcSunriseSunset(ptm->tm_year+1900, ptm->tm_mon+1, ptm->tm_mday, 48.402141, 9.988537, transit, sunrise, sunset);

    timeAct = timeClient.getHours() + ((float)timeClient.getMinutes() / 60.0);
    inTimerange = (timeAct > (sunrise + utc_offset + sunsetOffset)) && (timeAct < (sunset + utc_offset - sunsetOffset));
    
  }

  

  if(timediff2 > 2*modbusTimer) {


    enableA = (digitalRead(ENABLE_PIN_A) == LOW);
    //enableB = (digitalRead(ENABLE_PIN_B) == LOW);
    
    lastHeaterCall = currentTime;
    estimatedCurrent = pvCurrent + vebusCurrent;
    
    if(vebusCurrent > 0) estimatedCurrent = pvCurrent;

    
    numberOfActiveHeater = 0;

    if(batterySOC > bufferSOC) {
      bufferHeatEnable = true;
    }

    if(batterySOC < (bufferSOC -5)) {
      bufferHeatEnable = false;
    }

    if(bufferHeatEnable) {
      if(estimatedCurrent < 0) {
        estimatedCurrent = 0;
      }

      estimatedCurrent = estimatedCurrent +  (float)bufferCurrent;
    }

    remainingCurrent = estimatedCurrent;

    
    for(int i=0; i<6; i++) {
      if( (heaterCurrent[i] != 0) && (remainingCurrent > heaterCurrent[i]) && (batterySOC > minBatSOC) && inTimerange && ((i<3 && enableA) || (i>=3)) ) {
        remainingCurrent -= heaterCurrent[i];
        numberOfActiveHeater++;
        heaterEnable[i] = true;
      } else {
        heaterEnable[i] = false;
      }
    }
    
    for(int i=0; i<6; i++) {
      if((currentTime > t_man_activate[i]) &&((currentTime - t_man_activate[i]) < 120 * 1000)) {
        heaterEnable[i] = true;
      }
    }


    digitalWrite(PIN_HEATER_A1, heaterEnable[0] ? HIGH : LOW);
    digitalWrite(PIN_HEATER_A2, heaterEnable[1] ? HIGH : LOW);
    digitalWrite(PIN_HEATER_A3, heaterEnable[2] ? HIGH : LOW);
    digitalWrite(PIN_HEATER_B1, heaterEnable[3] ? HIGH : LOW);
    digitalWrite(PIN_HEATER_B2, heaterEnable[4] ? HIGH : LOW);
    digitalWrite(PIN_HEATER_B3, heaterEnable[5] ? HIGH : LOW);

    sendMqttStatus();
    
  }

  if(timediff3 > 300 * 1000) {
    lastMqttCall = currentTime;

    Serial.println("Send MQTT Status");
    sendMqttStatus();
  }

}

void wifiConnected()
{
  Serial.println("Wifi connected");
  needMqttConnect = true;
}

bool connectMqtt() {
  unsigned long now = millis();

  if(mqttServerValue [0] == '\0') {
    return false;
  }

  if (300000 > now - lastMqttConnectionAttempt)
  {
    // Do not repeat within 30 sec.
    return false;
  }
  Serial.println("Connecting to MQTT server...");
  if (!connectMqttOptions()) {
    lastMqttConnectionAttempt = now;
    return false;
  }

  if(mqttClient.connected()) {
    Serial.println("Connected!");
  } else {
    Serial.println("Still not connected.");
  }

  return true;
}

bool connectMqttOptions()
{
  bool result;
  if (mqttUserPasswordValue[0] != '\0')
  {
    result = mqttClient.connect(iotWebConf.getThingName(), mqttUserNameValue, mqttUserPasswordValue);
  }
  else if (mqttUserNameValue[0] != '\0')
  {
    result = mqttClient.connect(iotWebConf.getThingName(), mqttUserNameValue);
  }
  else
  {
    result = mqttClient.connect(iotWebConf.getThingName());
  }
  return result;
}

bool cb(Modbus::ResultCode event, uint16_t transactionId, void* data) { // Modbus Transaction callback
  Serial.printf("Modbus result: %02X\n", event); 
  modbusresult = event;
  if(event != Modbus::EX_SUCCESS) {                // If transaction got an error
    Serial.printf("Modbus result: %02X\n", event);  // Display Modbus error code
    batteryCurrent = 0;
    batterySOC = 0;
    pvCurrent = 0;
    vebusCurrent = 0;
    modbusErrorCounter++;
  }
  if (event == Modbus::EX_TIMEOUT) {    // If Transaction timeout took place
    mb.disconnect(remote);              // Close connection to slave and
    mb.dropTransactions();              // Cancel all waiting transactions
  }

  if(event == Modbus::EX_SUCCESS) {
    modbusSuccessCounter++;
    batterySOC = res[2];

    conv.uint_val = res[0];
    batteryCurrent = ((float)conv.int_val) / 10;

    conv.uint_val = res1;
    pvCurrent = ((float)conv.int_val) / 10;

    conv.uint_val = res2;
    vebusCurrent = ((float)conv.int_val) / 10;
  }
  return true;
}

void reconnect() {
  timeClient.begin();
  timeClient.setTimeOffset(utc_offset * 3600);
}

void onlineAction() {
  timeClient.update();
}


String getDateTime() {
  time_t epochTime = timeClient.getEpochTime();
  struct tm *ptm = gmtime ((time_t *)&epochTime); 

  String s = "";
  s += (ptm->tm_mday);
  s += ".";
  s += (ptm->tm_mon +1);
  s += ".";
  s += (ptm->tm_year+1900);
  s += " ";
  s += timeClient.getFormattedTime();

  return s;
}

/**
 * Handle web requests to "/" path.
 */
void handleRoot()
{
  // -- Let IotWebConf test and handle captive portal requests.
  if (iotWebConf.handleCaptivePortal())
  {
    // -- Captive portal request were already served.
    return;
  }
  String s = "<!DOCTYPE html><html lang=\"en\"><head><meta http-equiv=\"refresh\" content=\"60; URL=/\"><meta name=\"viewport\" content=\"width=device-width, initial-scale=1, user-scalable=no\"/>";
  s += "\n<title>GX Remote Heater Control</title></head>\n<body>\n<h3>Config Overview</h3>\n";
  s += "<ul>";
  s += "<li>GX IP Adress: ";
  s += remote.toString();
  s += "<li>Min SOC: ";
  s += minBatSOC;
  s += " &#037;<li>Min Current Heater A1: ";
  s += heaterCurrent[0];
  s += " A<li>Min Current Heater A2: ";
  s += heaterCurrent[1];
  s += " A<li>Min Current Heater A3: ";
  s += heaterCurrent[2];
  s += " A<li>Min Current Heater B1: ";
  s += heaterCurrent[3];
  s += " A<li>Min Current Heater B2: ";
  s += heaterCurrent[4];
  s += " A<li>Min Current Heater B3: ";
  s += heaterCurrent[5];
  s += " A<li>Buffer SOC: ";
  s += bufferSOC;
  s += " &#037;<li>Buffer Current: ";
  s += bufferCurrent;
  s += " A<li>Sunset Offset: ";
  s += sunsetOffset;
  s += " h</ul>\n<h3>Modbus Measurments</h3>\n<ul><li>PV Current: ";
  s += pvCurrent;
  s += " A<li>VE.Bus Current: ";
  s += vebusCurrent;
  s += " A<li>Actual SOC: ";
  s += batterySOC;
  s += " &#037;<li>Battery Current: ";
  s += batteryCurrent;
  s += " A</ul>\n<h3>Results</h3>\n<ul><li>Number of Active Heaters: ";
  s += numberOfActiveHeater;
  s += "<li>Estimated Current: ";
  s += estimatedCurrent;
  s += " A<li>Delta Current: ";
  s += pvCurrent + vebusCurrent;
  s += " A<li>Modbus Result: ";
  s += modbusresult;
  s += "<li>Modbus Success Counter: ";
  s += modbusSuccessCounter;
  s += "<li>Modbus Error Counter: ";
  s += modbusErrorCounter;
  s += "<li>IotWebConf State: ";
  s += iotWebConf.getState();
  s += "<li>A1 State: ";
  s += heaterEnable[0];
  s += "<li>A2 State: ";
  s += heaterEnable[1];
  s += "<li>A3 State: ";
  s += heaterEnable[2];
  s += "<li>B1 State: ";
  s += heaterEnable[3];
  s += "<li>B2 State: ";
  s += heaterEnable[4];
  s += "<li>B3 State: ";
  s += heaterEnable[5];
  s += "<li>Group A Enable: ";
  s += enableA;
  //s += "<li>Group B Enable: ";
  //s += enableB;
  s += "<li>Within valid sun range: ";
  s += inTimerange;
  
  s += "</ul></ul>\n<h3>Links</h3>\n<ul>\n";
  s += "<li> <a href=\"/man/a1\">Activate A1</a>";
  s += "<li> <a href=\"/man/a2\">Activate A2</a>";
  s += "<li> <a href=\"/man/a3\">Activate A3</a>";
  s += "<li> <a href=\"/man/b1\">Activate B1</a>";
  s += "<li> <a href=\"/man/b2\">Activate B2</a>";
  s += "<li> <a href=\"/man/b3\">Activate B3</a>";
  s += "<li> <a href=\"/api\">API</a>";
  s += "</ul><p>Aktuelles Datum: ";
  s += getDateTime();
  s += "</p>\n<p>Tageszeit: ";
  s += timeClient.getHours() + ((float)timeClient.getMinutes() / 60.0);
  s += "</p>\n";
  s += "<p>Sunrise: ";
  s += sunrise + utc_offset;
  s += "</p>\n";
  s += "<p>Sunset: ";
  s += sunset + utc_offset;
  s += "</p>\n";
  s += "<p>Uptime: ";
  s += millis() / 1000;
  s += " s</p>\n";
  
  s += "<p>Go to <a href='config'>configure page</a> to change values.</p>";
  s += "</body></html>\n";

  server.send(200, "text/html", s);
}

void handleAPI() {

  String s;

  s = getJSONStatus();

  server.send(200, "application/json", s);
}

String getJSONStatus() {
  uint8_t heaters = 0;
  String msg;

  for(int i=0; i<6; i++) {
    if(heaterEnable[i]) {
      heaters += (1<<i);
    }
  }

  DynamicJsonDocument doc(1024);

  doc["pvCurr"] = pvCurrent;
  doc["vebusCurr"] = vebusCurrent;
  doc["batSOC"] = batterySOC;
  doc["batCurr"] = batteryCurrent;
  doc["numAct"] = numberOfActiveHeater;
  doc["estCurr"] = estimatedCurrent;
  doc["enA"] = enableA;
  doc["mbres"] = modbusresult;
  doc["heaters"] = heaters;

  serializeJson(doc, msg);

  return msg;
}


void sendMqttStatus() {
  String topic;
  String msg;

  if(!mqttClient.connected()) return;
  
  topic = "";
  topic += mqttTopicPrefixValue;
  topic += "/status";

  msg = getJSONStatus();

  Serial.print("publish mqtt message to: ");
  Serial.print(topic);
  Serial.print(" with Message ");
  Serial.println(msg);
  
  mqttClient.publish(topic, msg.c_str());
}

void configSaved()
{
  Serial.println("Configuration was updated.");
  needReset = true;
  
}

bool formValidator(iotwebconf::WebRequestWrapper* webRequestWrapper)
{
  Serial.println("Validating form.");
  bool valid = multipleWifiAddition.formValidator(webRequestWrapper);

/*
  int l = webRequestWrapper->arg(stringParam.getId()).length();
  if (l < 3)
  {
    stringParam.errorMessage = "Please provide at least 3 characters for this test!";
    valid = false;
  }
*/
  return valid;
}

void manEnableA1() {
  t_man_activate[0] = millis();
  server.send(200, "text/html","<!DOCTYPE html><html lang=\"en\"><head><meta http-equiv=\"refresh\" content=\"2; URL=/\"></head><body>Pin activated</body></html>");
}

void manEnableA2() {
  t_man_activate[1] = millis();
  server.send(200, "text/html","<!DOCTYPE html><html lang=\"en\"><head><meta http-equiv=\"refresh\" content=\"2; URL=/\"></head><body>Pin activated</body></html>");
}

void manEnableA3() {
  t_man_activate[2] = millis();
  server.send(200, "text/html","<!DOCTYPE html><html lang=\"en\"><head><meta http-equiv=\"refresh\" content=\"2; URL=/\"></head><body>Pin activated</body></html>");
}

void manEnableB1() {
  t_man_activate[3] = millis();
  server.send(200, "text/html","<!DOCTYPE html><html lang=\"en\"><head><meta http-equiv=\"refresh\" content=\"2; URL=/\"></head><body>Pin activated</body></html>");
}

void manEnableB2() {
  t_man_activate[4] = millis();
  server.send(200, "text/html","<!DOCTYPE html><html lang=\"en\"><head><meta http-equiv=\"refresh\" content=\"2; URL=/\"></head><body>Pin activated</body></html>");
}

void manEnableB3() {
  t_man_activate[5] = millis();
  server.send(200, "text/html","<!DOCTYPE html><html lang=\"en\"><head><meta http-equiv=\"refresh\" content=\"2; URL=/\"></head><body>Pin activated</body></html>");
}