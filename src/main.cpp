
#define IOTWEBCONF_DEBUG_PWD_TO_SERIAL 1
#define IOTWEBCONF_WORD_LEN 70
#define IOTWEBCONF_PASSWORD_LEN 70

#include <Arduino.h>
#include <IotWebConf.h>
#include <IotWebConfUsing.h>
#include <ModbusTCP.h>

#if defined(ESP8266)
  #include <ESP8266WiFi.h> 
#elif defined(ESP32)
  #include <WiFi.h>
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
#define CONFIG_VERSION "011"

// -- When CONFIG_PIN is pulled to ground on startup, the Thing will use the initial
//      password to buld an AP. (E.g. in case of lost password)
#define CONFIG_PIN D0

// -- Status indicator pin.
//      First it will light up (kept LOW), on Wifi connection it will blink,
//      when connected to the Wifi it will turn off (kept HIGH).
#define STATUS_PIN LED_BUILTIN

#define PIN_HEATER_A1 D5
#define PIN_HEATER_A2 D6
#define PIN_HEATER_A3 D7
#define PIN_HEATER_B1 D8
#define PIN_HEATER_B2 D4
#define PIN_HEATER_B3 D3

#define ENABLE_PIN_A D2
#define ENABLE_PIN_B D1


union conv 
{
  uint16_t uint_val;
  int16_t int_val;
} conv;


uint8_t targetSID = 100;


#define REG_COUNT 4

void handleRoot();
// -- Callback methods.
void configSaved();
bool formValidator(iotwebconf::WebRequestWrapper* webRequestWrapper);

bool cb(Modbus::ResultCode event, uint16_t transactionId, void* data);

DNSServer dnsServer;
WebServer server(80);

char intParamValueminSOC[NUMBER_LEN];

char intParamValueCurrA1[NUMBER_LEN];
char intParamValueCurrA2[NUMBER_LEN];
char intParamValueCurrA3[NUMBER_LEN];
char intParamValueCurrB1[NUMBER_LEN];
char intParamValueCurrB2[NUMBER_LEN];
char intParamValueCurrB3[NUMBER_LEN];

char intParamValueBufferSOC[NUMBER_LEN];
char intParamValueBufferCurrent[NUMBER_LEN];

char intParamValueIP1[NUMBER_LEN];
char intParamValueIP2[NUMBER_LEN];
char intParamValueIP3[NUMBER_LEN];
char intParamValueIP4[NUMBER_LEN];


unsigned long lastModbusCall = 0;
unsigned long lastHeaterCall = 0;
unsigned long modbusTimer = 10 * 1000;

IPAddress remote;
Modbus::ResultCode modbusresult;

uint16 res[REG_COUNT];
uint16 res1;
uint16 res2;

int currentHeaterA1;
int currentHeaterA2;
int currentHeaterA3;
int currentHeaterB1;
int currentHeaterB2;
int currentHeaterB3;


int numberOfActiveHeater = 0;

int minBatSOC;


IotWebConf iotWebConf(thingName, &dnsServer, &server, wifiInitialApPassword, CONFIG_VERSION);
// -- You can also use namespace formats e.g.: iotwebconf::TextParameter

IotWebConfParameterGroup group1 = IotWebConfParameterGroup("group1", "Cerbo");
IotWebConfNumberParameter intParamIP1 = IotWebConfNumberParameter("IP Adress Part 1", "ipaddr1", intParamValueIP1, NUMBER_LEN, "192", "0..255", "min='0' max='255' step='1'");
IotWebConfNumberParameter intParamIP2 = IotWebConfNumberParameter("IP Adress Part 2", "ipaddr2", intParamValueIP2, NUMBER_LEN, "168", "0..255", "min='0' max='255' step='1'");
IotWebConfNumberParameter intParamIP3 = IotWebConfNumberParameter("IP Adress Part 3", "ipaddr3", intParamValueIP3, NUMBER_LEN, "1", "0..255", "min='0' max='255' step='1'");
IotWebConfNumberParameter intParamIP4 = IotWebConfNumberParameter("IP Adress Part 4", "ipaddr4", intParamValueIP4, NUMBER_LEN, "41", "0..255", "min='0' max='255' step='1'");


// -- We can add a legend to the separator
IotWebConfParameterGroup group2 = IotWebConfParameterGroup("group2", "Current");
IotWebConfNumberParameter intParamMinSOC = IotWebConfNumberParameter("Min SOC", "minSOC", intParamValueminSOC, NUMBER_LEN, "20", "1..100", "min='1' max='100' step='1'");
IotWebConfNumberParameter intParamMinCurrA1 = IotWebConfNumberParameter("Current Heater A1", "usedCurrentA1", intParamValueCurrA1, NUMBER_LEN, "0", "0..100", "min='0' max='100' step='1'");
IotWebConfNumberParameter intParamMinCurrA2 = IotWebConfNumberParameter("Current Heater A2", "usedCurrentA2", intParamValueCurrA2, NUMBER_LEN, "0", "0..100", "min='0' max='100' step='1'");
IotWebConfNumberParameter intParamMinCurrA3 = IotWebConfNumberParameter("Current Heater A3", "usedCurrentA3", intParamValueCurrA3, NUMBER_LEN, "0", "0..100", "min='0' max='100' step='1'");
IotWebConfNumberParameter intParamMinCurrB1 = IotWebConfNumberParameter("Current Heater B1", "usedCurrentB1", intParamValueCurrB1, NUMBER_LEN, "0", "0..100", "min='0' max='100' step='1'");
IotWebConfNumberParameter intParamMinCurrB2 = IotWebConfNumberParameter("Current Heater B2", "usedCurrentB2", intParamValueCurrB2, NUMBER_LEN, "0", "0..100", "min='0' max='100' step='1'");
IotWebConfNumberParameter intParamMinCurrB3 = IotWebConfNumberParameter("Current Heater B3", "usedCurrentB3", intParamValueCurrB3, NUMBER_LEN, "0", "0..100", "min='0' max='100' step='1'");

IotWebConfNumberParameter intParamBufferSOC = IotWebConfNumberParameter("Buffer SOC", "buffersoc", intParamValueBufferSOC, NUMBER_LEN, "90", "0..100", "min='0' max='100' step='1'");
IotWebConfNumberParameter intParamBufferCurrent = IotWebConfNumberParameter("Buffer Current", "buffercurrent", intParamValueBufferCurrent, NUMBER_LEN, "24", "0..100", "min='0' max='100' step='1'");


float batteryCurrent = 0;
float batterySOC = 0;
float estimatedCurrent = 0;
float pvCurrent = 0;
float vebusCurrent = 0;

float bufferSOC;
float bufferCurrent;

unsigned long modbusSuccessCounter = 0;
unsigned long modbusErrorCounter = 0;

bool ha1 = false;
bool ha2 = false;
bool ha3 = false;
bool hb1 = false;
bool hb2 = false;
bool hb3 = false;

bool enableA = false;
bool enableB = false;

ModbusTCP mb;  //ModbusTCP object

void setup() 
{
  Serial.begin(115200);
  Serial.println();
  Serial.println("Starting up...");

  pinMode(CONFIG_PIN, INPUT_PULLUP);

  delay(50);

  group1.addItem(&intParamIP1);
  group1.addItem(&intParamIP2);
  group1.addItem(&intParamIP3);
  group1.addItem(&intParamIP4);


  group2.addItem(&intParamMinSOC);
  
  group2.addItem(&intParamMinCurrA1);
  group2.addItem(&intParamMinCurrA2);
  group2.addItem(&intParamMinCurrA3);
  group2.addItem(&intParamMinCurrB1);
  group2.addItem(&intParamMinCurrB2);
  group2.addItem(&intParamMinCurrB3);
  group2.addItem(&intParamBufferSOC);
  group2.addItem(&intParamBufferCurrent);


  iotWebConf.setStatusPin(STATUS_PIN);
  iotWebConf.setConfigPin(CONFIG_PIN);

  iotWebConf.addParameterGroup(&group1);
  iotWebConf.addParameterGroup(&group2);
  iotWebConf.setConfigSavedCallback(&configSaved);
  iotWebConf.setFormValidator(&formValidator);
  iotWebConf.getApTimeoutParameter()->visible = true;

  // -- Initializing the configuration.
  iotWebConf.init();

  // -- Set up required URL handlers on the web server.
  server.on("/", handleRoot);
  server.on("/config", []{ iotWebConf.handleConfig(); });
  server.onNotFound([](){ iotWebConf.handleNotFound(); });

  Serial.println("Ready.");

  mb.client();

  pinMode(PIN_HEATER_A1, OUTPUT);
  pinMode(PIN_HEATER_A2, OUTPUT);
  pinMode(PIN_HEATER_A3, OUTPUT);
  pinMode(PIN_HEATER_B1, OUTPUT);
  pinMode(PIN_HEATER_B2, OUTPUT);
  pinMode(PIN_HEATER_B3, OUTPUT);

  pinMode(ENABLE_PIN_A, INPUT_PULLUP);
  pinMode(ENABLE_PIN_B, INPUT_PULLUP);

  remote = IPAddress(atoi(intParamValueIP1), atoi(intParamValueIP2), atoi(intParamValueIP3), atoi(intParamValueIP4));

  currentHeaterA1 = atoi(intParamValueCurrA1);
  currentHeaterA2 = atoi(intParamValueCurrA2);
  currentHeaterA3 = atoi(intParamValueCurrA3);
  currentHeaterB1 = atoi(intParamValueCurrB1);
  currentHeaterB2 = atoi(intParamValueCurrB2);
  currentHeaterB3 = atoi(intParamValueCurrB3);

  minBatSOC = atoi(intParamValueminSOC);

  bufferSOC = atoi(intParamValueBufferSOC);
  bufferCurrent = atoi(intParamValueBufferCurrent);
}

void loop() 
{
  float remainingCurrent;
  unsigned long currentTime = millis();
  unsigned long timediff;
  unsigned long timediff2;

  
  
  // -- doLoop should be called as frequently as possible.
  iotWebConf.doLoop();
  mb.task();

  timediff = (currentTime > lastModbusCall) ? currentTime - lastModbusCall : lastModbusCall - currentTime;
  timediff2 = (currentTime > lastHeaterCall) ? currentTime - lastHeaterCall : lastHeaterCall - currentTime;

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
  }

  enableA = digitalRead(ENABLE_PIN_A) == LOW;
  enableB = digitalRead(ENABLE_PIN_B) == LOW;

  if(timediff2 > 2*modbusTimer) {
    lastHeaterCall = currentTime;
    Serial.println("Update output pins");

    estimatedCurrent = pvCurrent + vebusCurrent;
    
    if(vebusCurrent > 0) estimatedCurrent = pvCurrent;

    remainingCurrent = estimatedCurrent;
    numberOfActiveHeater = 0;

    if(batterySOC > bufferSOC) {
      if(remainingCurrent < 0) remainingCurrent = 0;

      remainingCurrent += bufferCurrent;
    }

    if(batterySOC > minBatSOC) {

      if(remainingCurrent > currentHeaterA3 && currentHeaterA3 > 0 && batteryCurrent > 0 && enableA) {
        remainingCurrent -= currentHeaterA3;
        numberOfActiveHeater++;
        ha3 = true;
      } else {
        ha3 = false;
      }

      if(remainingCurrent > currentHeaterA2 && currentHeaterA2 > 0 && batteryCurrent > 0 && enableA) {
        remainingCurrent -= currentHeaterA2;
        numberOfActiveHeater++;
        ha2 = true;
      } else {
        ha2 = false;
      }

      if(remainingCurrent > currentHeaterA1 && currentHeaterA1 > 0 && batteryCurrent > 0 && enableA) {
        remainingCurrent -= currentHeaterA1;
        numberOfActiveHeater++;
        ha1 = true;
      } else {
        ha1 = false;
      }

      if(remainingCurrent > currentHeaterB3 && currentHeaterB3 > 0 && batteryCurrent > 0 && enableB) {
        remainingCurrent -= currentHeaterB3;
        numberOfActiveHeater++;
        hb3 = true;
      } else {
        hb3 = false;
      }

      if(remainingCurrent > currentHeaterB2 && currentHeaterB2 > 0 && batteryCurrent > 0 && enableB) {
        remainingCurrent -= currentHeaterB2;
        numberOfActiveHeater++;
        hb2 = true;
      } else {
        hb2 = false;
      }

      if(remainingCurrent > currentHeaterB1 && currentHeaterB1 > 0 && batteryCurrent > 0 && enableB) {
        remainingCurrent -= currentHeaterB1;
        numberOfActiveHeater++;
        hb1 = true;
      } else {
        hb1 = false;
      }

    } else {
      ha1 = false;
      ha2 = false;
      ha3 = false;
      hb1 = false;
      hb2 = false;
      hb3 = false;
    }
  }

  if(!enableA) {
    ha1 = false;
    ha2 = false;
    ha3 = false;
  }

  if(!enableB) {
    hb1 = false;
    hb2 = false;
    hb3 = false;
  }

  if(ha1) digitalWrite(PIN_HEATER_A1, HIGH);
  else digitalWrite(PIN_HEATER_A1, LOW);

  if(ha2) digitalWrite(PIN_HEATER_A2, HIGH);
  else digitalWrite(PIN_HEATER_A2, LOW);

  if(ha3) digitalWrite(PIN_HEATER_A3, HIGH);
  else digitalWrite(PIN_HEATER_A3, LOW);

  if(hb1) digitalWrite(PIN_HEATER_B1, HIGH);
  else digitalWrite(PIN_HEATER_B1, LOW);

  if(hb2) digitalWrite(PIN_HEATER_B2, HIGH);
  else digitalWrite(PIN_HEATER_B2, LOW);

  if(hb3) digitalWrite(PIN_HEATER_B3, HIGH);
  else digitalWrite(PIN_HEATER_B3, LOW);

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
  String s = "<!DOCTYPE html><html lang=\"en\"><head><meta name=\"viewport\" content=\"width=device-width, initial-scale=1, user-scalable=no\"/>";
  s += "<title>GX Remote Heater Control</title></head><body>Hello world!";
  s += "<ul>";
  s += "<li>GX IP Adress: ";
  s += remote.toString();
  s += "<li>Min SOC: ";
  s += minBatSOC;
  s += " &#037;<li>Min Current Heater A1: ";
  s += currentHeaterA1;
  s += " A<li>Min Current Heater A2: ";
  s += currentHeaterA2;
  s += " A<li>Min Current Heater A3: ";
  s += currentHeaterA3;
  s += " A<li>Min Current Heater B1: ";
  s += currentHeaterB1;
  s += " A<li>Min Current Heater B2: ";
  s += currentHeaterB2;
  s += " A<li>Min Current Heater B3: ";
  s += currentHeaterB3;
  s += " A<li>Actual SOC: ";
  s += batterySOC;
  s += " &#037;<li>Battery Current: ";
  s += batteryCurrent;
  s += " A<li>Buffer SOC: ";
  s += bufferSOC;
  s += " &#037;<li>Buffer Current: ";
  s += bufferCurrent;
  s += " A<li>PV Current: ";
  s += pvCurrent;
  s += " A<li>VE.Bus Current: ";
  s += vebusCurrent;
  s += " A<li>Number of Active Heaters: ";
  s += numberOfActiveHeater;
  s += "<li>Estimated Current: ";
  s += estimatedCurrent;
  s += " A<li>Modbus Result: ";
  s += modbusresult;
  s += "<li>Modbus Success Counter: ";
  s += modbusSuccessCounter;
  s += "<li>Modbus Error Counter: ";
  s += modbusErrorCounter;
  s += "<li>IotWebConf State: ";
  s += iotWebConf.getState();
  s += "<li>A1 State: ";
  s += ha1;
  s += "<li>A2 State: ";
  s += ha2;
  s += "<li>A3 State: ";
  s += ha3;
  s += "<li>B1 State: ";
  s += hb1;
  s += "<li>B2 State: ";
  s += hb2;
  s += "<li>B3 State: ";
  s += hb3;
  s += "<li>Group A Enable: ";
  s += enableA;
  s += "<li>Group B Enable: ";
  s += enableB;
  
  s += "</ul>";
  s += "Go to <a href='config'>configure page</a> to change values.";
  s += "</body></html>\n";

  server.send(200, "text/html", s);
}

void configSaved()
{
  Serial.println("Configuration was updated.");

  remote = IPAddress(atoi(intParamValueIP1), atoi(intParamValueIP2), atoi(intParamValueIP3), atoi(intParamValueIP4));

  currentHeaterA1 = atoi(intParamValueCurrA1);
  currentHeaterA2 = atoi(intParamValueCurrA2);
  currentHeaterA3 = atoi(intParamValueCurrA3);
  currentHeaterB1 = atoi(intParamValueCurrB1);
  currentHeaterB2 = atoi(intParamValueCurrB2);
  currentHeaterB3 = atoi(intParamValueCurrB3);
  minBatSOC = atoi(intParamValueminSOC);
  bufferSOC = atoi(intParamValueBufferSOC);
  bufferCurrent = atoi(intParamValueBufferCurrent);
}

bool formValidator(iotwebconf::WebRequestWrapper* webRequestWrapper)
{
  Serial.println("Validating form.");
  bool valid = true;

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