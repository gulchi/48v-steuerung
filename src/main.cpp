
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
#define CONFIG_VERSION "008"

// -- When CONFIG_PIN is pulled to ground on startup, the Thing will use the initial
//      password to buld an AP. (E.g. in case of lost password)
#define CONFIG_PIN D0

// -- Status indicator pin.
//      First it will light up (kept LOW), on Wifi connection it will blink,
//      when connected to the Wifi it will turn off (kept HIGH).
#define STATUS_PIN LED_BUILTIN

#define PIN_HEATER_1 D5
#define PIN_HEATER_2 D6
#define PIN_HEATER_3 D7
#define PIN_HEATER_4 D8

union conv 
{
  uint16_t uint_val;
  int16_t int_val;
} conv;

uint16_t targetPort = 502;
uint8_t targetSID = 1;
uint16_t addr = 1;
uint16_t words = 4;

#define REG_COUNT 4

void handleRoot();
// -- Callback methods.
void configSaved();
bool formValidator(iotwebconf::WebRequestWrapper* webRequestWrapper);

bool cb(Modbus::ResultCode event, uint16_t transactionId, void* data);

DNSServer dnsServer;
WebServer server(80);

char intParamValueminSOC[NUMBER_LEN];

char intParamValueCurr1[NUMBER_LEN];
char intParamValueCurr2[NUMBER_LEN];
char intParamValueCurr3[NUMBER_LEN];
char intParamValueCurr4[NUMBER_LEN];

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

int currentHeater1;
int currentHeater2;
int currentHeater3;
int currentHeater4;

int numberOfActiveHeater = 0;

int minBatSOC;


IotWebConf iotWebConf(thingName, &dnsServer, &server, wifiInitialApPassword, CONFIG_VERSION);
// -- You can also use namespace formats e.g.: iotwebconf::TextParameter

IotWebConfParameterGroup group1 = IotWebConfParameterGroup("group1", "Cerbo");
IotWebConfNumberParameter intParamIP1 = IotWebConfNumberParameter("IP Adress Part 1", "ipaddr1", intParamValueIP1, NUMBER_LEN, "192", "0..255", "min='1' max='255' step='1'");
IotWebConfNumberParameter intParamIP2 = IotWebConfNumberParameter("IP Adress Part 2", "ipaddr2", intParamValueIP2, NUMBER_LEN, "168", "0..255", "min='1' max='255' step='1'");
IotWebConfNumberParameter intParamIP3 = IotWebConfNumberParameter("IP Adress Part 3", "ipaddr3", intParamValueIP3, NUMBER_LEN, "1", "0..255", "min='1' max='255' step='1'");
IotWebConfNumberParameter intParamIP4 = IotWebConfNumberParameter("IP Adress Part 4", "ipaddr4", intParamValueIP4, NUMBER_LEN, "41", "0..255", "min='1' max='255' step='1'");


// -- We can add a legend to the separator
IotWebConfParameterGroup group2 = IotWebConfParameterGroup("group2", "Current");
IotWebConfNumberParameter intParamMinSOC = IotWebConfNumberParameter("Min SOC", "minSOC", intParamValueminSOC, NUMBER_LEN, "20", "1..100", "min='1' max='100' step='1'");
IotWebConfNumberParameter intParamMinCurr1 = IotWebConfNumberParameter("Min Current Heater 1", "minCurrent1", intParamValueCurr1, NUMBER_LEN, "20", "1..100", "min='1' max='100' step='1'");
IotWebConfNumberParameter intParamMinCurr2 = IotWebConfNumberParameter("Min Current Heater 2", "minCurrent2", intParamValueCurr2, NUMBER_LEN, "36", "1..100", "min='1' max='100' step='1'");
IotWebConfNumberParameter intParamMinCurr3 = IotWebConfNumberParameter("Min Current Heater 3", "minCurrent3", intParamValueCurr3, NUMBER_LEN, "52", "1..100", "min='1' max='100' step='1'");
IotWebConfNumberParameter intParamMinCurr4 = IotWebConfNumberParameter("Min Current Heater 4", "minCurrent4", intParamValueCurr4, NUMBER_LEN, "68", "1..100", "min='1' max='100' step='1'");


float batteryCurrent = 0;
float batterySOC = 0;
float estimatedCurrent = 0;

unsigned long modbusSuccessCounter = 0;
unsigned long modbusErrorCounter = 0;


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
  
  group2.addItem(&intParamMinCurr1);
  group2.addItem(&intParamMinCurr2);
  group2.addItem(&intParamMinCurr3);
  group2.addItem(&intParamMinCurr4);


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

  pinMode(PIN_HEATER_1, OUTPUT);
  pinMode(PIN_HEATER_2, OUTPUT);
  pinMode(PIN_HEATER_3, OUTPUT);
  pinMode(PIN_HEATER_4, OUTPUT);

  remote = IPAddress(atoi(intParamValueIP1), atoi(intParamValueIP2), atoi(intParamValueIP3), atoi(intParamValueIP4));

  currentHeater1 = atoi(intParamValueCurr1);
  currentHeater2 = atoi(intParamValueCurr2);
  currentHeater3 = atoi(intParamValueCurr3);
  currentHeater4 = atoi(intParamValueCurr4);

  minBatSOC = atoi(intParamValueminSOC);
}

void loop() 
{
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
        mb.readIreg(remote, addr, res, REG_COUNT, cb, targetSID);
      } else {
        Serial.println("Reconnect to Modbus Server");
        mb.connect(remote);
      }
    }
  }

  if(timediff2 > 2*modbusTimer) {
    lastHeaterCall = currentTime;
    Serial.println("Update output pins");

    estimatedCurrent = batteryCurrent + (16*numberOfActiveHeater);

    numberOfActiveHeater = 0;

    if(batteryCurrent > 0 && estimatedCurrent > currentHeater1) {
      digitalWrite(PIN_HEATER_1, HIGH);
      numberOfActiveHeater++;
    } else {
      digitalWrite(PIN_HEATER_1, LOW);
    }

    if(batteryCurrent > 0 && estimatedCurrent > currentHeater2 && batterySOC > minBatSOC) {
      digitalWrite(PIN_HEATER_2, HIGH);
      numberOfActiveHeater++;
    } else {
      digitalWrite(PIN_HEATER_2, LOW);
    }

    if(batteryCurrent > 0 && estimatedCurrent > currentHeater3 && batterySOC > minBatSOC) {
      digitalWrite(PIN_HEATER_3, HIGH);
      numberOfActiveHeater++;
    } else {
      digitalWrite(PIN_HEATER_3, LOW);
    }

    if(batteryCurrent > 0 && estimatedCurrent > currentHeater4 && batterySOC > minBatSOC) {
      digitalWrite(PIN_HEATER_4, HIGH);
      numberOfActiveHeater++;
    } else {
      digitalWrite(PIN_HEATER_4, LOW);
    }

  }
}

bool cb(Modbus::ResultCode event, uint16_t transactionId, void* data) { // Modbus Transaction callback
  Serial.printf("Modbus result: %02X\n", event); 
  modbusresult = event;
  if(event != Modbus::EX_SUCCESS) {                // If transaction got an error
    Serial.printf("Modbus result: %02X\n", event);  // Display Modbus error code
    batteryCurrent = 0;
    batterySOC = 0;
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
  s += "<li>Min Current Heater 1: ";
  s += currentHeater1;
  s += "<li>Min Current Heater 2: ";
  s += currentHeater2;
  s += "<li>Min Current Heater 3: ";
  s += currentHeater3;
  s += "<li>Min Current Heater 4: ";
  s += currentHeater4;
  s += "<li>Actual SOC: ";
  s += batterySOC;
  s += "<li>Actual Current: ";
  s += batteryCurrent;
  s += "<li>Number of Active Heaters: ";
  s += numberOfActiveHeater;
  s += "<li>Estimated Current: ";
  s += estimatedCurrent;
  s += "<li>Modbus Result: ";
  s += modbusresult;
  s += "<li>Modbus Success Counter: ";
  s += modbusSuccessCounter;
  s += "<li>Modbus Error Counter: ";
  s += modbusErrorCounter;
  s += "<li>IotWebConf State: ";
  s += iotWebConf.getState();
  s += "</ul>";
  s += "Go to <a href='config'>configure page</a> to change values.";
  s += "</body></html>\n";

  server.send(200, "text/html", s);
}

void configSaved()
{
  Serial.println("Configuration was updated.");

  remote = IPAddress(atoi(intParamValueIP1), atoi(intParamValueIP2), atoi(intParamValueIP3), atoi(intParamValueIP4));

  currentHeater1 = atoi(intParamValueCurr1);
  currentHeater2 = atoi(intParamValueCurr2);
  currentHeater3 = atoi(intParamValueCurr3);
  currentHeater4 = atoi(intParamValueCurr4);
  minBatSOC = atoi(intParamValueminSOC);
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