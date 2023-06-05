

/* code zet mbv de DAC een spanning over een elektrode. vervolgens leest de ADC een spanning uit en rekent dit om naar een stroom. */

//#define DEBUG       // uncommend for debug printouts.

#include <Adafruit_ADS1X15.h>
#include "DAC80501.h"
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <SPIFFS.h>

//defines voor DAC
#define LSB_DAC (5000 / pow(2, 16))
#define DACOFFSET 2049
#define SPI1_SCK 18   // Clock
#define SPI1_MISO 19  // fuer dac nur dummy def
#define SPI1_MOSI 23  // Master Output Slave Input
#define SPI1_CS 5     // Chip-Select
DAC80501 dac;

//defines voor ADC
#define LSB_ADC 62.5E-6
#define UPPER_LIMIT (pow(2,15)-600)  //    //2^15 -700
#define LOWER_LIMIT 680              //
Adafruit_ADS1115 ads;

#ifdef DEBUG
  #define LED 21
#endif
//global variables:

const char* ssid = "ACCESPOINT";
//const char* password = "Your_AP_Password";
//esther asdfa
AsyncWebServer server(80);

struct SensorData {
  int value1;
  int value2;
};

struct ReceivedData {
  int startPot;
  int endPot;
  int totalTime;
  float frequency;
  int amplitude;
};


struct ReceivedData parameters;

bool saveDataButtonState = false;
bool startMeasurementButtonState = false;

//Mux pins
const int enablePin = 12;
const int A1Pin = 4;
const int A0Pin = 16;

// variables voor stroommeeting
uint8_t huidige_state = 3;
long weerstand;
int16_t results;
float offset = 0;
float currentHigh = 0;
float currentLow = 0;
int measurementIndex = 0;

//variables voor squarewave
hw_timer_t *Timer0_Cfg = NULL;
int potDiff = 0;
int stepSize = 0;
int rampPot = 0;
int nstep = 0;
float period = 0;
int dir = 1;        // keeps track if there needs to be a positive or negative puls
int amplitude = 0;
int endPot = 0;
bool isrTrigger = false;  // event trigger variable
bool done = true;   // keeps track if the measurment is done or still going.

portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

struct IV {
  int spanning;
  float stroom;
};

//functie prototypes
void switchTMUX1109(uint8_t state);
void setDacValue(int spanning);
float measure();
void IRAM_ATTR Timer0_ISR();
void timerForInterrupt(uint64_t timeInMs);
void squareWaveVoltammetry(struct ReceivedData parameters);

const int dataSize = 1000;
IV data[dataSize];

void setup(void) {

  parameters.startPot = 0;
  parameters.endPot = 1500;
  parameters.totalTime = 10000;
  parameters.frequency =10;
  parameters.amplitude =0;

  Serial.begin(115200);

  pinMode(enablePin, OUTPUT);
  pinMode(A1Pin, OUTPUT);
  pinMode(A0Pin, OUTPUT);

  #ifdef DEBUG
    pinMode(LED, OUTPUT);
  #endif

  Timer0_Cfg = timerBegin(0, 80, true);
  timerAttachInterrupt(Timer0_Cfg, &Timer0_ISR, true);
  timerAlarmWrite(Timer0_Cfg, 1000000, true);
  timerAlarmEnable(Timer0_Cfg);

  //begin DAC
  dac.begin(&SPI, SPI1_SCK, SPI1_MISO, SPI1_MOSI, SPI1_CS);
  dac.setREG4_DivGain(SET4_DIV1, SET4_GAIN2);
  setDacValue(DACOFFSET+100);
  delay(100);  

 switchTMUX1109(huidige_state);

 if (!ads.begin()) {
   Serial.println("Failed to initialize ADS.");
   while (1);
 }
  

 ads.setDataRate(RATE_ADS1115_32SPS);
 ads.startADCReading(ADS1X15_REG_CONFIG_MUX_DIFF_0_1, /*continuous=*/true);
 int test = ads.readADC_Differential_0_1();


  WiFi.softAP(ssid);
  IPAddress IP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(IP);

  SPIFFS.begin();

  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(SPIFFS, "/index.html", "text/html");
  });

  server.on("/Chart.js", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(SPIFFS, "/Chart.js", "application/javascript");
  });

  server.on("/sendData", HTTP_GET, [](AsyncWebServerRequest *request) {
    if (request->hasParam("startPot") && request->hasParam("endPot") && request->hasParam("totalTime") && request->hasParam("frequency") && request->hasParam("amplitude")) {
      parameters.startPot   = request->getParam("startPot")->value().toInt();
      parameters.endPot     = request->getParam("endPot")->value().toInt();
      parameters.totalTime  = request->getParam("totalTime")->value().toInt();
      parameters.frequency  = request->getParam("frequency")->value().toInt();
      parameters.amplitude  = request->getParam("amplitude")->value().toInt();

      Serial.print("Received data: ");
      Serial.print(parameters.startPot);
     Serial.print(", ");
     Serial.print(parameters.endPot);
     Serial.print(", ");
     Serial.print(parameters.totalTime);
     Serial.print(", ");
     Serial.print(parameters.frequency);
     Serial.print(", ");
     Serial.println(parameters.amplitude);
      
      request->send(200, "text/plain", "Data received!");
    } else {
      request->send(400);
    }
  });

  server.on("/getData", HTTP_GET, [](AsyncWebServerRequest *request) {
    String responseData;
    if(done==false){
    for (int i = 1; i < dataSize; i++) {
      if(data[i].spanning != 0 && data[i].stroom !=0){
        responseData += String(data[i].spanning) + "," + String(data[i].stroom) + ",";
      }
    }
    request->send(200, "text/plain", responseData);
    }else{
      request->send(204,"text/plain", "");

    }
});

  server.on("/saveData", HTTP_GET, [](AsyncWebServerRequest *request) {
   Serial.println("Save data button is pressed");
    saveDataButtonState = !saveDataButtonState; // Toggle button state
    request->send(200, "text/plain", String(saveDataButtonState));
  });

  server.on("/startMeasurement", HTTP_GET, [](AsyncWebServerRequest *request) {
    Serial.println("Start Measurement button is pressed");
    startMeasurementButtonState = true; // Toggle button state
    request->send(200, "text/plain", String(startMeasurementButtonState));
  });

  server.begin();
}


void loop(void) {
  yield();
  if(startMeasurementButtonState == true){
    squareWaveVoltammetry(parameters);
    startMeasurementButtonState = false;
  }
  
  if (isrTrigger) {

   if (rampPot < endPot && done == false) {

     #ifdef DEBUG
       digitalWrite(LED, !digitalRead(LED));
       //Serial.println("triggerd");
     #endif

     if (dir == 1) {
       currentHigh = measure();
       rampPot += stepSize;
       setDacValue(rampPot + amplitude + DACOFFSET);
       #ifdef DEBUG
         Serial.println("");
         Serial.print(" currenthigh ");
         Serial.print(currentHigh);
         Serial.print(" voltagehigh ");
         Serial.println(rampPot + amplitude + DACOFFSET);
       #endif
       dir = 0;
     } else if (dir == 0) {
       currentLow = measure();
       setDacValue(rampPot - amplitude + DACOFFSET);
       #ifdef DEBUG
         Serial.print(" currentLow ");
         Serial.print(currentLow);
         Serial.print(" voltageLow ");
         Serial.println(rampPot - amplitude + DACOFFSET);
       #endif
       data[measurementIndex].spanning = rampPot;
       data[measurementIndex].stroom = (currentHigh + currentLow) *0.5;
       #ifdef DEBUG
         Serial.print(" RampPot ");
         Serial.print(data[measurementIndex].spanning);
         Serial.print(" current ");
         Serial.println(data[measurementIndex].stroom);
       #endif
       measurementIndex++;
       dir = 1;
     }
    }
    
    portENTER_CRITICAL_ISR(&timerMux);
    isrTrigger = false;
    portEXIT_CRITICAL_ISR(&timerMux);
  }

   if (rampPot >= endPot && done == false){
      measurementIndex = 0;
      memset(data, '\0', sizeof(data)); 
      done = true;
   }
}


void IRAM_ATTR Timer0_ISR() {
  portENTER_CRITICAL_ISR(&timerMux);
  isrTrigger = true;
  portEXIT_CRITICAL_ISR(&timerMux);
}

void timerForInterrupt(uint64_t timeInMs) {
  portENTER_CRITICAL(&timerMux);
  timerAlarmWrite(Timer0_Cfg, timeInMs, true);
  portEXIT_CRITICAL(&timerMux);
}

void squareWaveVoltammetry(struct ReceivedData parameters) {
  #ifdef DEBUG
  Serial.println("endPot:");
  Serial.println(parameters.endPot);
  Serial.println("startPot:");
  Serial.println(parameters.startPot);
  Serial.println("totalTime:");
  Serial.println(parameters.totalTime);
  Serial.println("frequency:");
  Serial.println(parameters.frequency);
  Serial.println("amplitude:");
  Serial.println(parameters.amplitude);
  #endif

  potDiff = parameters.endPot - parameters.startPot;
  period = (1 / parameters.frequency) * 1000;
  nstep = parameters.totalTime / period;
  stepSize = potDiff / nstep;
  amplitude = parameters.amplitude;
  endPot = parameters.endPot;
  rampPot = parameters.startPot;
  done = false;
  setDacValue(rampPot+amplitude+stepSize+DACOFFSET);
  int temp = ads.readADC_Differential_0_1();

#ifdef DEBUG
  Serial.print("potDiff ");
  Serial.println(potDiff);
  Serial.print("period ");
  Serial.println(period);
  Serial.print("nstep ");
  Serial.println(nstep);
  Serial.print("stepSize ");
  Serial.println(stepSize);
#endif
timerForInterrupt((period / 2)*1000);
  return;
}

void setDacValue(int spanning) {
  if (spanning < DACOFFSET) {
    spanning == DACOFFSET;
  }
  dac.writeDAC(spanning / LSB_DAC);
}

/*
TMUX1109
EN  A1  A0
0   X   X   All channels are off
1   0   0   S1A and S1B
1   0   1   S2A and S2B
1   1   0   S3A and S3B
1   1   1   S4A and S4B
*/

void switchTMUX1109(uint8_t state) {
 switch (state) {
   case 1:  //S1
     digitalWrite(enablePin, 1);
     digitalWrite(A1Pin, 0);
     digitalWrite(A0Pin, 0);
     weerstand = 50E6;
     offset = 0;
     break;
   case 2:  //S2
     digitalWrite(enablePin, 1);
     digitalWrite(A1Pin, 0);
     digitalWrite(A0Pin, 1);
     weerstand = 12E5;  // verander weerstand voor stroom berekening
     offset = 0;
     break;
   case 3:  //S3
     digitalWrite(enablePin, 1);
     digitalWrite(A1Pin, 1);
     digitalWrite(A0Pin, 0);
     weerstand = 27700;  // verander weerstand voor stroom berekening
     offset = 0;
     break;
   case 4:  //S4
     digitalWrite(enablePin, 1);
     digitalWrite(A1Pin, 1);
     digitalWrite(A0Pin, 1);
     weerstand = 649;  // verander weerstand voor stroom berekening
     offset = -8000;
     break;
   default:
     digitalWrite(enablePin, 0);
     digitalWrite(A1Pin, 0);
     digitalWrite(A0Pin, 0);
     break;
 }
}

float measure() {
 do {
   results = ads.readADC_Differential_0_1() * -1;
   if (results < LOWER_LIMIT && huidige_state != 1 && dir ==1) {
     huidige_state--;
     switchTMUX1109(huidige_state);
     int temp = ads.readADC_Differential_0_1() * -1;
     //delay(10);

#ifdef DEBUG
     Serial.print("results = ");
     Serial.print(results);
     Serial.print("  mux geschakeld naar weerstand: ");
     Serial.print(weerstand);
     Serial.print("  state: ");
     Serial.println(huidige_state);
#endif

   } else if (results > UPPER_LIMIT && huidige_state != 4 && dir == 1) {
     huidige_state++;
     switchTMUX1109(huidige_state);
     int temp = ads.readADC_Differential_0_1() * -1;
     //delay(10);

#ifdef DEBUG
     Serial.print("results = ");
     Serial.print(results);
     Serial.print("  mux geschakeld naar weerstand: ");
     Serial.print(weerstand);
     Serial.print("  state: ");
     Serial.println(huidige_state);
#endif

   }
   if ((results < LOWER_LIMIT && huidige_state == 1) || (results > UPPER_LIMIT && huidige_state == 4) || dir != 1) {
     results = ads.readADC_Differential_0_1() * -1;
     #ifdef DEBUG
       Serial.print(" resultsB: ");
       Serial.println(results);
     #endif
     break;
   }
     #ifdef DEBUG
       Serial.print(" results: ");
       Serial.println(results);
     #endif
 } while (results < LOWER_LIMIT || results > UPPER_LIMIT);


 float ADC_spanning = (results * LSB_ADC);
 float stroom = ((ADC_spanning / weerstand) * 1E9);//+ offset;  // in nA


#ifdef DEBUG
 Serial.print(results);
 Serial.print(",");
 Serial.print(ADC_spanning);
 Serial.print("V ,");
 Serial.print(stroom);
 Serial.println("nA ,");
#endif

 return stroom;
}
