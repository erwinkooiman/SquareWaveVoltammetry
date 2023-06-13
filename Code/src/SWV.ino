
/* This code implements Squarewave Voltammetry using the DAC80501 to apply a voltage and the ADS1113 to measure the resulting current. */

//#define DEBUG       // Uncomment to enable debug printouts
//#define PRINTRESULTS // Uncomment to enable printing the results to the serial monitor


#include <Adafruit_ADS1X15.h>
#include <DAC80501.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <SPIFFS.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <SDcard.h>
#include "SD.h"

// defines for SD card 
#define SCK  14   // Clock
#define MISO  12  // Master Input Slave Output 
#define MOSI  13  // Master Output Slave Input 
#define CS  15      // Chip-Select
SPIClass spi = SPIClass(HSPI);  // HSPI bus for SD card


// defines for DAC
#define LSB_DAC (5000 / pow(2, 16)) // 5V / 2^16 = 76.3 uV
#define DACOFFSET 2049              // the offset of the DAC, this is the voltage that is applied when the DAC is set to 0 (in mV), because the opamp is offset by 2049mV

#define SPI1_SCK  18  // Clock
#define SPI1_MISO 19  // Master Input Slave Output not used for DAC
#define SPI1_MOSI 23  // Master Output Slave Input
#define SPI1_CS   5  // Chip-Select
DAC80501 dac;       // Create DAC80501 object



// defines for ADC
#define LSB_ADC 62.5E-6                // 2.048V / 2^15 = 62.5 uV
#define UPPER_LIMIT (pow(2, 15) - 600) // 2^15 -700 = 32768 - 700 = 32068 is the value the mux switches to the next resistance
#define LOWER_LIMIT 680                // is the value the mux switches to the next resistance
Adafruit_ADS1115 ads;                  // Create ADS1115 object



// global variables
const char *ssid = "ACCESPOINT"; // SSID of the Access Point

AsyncWebServer server(80); // Create AsyncWebServer object on port 80


struct ReceivedData
{ // Struct to hold received data
  int startPot;
  int endPot;
  int totalTime;
  float frequency;
  int amplitude;
};

struct ReceivedData parameters; // Create struct to hold received data

bool startMeasurementButtonState = false; // State of the start measurement button
bool sdCardPesent = true;               // check if the sd card is present

// Temperature sensor
const int oneWireBus = 32;           // GPIO16 where the data pin of DS18B20 is connected to
OneWire oneWire(oneWireBus);         // Setup a oneWire instance to communicate with any OneWire devices
DallasTemperature sensors(&oneWire); // Pass our oneWire reference to Dallas Temperature sensor
float temperatureC = 0;              // Variable to hold temp in Celsius

// Mux pins
const int enablePin = 17; // enable pin of the mux
const int A1Pin = 4;      // A1 pin of the mux
const int A0Pin = 16;     // A0 pin of the mux

// variables for current measurement
uint8_t currentState = 3; // current state of the mux
long resistance;            // resistance of the mux
int16_t results;           // variable to hold the ADC value
float offset = 0;          // offset of the opamp (not used in current version)
float currentHigh = 0;     // variable to hold the current value
float currentLow = 0;      // variable to hold the current value
int measurementIndex = 0;  // index for the data array

// variables for squarewave
hw_timer_t *Timer0_Cfg = NULL; // timer for the squarewave
int potDiff = 0;               // difference between the start and end pot
int stepSize = 0;              // stepsize of the pot
int rampPot = 0;               // current pot value
int nstep = 0;                 // number of steps
float period = 0;              // period of the squarewave
int dir = 1;                   // keeps track if there needs to be a positive or negative puls
int amplitude = 0;             // amplitude of the squarewave
int endPot = 0;                // endpot of the squarewave
bool isrTrigger = false;       // keeps track if the timer has triggered
bool measurmentDone = true;              // keeps track if the measurement is done

portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED; // mutex for the timer interrupt

struct IV
{ // struct to hold the measurment data
  int celPotential;
  float measuredCurrent;
  float temperature;
};

// function prototypes
void switchTMUX1109(uint8_t state);                         // function to switch the mux
void setDacValue(int voltage);                             // function to set the DAC value
float measure();                                            // function to measure the current
void IRAM_ATTR Timer0_ISR();                                // timer interrupt
void timerForInterrupt(uint64_t timeInMs);                  // function to set the timer interrupt
void squareWaveVoltammetry(struct ReceivedData parameters); // function to start the swv measurement

const int dataSize = 1000; // size of the data array
IV data[dataSize];         // array to hold the data

void setup(void)
{
  // defualt values for the parameters
  parameters.startPot = 50;
  parameters.endPot = 1500;
  parameters.totalTime = 20000;
  parameters.frequency = 5;
  parameters.amplitude = 0;

  Serial.begin(115200); // Start Serial Monitor

  pinMode(enablePin, OUTPUT); // set the mux pins as output
  pinMode(A1Pin, OUTPUT);
  pinMode(A0Pin, OUTPUT);


  Timer0_Cfg = timerBegin(0, 80, true);                // begin timer
  timerAttachInterrupt(Timer0_Cfg, &Timer0_ISR, true); // attach interrupt
  timerAlarmWrite(Timer0_Cfg, 1000000, true);          // set timer to 1 second
  timerAlarmEnable(Timer0_Cfg);                        // enable timer

  dac.begin(&SPI,SPI1_SCK,SPI1_MISO,SPI1_MOSI,SPI1_CS); // initialize DAC
  dac.setREG4_DivGain(SET4_DIV1,SET4_GAIN2);            // set DAC gain and divider
  // setDacValue(DACOFFSET + 100);                             // set DAC value to 100mV above the offset
  // delay(100);                                               // wait for the opamp to settle

  spi.begin(SCK, MISO, MOSI, CS); // initialize SPI for SD card

  if (!SD.begin(CS,spi,80000000)) { // initialize SD card
  #ifdef DEBUG
    Serial.println("Card Mount Failed");  // if SD card fails to initialize, stop program
  #endif
   sdCardPesent = false;
  }
  #ifdef DEBUG
    Serial.print("SD car present: "); // if SD card initializes, print message
    Serial.println(sdCardPesent);     // if SD card initializes, print message
  #endif


  switchTMUX1109(currentState); // set the mux to the default state

  if (!ads.begin()) // initialize ADC
  {
    Serial.println("Failed to initialize ADS.");
    while (1) // stop program if ADC fails to initialize
      ;
  }

  ads.setDataRate(RATE_ADS1115_64SPS);                                       // set ADC datarate
  ads.startADCReading(ADS1X15_REG_CONFIG_MUX_DIFF_0_1, /*continuous=*/true); // start ADC reading
  int test = ads.readADC_Differential_0_1();                                 // read ADC value (dummy read)


  WiFi.softAP(ssid);               // Start the Access Point
  IPAddress IP = WiFi.softAPIP();  // Get the IP address of the Access Point
  Serial.print("AP IP address: "); // Print the IP address
  Serial.println(IP);              // Print the IP address

  SPIFFS.begin(); // Start the SPI Flash File System (SPIFFS)

  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) { // Handle requests to root path (/)
    request->send(SPIFFS, "/index.html", "text/html");
  });

  server.on("/Chart.js", HTTP_GET, [](AsyncWebServerRequest *request) { // Handle requests to Chart.js path (/Chart.js)
    request->send(SPIFFS, "/Chart.js", "application/javascript");
  });

  server.on("/sendData", HTTP_GET, [](AsyncWebServerRequest *request) { // Handle requests to sendData path (/sendData)
    if (request->hasParam("startPot") && request->hasParam("endPot") && request->hasParam("totalTime") && request->hasParam("frequency") && request->hasParam("amplitude"))
    {                                                                       // Check if all parameters are present
      parameters.startPot = request->getParam("startPot")->value().toInt(); // Get parameters and convert to int
      parameters.endPot = request->getParam("endPot")->value().toInt();
      parameters.totalTime = request->getParam("totalTime")->value().toInt();
      parameters.frequency = request->getParam("frequency")->value().toInt();
      parameters.amplitude = request->getParam("amplitude")->value().toInt();
#ifdef DEBUG
      Serial.println("Received data: ");
      Serial.print(parameters.startPot);
      Serial.print(", ");
      Serial.print(parameters.endPot);
      Serial.print(", ");
      Serial.print(parameters.totalTime);
      Serial.print(", ");
      Serial.print(parameters.frequency);
      Serial.print(", ");
      Serial.println(parameters.amplitude);
#endif

      request->send(200, "text/plain", "Data received!"); // Send response to client
    }
    else
    {
      request->send(400); // Send error code if all parameters are not present
    }
  });

  server.on("/getData", HTTP_GET, [](AsyncWebServerRequest *request) { // Handle requests to getData path (/getData)
    String responseData;
    if (measurmentDone == false)
    { // check if the measurement is done
      for (int i = 1; i < dataSize; i++)
      { // loop through the data array and add the data to the response
        if (data[i].celPotential != 0 && data[i].measuredCurrent != 0)
        {                                                                                // check if the data is not empty
          responseData += String(data[i].celPotential) + "," + String(data[i].measuredCurrent) + "," + String(data[i].temperature) + ","; // add the data to the response
        }
      }
      request->send(200, "text/plain", responseData); // Send response to client
    }
    else
    {
      request->send(204, "text/plain", ""); // Send no content if the measurement is done
    }
  });


  server.on("/startMeasurement", HTTP_GET, [](AsyncWebServerRequest *request) { // Handle requests to startMeasurement path (/startMeasurement)
    #ifdef DEBUG
      Serial.println("Start Measurement button is pressed"); 
    #endif                     // Print to serial monitor
    
    startMeasurementButtonState = true;                                         // toggle button state
    request->send(200, "text/plain", String(startMeasurementButtonState));      // Send response to client
  });

  server.begin(); // Start server

  sensors.begin(); // Start temperature sensor

}

void loop(void)
{
  yield(); // Yield to allow ESPAsyncWebServer to handle requests
  if (startMeasurementButtonState == true)
  {                                      // check if the start measurement button is pressed
    squareWaveVoltammetry(parameters);   // start the measurement
    startMeasurementButtonState = false; // reset the button state
  }

  if (isrTrigger)
  { // check if the timer has triggered

    if (rampPot < endPot && measurmentDone == false)
    { // check if the endpot is reached

      if (dir == 1)
      {                                               // check if the direction is positive
        currentHigh = measure();                      // measure the current on the positive puls
        rampPot += stepSize;                          // increase the pot value
        setDacValue(rampPot + amplitude + DACOFFSET); // set the DAC value
#ifdef DEBUG
        Serial.println("");
        Serial.print(" currenthigh ");
        Serial.print(currentHigh);
        Serial.print(" voltagehigh ");
        Serial.println(rampPot + amplitude + DACOFFSET);
#endif
        dir = 0; // change the direction
      }
      else if (dir == 0)
      {                                               // check if the direction is negative
        currentLow = measure();                       // measure the current on the negative puls
        setDacValue(rampPot - amplitude + DACOFFSET); // set the DAC value
#ifdef DEBUG
        Serial.print(" currentLow ");
        Serial.print(currentLow);
        Serial.print(" voltageLow ");
        Serial.println(rampPot - amplitude + DACOFFSET);
#endif
        data[measurementIndex].celPotential = rampPot;                             // add the data to the array
        data[measurementIndex].measuredCurrent = (currentHigh + currentLow) * 0.5; // add the data to the array
        data[measurementIndex].temperature = temperatureC;                         // add the data to the array
#ifdef DEBUG
        Serial.print(" RampPot ");
        Serial.print(data[measurementIndex].celPotential);
        Serial.print(" current ");
        Serial.println(data[measurementIndex].measuredCurrent);
#endif
        measurementIndex++; // increase the index
        dir = 1;            // change the direction
      }
    }

    portENTER_CRITICAL_ISR(&timerMux); // enter critical section
    isrTrigger = false;                // reset the trigger
    portEXIT_CRITICAL_ISR(&timerMux);  // exit critical section
  }


  if (rampPot >= endPot && !measurmentDone)// Check if the end potential is reached
  { 
    if (SD.begin(CS,spi,80000000)) { // initialize SD card
  #ifdef DEBUG
    Serial.println("Card Mount succesful");  // if SD card fails to initialize, stop program
  #endif
   sdCardPesent = true;
  }else{
    sdCardPesent = false; 
  }
    
    if (sdCardPesent) // Check if the sd card is present
    { 
      // Check directory and create new file
      int measurmentFileIndex = 1;
      char filename[20] = "/measurment.csv";
      #ifdef DEBUG
        Serial.println("Saving data to sd card");
      #endif
      while (SD.exists(filename)) 
      {
        measurmentFileIndex++;
        sprintf(filename, "/measurment_%d.csv", measurmentFileIndex);
      }
      #ifdef DEBUG
        Serial.print("Filename: ");
        Serial.println(filename);
      #endif

      writeFile(SD, filename, "");  // create the file
      appendFile(SD, filename, "celPotential;measuredCurrent;temperature\n");  // add the header to the file

      // Save the data to the sd card
      char dataString[200];
      for (int i = 1; i < measurementIndex; i++)
      {

        sprintf(dataString, "%d;%f;%f\n", data[i].celPotential, data[i].measuredCurrent, data[i].temperature);
        //convert point to comma
        for (int i = 0; i < strlen(dataString); i++)
        {
          if (dataString[i] == '.')
          {
            dataString[i] = ',';
          }
        }
        appendFile(SD, filename, dataString);
      }
    }

    measurementIndex = 0;             // reset the index
    memset(data, '\0', sizeof(data)); // reset the array
    measurmentDone = true;            // set the done flag
    SD.end();                         // stop the sd card
  }
}

void IRAM_ATTR Timer0_ISR()
{                                    // timer interrupt
  portENTER_CRITICAL_ISR(&timerMux); // enter critical section
  isrTrigger = true;                 // set the trigger
  portEXIT_CRITICAL_ISR(&timerMux);  // exit critical section
}

void timerForInterrupt(uint64_t timeInMs)
{                                              // function to set the timer interrupt
  portENTER_CRITICAL(&timerMux);               // enter critical section
  timerAlarmWrite(Timer0_Cfg, timeInMs, true); // set the timer
  portEXIT_CRITICAL(&timerMux);                // exit critical section
}

void squareWaveVoltammetry(struct ReceivedData parameters)
{ // function to start the swv measurement
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

  sensors.requestTemperatures(); // Send the command to get temperatures
  temperatureC = sensors.getTempCByIndex(0);  // Read temperature in Celsius

  #ifdef DEBUG  
    Serial.print("Temperature: ");
    Serial.println(temperatureC);
  #endif

  potDiff = parameters.endPot - parameters.startPot;       // calculate the difference between the start and end pot
  period = (1 / parameters.frequency) * 1000;              // calculate the period of the squarewave
  nstep = parameters.totalTime / period;                   // calculate the number of steps
  stepSize = potDiff / nstep;                              // calculate the stepsize
  amplitude = parameters.amplitude;                        // set the amplitude
  endPot = parameters.endPot;                              // set the endpot
  rampPot = parameters.startPot;                           // set the startpot
  measurmentDone = false;                                  // reset the done flag
  setDacValue(rampPot + amplitude + stepSize + DACOFFSET); // set the DAC value
  int temp = ads.readADC_Differential_0_1();               //  dummy read

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
  timerForInterrupt((period / 2) * 1000); // set the timer interrupt
  return;
}

void setDacValue(int voltage)
{ // function to set the DAC value
  if (voltage < DACOFFSET)
  {                        // check if the DAC value is below the offset
    voltage == DACOFFSET; // set the DAC value to the offset
  }
  dac.writeDAC(voltage / LSB_DAC); // set the DAC value
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

void switchTMUX1109(uint8_t state)
{ // change mux state for higher or lower amplification
  switch (state)
  {
  case 1:                       // S1 50MOhm gian resistor (20nA/V)
    digitalWrite(enablePin, 1); // enable the mux
    digitalWrite(A1Pin, 0);     // set the mux pins
    digitalWrite(A0Pin, 0);     // set the mux pins
    resistance = 50E6;           // change resistance for current calculation

    offset = 0.03;                 // change offset for current calculation (not used in current version)

    break;
  case 2:                       // S2  1.2MOhm gian resistor (833nA/V)
    digitalWrite(enablePin, 1); // enable the mux
    digitalWrite(A1Pin, 0);     // set the mux pins
    digitalWrite(A0Pin, 1);     // set the mux pins
    resistance = 12E5;           // change resistance for current calculation
    offset = 0;                 // change offset for current calculation (not used in current version)
    break;
  case 3:                       // S3  27.7kOhm gian resistor (36,1uA/V)
    digitalWrite(enablePin, 1); // enable the mux
    digitalWrite(A1Pin, 1);     // set the mux pins
    digitalWrite(A0Pin, 0);     // set the mux pins
    resistance = 27700;          // change resistance for current calculation
    offset = 0;
    break;
  case 4:                       // S4    649Ohm gian resistor (1,54mA/V)
    digitalWrite(enablePin, 1); // enable the mux
    digitalWrite(A1Pin, 1);     // set the mux pins
    digitalWrite(A0Pin, 1);     // set the mux pins
    resistance = 649;            // change resistance for current calculation
    offset = 0;             // change offset for current calculation (not used in current version)
    break;
  default:
    digitalWrite(enablePin, 0); // disable the mux
    digitalWrite(A1Pin, 0);     // set the mux pins
    digitalWrite(A0Pin, 0);     // set the mux pins
    break;
  }
}

float measure() // function to measure the current
{ 
  do
  {
    results = ads.readADC_Differential_0_1() * -1; // read the ADC value
    if (results < LOWER_LIMIT && currentState != 1 && dir == 1)
    {                                                 // check if the ADC value is below the lower limit
      currentState--;                                // decrease the state
      switchTMUX1109(currentState);                  // switch the mux
      int temp = ads.readADC_Differential_0_1() * -1; // dummy read
      // delay(10);

#ifdef DEBUG
      Serial.print("results = ");
      Serial.print(results);
      Serial.print("  mux geschakeld naar weerstand: ");
      Serial.print(resistance);
      Serial.print("  state: ");
      Serial.println(currentState);
#endif
    }
    else if (results > UPPER_LIMIT && currentState != 4 && dir == 1)
    {                                                 // check if the ADC value is above the upper limit
      currentState++;                                // increase the state
      switchTMUX1109(currentState);                  // switch the mux
      int temp = ads.readADC_Differential_0_1() * -1; // dummy read
      // delay(10);

#ifdef DEBUG
      Serial.print("results = ");
      Serial.print(results);
      Serial.print("  mux geschakeld naar weerstand: ");
      Serial.print(resistance);
      Serial.print("  state: ");
      Serial.println(currentState);
#endif
    }
    if ((results < LOWER_LIMIT && currentState == 1) || (results > UPPER_LIMIT && currentState == 4) || dir != 1)
    {                                                // check if the ADC value is above the upper limit or below the lower limit or if the direction is negative
      results = ads.readADC_Differential_0_1() * -1; // dummy read
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
  } while (results < LOWER_LIMIT || results > UPPER_LIMIT); // loop until the ADC value is between the limits

  float adcVoltage = (results * LSB_ADC);          // in V
  float current = ((adcVoltage / resistance) * 1E9) + offset;    // in nA

#ifdef DEBUG
  Serial.print(results);
  Serial.print(",");
  Serial.print(adcVoltage);
  Serial.print("V ,");
  Serial.print(current);
  Serial.println("nA ,");
#endif

  return current; // return the current
}
