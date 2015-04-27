//FOR 2 ADC
#include <ADC.h>
#include "Teensy31FastADC.h"

//FOR TEMPERATURE/HUMIDITY SENSOR
#include <dht11.h>
#define DHT11PIN 12
static dht11 DHT11;

// Teensy 3.1 has the LED on pin 13
#define LEDPIN 13

void setup() {

  pinMode(LEDPIN, OUTPUT);
  pinMode(A2, INPUT); 
  pinMode(A3, INPUT); 
  pinMode(A10, INPUT); 
  pinMode(A11, INPUT);
  highSpeed8bitADCSetup();

  Serial.begin(115200);
  //BLINK LED, WE ARE ALIVE
  digitalWrite(LEDPIN,1);
  delay(2000);
  digitalWrite(LEDPIN,0);
}


#define SAMPLES 2048
#define BUFFERSIZE 2048
#define NO_EVENT -1

const int channelA2 = ADC::channel2sc1aADC0[2];
const int channelA3 = ADC::channel2sc1aADC1[3];
const int channelA11 = ADC::channel2sc1aADC0[11];
const int channelA10 = ADC::channel2sc1aADC1[10];

byte THRESHOLD = 180;
byte value1 = 0;
byte value2 = 0;
byte value3 = 0;
byte value4 = 0;

byte buffer1[BUFFERSIZE] = {0};
byte buffer2[BUFFERSIZE] = {0};
byte buffer3[BUFFERSIZE] = {0};
byte buffer4[BUFFERSIZE] = {0};

int samples = 0;
long startTime = 0;
long stopTime = 0;
long totalTime = 0;
int event = NO_EVENT;

int i = 0;
int k = 0;


void loop() {
  startTime = micros();
     //START SAMPLING
     //Strange init in this for, but the compiler seems to optimize this code better, so we get faster sampling
  i = 0;
  k = 0;
  samples = SAMPLES;
  event = NO_EVENT;
  for(i<samples;i++) {
    //TAKE THE READINGS
    highSpeed8bitAnalogReadMacro(channelA2,channelA3,value1,value2);
    //SHOULD ADJUST THIS 2nd READING
    highSpeed8bitAnalogReadMacro(channelA11, channelA10,value3,value4);
    
    buffer1[k] = value1;
    buffer2[k] = value2;
    buffer3[k] = value3;
    buffer4[k] = value4;
    
    //CHECK FOR EVENTS
    if (value1 > THRESHOLD && event != NO_EVENT) {
      event = k;
      //THERE IS AN EVENT, ARE WE REACHING THE END? IF SO TAKE MORE SAMPLES
      if (i > SAMPLES-1024) samples = SAMPLES+1024;
      //SHOULD AJUST TIME LOST IN THIS LOGIC TOO
    }
    
    if (++k == BUFFERSIZE) k = 0; 
  }
  stopTime = micros();
  
  //WAS AN EVENT BEEN DETECTED?
  if (event != NO_EVENT) {
    printInfo();
    printSamples(); 
  }
 
  //DID WE RECEIVE COMMANDS?
  if (Serial.available()) parseSerial();

}


void parseSerial() {
  char c = Serial.read();

  switch (c) {
  case 'p': 
    printInfo();
    break;
  case 's': 
    printSamples();
    break;
  case '+': 
    THRESHOLD += 5;
    break;             
  case '-': 
    THRESHOLD -= 5;
    break;             
  default:  
    break;
  }
}


void printSamples() {
  
  Serial.print("BUFFSIZE: ");
  Serial.print(BUFFERSIZE,DEC);
  Serial.print(" Event: ");
  Serial.println(event);
  serialWrite(buffer1,BUFFERSIZE);
  serialWrite(buffer2,BUFFERSIZE);
  serialWrite(buffer3,BUFFERSIZE);
  serialWrite(buffer4,BUFFERSIZE);
  Serial.flush();
  
}


//This should be optimized. Writing raw binary data seems to fail a lot of times
//and I ended up loosing bytes. Maybe some form of flow-control should be used.
void serialWrite(byte *buffer,int siz) {
  int kk;
  for (kk=0;kk<siz;kk++) {
    Serial.print(buffer[kk],HEX);    
    Serial.print(" ");
  }
  Serial.println();
}

void printInfo() {
  totalTime = stopTime-startTime;
  double samplesPerSec = i*1000.0/totalTime;
  
  //Take a temperature/humidity reading
  //The DHT11 should be connected with a resistor for less errors in readings,
  // but works without it if you take some readings untils you got an ok one.
  while(DHT11.read(DHT11PIN) != DHTLIB_OK);

  Serial.print("T: ");
  Serial.print(totalTime);
  Serial.print(" Samples: ");
  Serial.print(i,DEC);
  Serial.print(" Samples/uSec: ");
  Serial.print(samplesPerSec,7);
  Serial.print(" Temp: ");
  Serial.print((float)DHT11.temperature,2);
  Serial.print(" Hum: ");
  Serial.print((float)DHT11.humidity,2);
  Serial.print(" Threshold: ");
  Serial.println(THRESHOLD,DEC);
  Serial.flush();
}



