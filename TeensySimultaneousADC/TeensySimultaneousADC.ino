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

const int channelA2 = ADC_Module::channel2sc1aADC0[2];
const int channelA3 = ADC_Module::channel2sc1aADC1[3];
const int channelA11 = ADC_Module::channel2sc1aADC0[11];
const int channelA10 = ADC_Module::channel2sc1aADC1[10];

byte THRESHOLD = 180;
byte value1;
byte value2;
byte value3;
byte value4;

byte buffer1[BUFFERSIZE];
byte buffer2[BUFFERSIZE];
byte buffer3[BUFFERSIZE];
byte buffer4[BUFFERSIZE];

int samples;
long startTime;
long stopTime;
long totalTime;
int event;

int i;
int k;


void loop() {
  startTime = micros();
     //START SAMPLING
     //Strange init in this for, but the compiler seems to optimize this code better, so we get faster sampling
  for(i=0,k=0,samples=SAMPLES,event=0;i<samples;i++) {
    //TAKE THE READINGS
    highSpeed8bitAnalogReadMacro(channelA2,channelA3,value1,value2);
    //SHOULD ADJUST THIS 2nd READING
    highSpeed8bitAnalogReadMacro(channelA11, channelA10,value3,value4);
    
    buffer1[k] = value1;
    buffer2[k] = value2;
    buffer3[k] = value3;
    buffer4[k] = value4;
    
    //CHECK FOR EVENTS
    if (value1 > THRESHOLD && !event) {
      event = k;
      //THERE IS AN EVENT, ARE WE REACHING THE END? IF SO TAKE MORE SAMPLES
      if (i > SAMPLES-1024) samples = SAMPLES+1024;
      //SHOULD AJUST TIME LOST IN THIS LOGIC TOO
    }
    
    if (++k == BUFFERSIZE) k = 0; 
  }
  stopTime = micros();
  
  //WAS AN EVENT BEEN DETECTED?
  if (event != 0) {
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



