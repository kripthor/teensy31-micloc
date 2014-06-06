/* Example for analogRead
*  You can change the number of averages, bits of resolution and also the comparison value or range.
*/


#include <ADC.h>

#define ADC_0 0
#define ADC_1 1


// Teensy 3.0 has the LED on pin 13
const int ledPin = 13;


void highSpeed8bitADCSetup(){
  
  /*
      0 ADLPC (Low-Power Configuration)
      0 ADIV (Clock Divide Select)
      0
      0 ADLSMP (Sample time configuration)
      0 MODE (Conversion mode selection) (00=8/9, 01=12/13, 10=10/11, 11=16/16 bit; diff=0/1)
      0
      0 ADICLK (Input Clock Select)
      0
  */
  ADC0_CFG1 = 0b00000000;
  ADC1_CFG1 = 0b00000000;
   /*
      0 MUXSEL (ADC Mux Select)
      0 ADACKEN (Asynchrononous Clock Output Enable)
      0 ADHSC (High-Speed Configuration)
      0 ADLSTS (Long Sample Time Select) (00=+20 cycles, 01=+12, 10=+6, 11=+2)
      0
  */
  ADC0_CFG2 = 0b10100;
  ADC1_CFG2 = 0b10100;

  
  /*
      0 ADTRG (Conversion Trigger Select)
      0 ACFE (Compare Function Enable)
      0 ACFGT (Compare Function Greater than Enable)
      0 ACREN (Compare Function Range Enable)
      0 ACREN (COmpare Function Range Enable)
      0 DMAEN (DMA Enable)
      0 REFSEL (Voltage Reference Selection) (00=default,01=alternate,10=reserved,11=reserved)
  */
  ADC0_SC2 = 0b0000000;
  ADC1_SC2 = 0b0000000;
 
  /*
      1 CAL (Calibration)
      0 CALF (read only)
      0 (Reserved)
      0
      0 ADCO (Continuous Conversion Enable)
      1 AVGS (Hardware Average Enable)
      1 AVGS (Hardware Average Select) (00=4 times, 01=8, 10=16, 11=32)
      1
  */
  
  ADC0_SC3 = 0b1000000;
  ADC1_SC3 = 0b1000000;

  
  // Waiting for calibration to finish. The documentation is confused as to what flag to be waiting for (SC3[CAL] on page 663 and SC1n[COCO] on page 687+688).
  while (ADC0_SC3 & ADC_SC3_CAL) {} ;
  while (ADC1_SC3 & ADC_SC3_CAL) {} ;

}


void setup() {



  pinMode(ledPin, OUTPUT);
  pinMode(A2, INPUT); //pin 23 single ended
  pinMode(A3, INPUT); //pin 23 single ended
  pinMode(A10, INPUT); //pin 23 single ended
  pinMode(A11, INPUT); //pin 23 single ended
  Serial.begin(115200);

  highSpeed8bitADCSetup();
  delay(500);
  Serial.println("end setup");
}



int value1 = ADC_ERROR_VALUE;
int value2 = ADC_ERROR_VALUE;
int value3 = ADC_ERROR_VALUE;
int value4 = ADC_ERROR_VALUE;


int startTime;
int stopTime;
int totalTime;
int i;
int samples = 100000;
int samplesPerSec;

ADC::Sync_result result;

#define CHANNEL_A0 5
#define CHANNEL_A1 14
#define CHANNEL_A2 8
#define CHANNEL_A3 9
#define CHANNEL_A4 13
#define CHANNEL_A5 12
#define CHANNEL_A6 6
#define CHANNEL_A7 7
#define CHANNEL_A8 15
#define CHANNEL_A9 4
#define CHANNEL_A10 0
#define CHANNEL_A11 19
#define CHANNEL_A12 3
#define CHANNEL_A13 147

#define highSpeed8bitAnalogReadMacro(channel1, channel2) ADC0_SC1A = channel1;ADC1_SC1A = channel2;while (1) {if ((ADC0_SC1A & ADC1_SC1A & ADC_SC1_COCO)) {break;}}value1 = ADC0_RA;value2 = ADC1_RA;

int highSpeed8bitAnalogRead(uint8_t channel1, uint8_t channel2){
        ADC0_SC1A = channel1;
        ADC1_SC1A = channel2;
        while (1) {
	  if ((ADC0_SC1A & ADC1_SC1A & ADC_SC1_COCO)) {
	    break;
	  }
        }
	value1 = ADC0_RA;
	value2 = ADC1_RA;
}

void loop() {

 // GPIOC_PTOR = 1<<5;

  startTime = micros();
  
  //__disable_irq();
  for(i=0;i<samples;i++) {
      
     highSpeed8bitAnalogReadMacro(ADC_Module::channel2sc1aADC0[11], ADC_Module::channel2sc1aADC1[10]);
     value3 = value1;
     value4 = value2;
     highSpeed8bitAnalogReadMacro(ADC_Module::channel2sc1aADC0[2], ADC_Module::channel2sc1aADC1[3]);
    
     
  }
  //__enable_irq();
  
  stopTime = micros();
  totalTime = stopTime-startTime;
  samplesPerSec = i*1000/totalTime;
  Serial.print("T: ");
  Serial.print(totalTime);
  Serial.print(" S/uSec: ");
  Serial.print(samplesPerSec);
  Serial.print(" V1: ");
  Serial.print(value1);
  Serial.print(" V2: ");
  Serial.print(value2);
  Serial.print(" V3: ");
  Serial.print(value3);
  Serial.print(" V4: ");
  Serial.print(value4);
  Serial.println();
  digitalWrite(ledPin, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(100);               // wait for a second
  digitalWrite(ledPin, LOW);    // turn the LED off by making the voltage LOW
  delay(100);  // wait for a second
 

}


