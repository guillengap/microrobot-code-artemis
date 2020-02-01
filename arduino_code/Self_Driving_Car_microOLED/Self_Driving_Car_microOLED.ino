// AUTHOR: GUILLERMO PEREZ GUILLEN
// SELF DRIVING CAR USING REDBOARD ARTEMIS ATP
// HERE I HAVE ADDED CHANGES TO USE THE MICRO OLED BREAKOUT
#include <Wire.h>  // Include Wire if you're using I2C
#include <SFE_MicroOLED.h>  // Include the SFE_MicroOLED library
#include "bitmaps.h"

#define ENA 3
#define ENB 5
#define IN1 9
#define IN2 8
#define IN3 11
#define IN4 12

//The library assumes a reset pin is necessary. The Qwiic OLED has RST hard-wired, so pick an arbitrarty IO pin that is not being used
#define PIN_RESET 9  
//The DC_JUMPER is the I2C Address Select jumper. Set to 1 if the jumper is open (Default), or set to 0 if it's closed.
#define DC_JUMPER 1

MicroOLED oled(PIN_RESET, DC_JUMPER);    // I2C declaration

/******************************************************************
   NETWORK CONFIGURATION
******************************************************************/
const int InputNodes = 4; // includes BIAS neuron
const int HiddenNodes = 4; //includes BIAS neuron
const int OutputNodes = 4;
int i, j;
double Accum;
double Hidden[HiddenNodes];
double Output[OutputNodes];
float HiddenWeights[4][4] = {{0.17866143746327376, 1.2476813456677593, 1.3090655495221495, 1.2445062312559725}, {-1.7132414471678064, 1.5596067696189984, 1.5406219743087697, 1.5730922805016239}, {-2.43289276480003, -1.204242114438389, -2.944643109916896, 0.9516471102541175}, {-2.0347785002603174, -1.558481720598503, -1.634567146499255, -1.5534420493981382}};
float OutputWeights[4][4] = {{-2.7727880100233158, 0.0037884509782587852, -2.0728190956836148, 0.03398843127997504}, {0.5583295718411021, -2.5080202424896245, -2.3237787704971336, 2.3569365260109474}, {1.4514053378567477, 0.16314133834084582, 2.4036605861654485, -2.2387636044149746}, {-1.4487173474748358, 2.340813414462775, 0.30276781122413404, -0.09562873030426469}};

int error=0;
int dif,difAnt=0;
const float Kp=0.1;
const float Kd=0.1;

void setup() {
  delay(100);
  Wire.begin();
  oled.begin();    // Initialize the OLED
  oled.clear(ALL); // Clear the display's internal memory
  oled.display();  // Display what's in the buffer (splashscreen)
  delay(100);     // Delay 100 ms
  oled.clear(PAGE); // Clear the buffer.
    
	Serial.begin(9600);
	pinMode(A0, INPUT); //left sensor
	pinMode(A1, INPUT); //center sensor
	pinMode(A3, INPUT); //right sensor 
	pinMode(IN1, OUTPUT);
	pinMode(IN2, OUTPUT);
	pinMode(IN3, OUTPUT);
	pinMode(IN4, OUTPUT);
	pinMode(ENA, OUTPUT);
	pinMode(ENB, OUTPUT); 
} 

void loop()
{
double TestInput[] = {0, 0, 0};
double input1=0,input2=0,input3=0;
    
float volts0 =  analogRead(A0)*0.00322265625;  // value from sensor * (3.3/1024) 
float volts1 =  analogRead(A1)*0.00322265625;  // value from sensor * (3.3/1024) 
float volts2 =  analogRead(A3)*0.00322265625;  // value from sensor * (3.3/1024)

dif = analogRead(A3) - analogRead(A0);	// PID CONTROL
error = floor(Kp*(dif)+Kd*(difAnt-dif));	// PID CONTROL
difAnt=dif;	// CONTROL PID
int d0 = constrain(255 - error, 0, 255);//left speed - PID CONTROL
int d1 = constrain(255 + error, 0, 255);//right speed - PID CONTROL

float sensor_left =  8*pow(volts0, -1); // worked out from datasheet graph //GP2Y0A51SK0F - 2 a 15 cm
float sensor_center = 21*pow(volts1, -1); // worked out from datasheet graph //GP2Y0A41SK0F - 4 a 30 cm
float sensor_right = 9*pow(volts2, -1); // worked out from datasheet graph //GP2Y0A51SK0F - 2 a 15 cm

if (sensor_left<13){input1=1;}
else {input1=0;}
	
if (sensor_center<16.7){input2=1;}
else {input2=0;}

if (sensor_right<13){input3=1;}
else {input3=0;}

/******************************************************************
    WE CALL THE FEEDFORWARD NETWORK WITH THE INPUTS
******************************************************************/
 
  Serial.print("Input1:");
  Serial.println(input1);
  Serial.print("Input2:");
  Serial.println(input2);
  Serial.print("Input3:");
  Serial.println(input3);  
  
//THESE ARE THE THREE INPUTS WITH VALUES OF 0 TO 1 ********************
  TestInput[0] = 1.0;//BIAS UNIT
  TestInput[1] = input1;
  TestInput[2] = input2;
  TestInput[3] = input3;  

// THIS FUNCTION IS TO GET THE OUTPUTS **********************************
  InputToOutput(TestInput[0], TestInput[1], TestInput[2], TestInput[3]); //INPUT to ANN to obtain OUTPUT

  int out1 = round(abs(Output[0]));
  int out2 = round(abs(Output[1]));
  int out3 = round(abs(Output[2]));
  int out4 = round(abs(Output[3]));

  if (out1==0 && out2==0 && out3==0 && out4==0){
    drawStop();
  }

  if (out1==1 && out2==0 && out3==1 && out4==0){
    drawForward();
  }

  if (out1==0 && out2==1 && out3==0 && out4==1){
    drawBack();
  }

  if (out1==0 && out2==1 && out3==1 && out4==0){
    drawLeft();
  }
  
  if (out1==1 && out2==0 && out3==0 && out4==1){
    drawRight();
  }
  
  Serial.print("Output1:");
  Serial.println(out1);
  Serial.print("Output2:");
  Serial.println(out2);
  Serial.println(Output[1]);
  Serial.print("Output3:");
  Serial.println(out3);
  Serial.print("Output4:");
  Serial.println(out4);

/******************************************************************
    DRIVE MOTORS WITH THE NETWORK OUTPUT
******************************************************************/
  analogWrite(ENA, d0);
  analogWrite(ENB, d1);
  digitalWrite(IN1, out1 * HIGH); 
  digitalWrite(IN2, out2 * HIGH); 
  digitalWrite(IN3, out3 * HIGH);
  digitalWrite(IN4, out4 * HIGH);
  delay(50);
}

void InputToOutput(double In1, double In2, double In3, double In4)
{
  double TestInput[] = {0, 0, 0, 0};
  TestInput[0] = In1;
  TestInput[1] = In2;
  TestInput[2] = In3;
  TestInput[3] = In4;  

/******************************************************************
    CALCULATE ACTIVITIES IN HIDDEN LAYERS
******************************************************************/

  for ( i = 0 ; i < HiddenNodes ; i++ ) {	// We go through the four columns of the hidden weights
    Accum = 0;
    for ( j = 0 ; j < InputNodes ; j++ ) {	// Three values of the entry line and each column of hidden weights
      Accum += TestInput[j] * HiddenWeights[j][i] ;
    }
    Hidden[i] = tanh(Accum) ; // We obtain a matrix of a line with four values
  }

/******************************************************************
    CALCULATE ACTIVATION AND ERROR IN THE OUTPUT LAYER
******************************************************************/

  for ( i = 0 ; i < OutputNodes ; i++ ) {
    Accum = 0;
    for ( j = 0 ; j < HiddenNodes ; j++ ) {
        Accum += Hidden[j] * OutputWeights[j][i] ;
    }
    Output[i] = tanh(Accum) ;//tanh
  }
}

/******************************************************************
   END NETWORK CONFIGURATION
******************************************************************/

void stop() {	// We deactivate the engines
	digitalWrite(ENA, LOW); 
	digitalWrite(ENB, LOW); 
	Serial.println("Stop!");
} 

//---------------------------------------------------------------
void drawLeft()
{
    oled.clear(ALL);
    oled.clear(PAGE);
    oled.drawBitmap(left);//Display Logo
    oled.display();
}
//---------------------------------------------------------------

void drawRight()
{
    oled.clear(ALL);
    oled.clear(PAGE);
    oled.drawBitmap(right);//Display Logo
    oled.display();
}
//---------------------------------------------------------------

void drawForward()
{
    oled.clear(ALL);
    oled.clear(PAGE);
    oled.drawBitmap(forward);//Display Logo
    oled.display();
}//---------------------------------------------------------------

void drawBack()
{
    oled.clear(ALL);
    oled.clear(PAGE);
    oled.drawBitmap(back);//Display Logo
    oled.display();
}//---------------------------------------------------------------

void drawStop()
{
    oled.clear(ALL);
    oled.clear(PAGE);
    oled.drawBitmap(stop2);//Display Logo
    oled.display();
}
