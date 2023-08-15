#include <PID_v1.h>
#include <SPI.h>
#include <Adafruit_MAX31855.h> //1ss 2nc 3miso 4sck 5gnd 6vcc !3v3!
#include <OneWire.h>
#include <DallasTemperature.h>

#define TEMP_MODULE 0 // 0 Dallas One-Wire sensor, 1 MAX31855 thermocouple module 

#define PIN_OUTPUT 3 //PID  PWM Output pin
#define ONE_WIRE_BUS 2 // pin for onewire bus for the temp sensor
#define STP_PIN 7  
#define SW_PIN 11
#define DIR_PIN 4 
#define PULSES_PER_MM 128//4865//128 pulses per mm
#define TRAVEL 38 //in mm
#define INIT_SPEED 800
#define ACCEL 1.004
#define resolution 9 //default resolution of the temp sensor

//Definitions
#define MAXCS 8 // Chip enable on pin 8



double pstate1;
float tstate;
//=====PID variables & Sensors======
double Setpoint, Input, Output;
float ptemp; // previous temperature for averaging the value of the sensor
double Kp=4, Ki=0, Kd=0; //works p=6 i=0.3 d=10 Parameters for PID
float tempC;
//========================
boolean dir=1;
double pmillis;// previous time required for the async routine
boolean pstate;
boolean k;
//boolean d=1;
float acc=INIT_SPEED;
boolean PIDset=0;
boolean stp1=1,stp2=0,stp3=0,stp4=0,process=0;;
boolean done=0;
boolean next=0;
int MOVE=16;

//Initialize the thermocouple using hardware SPI
Adafruit_MAX31855 thermocouple(MAXCS);
OneWire oneWire(ONE_WIRE_BUS); //onewire struct
DallasTemperature sensors(&oneWire); //temperature sensor struct
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);


void setup() {
  analogWrite(PIN_OUTPUT, 255);
  Serial.begin(9600);
    while (!Serial) delay(1);

  Serial.println ("Initializing MAX31855 module...");
  delay(500);
  if (!thermocouple.begin()) {
    Serial.println("ERROR.");
    while (1) delay(10);
  }
  
  Serial.println("MAX31855 Module Ready!");
  
  //=====PID and temp sensor=====
  sensors.begin(); //for the temperature sensor
  Setpoint = 60; //PID setpoint i.e set temperature
  myPID.SetMode(AUTOMATIC); //Set the PID
  myPID.SetSampleTime(100);
  //ptemp = sensors.getTempCByIndex(0); //initialize ptemp for the averaging or the PID will get confused
  ptemp = thermocouple.readCelsius(); // read the temperature from the thermocouple
  //=============================
  pinMode(DIR_PIN,OUTPUT);
  pinMode(STP_PIN,OUTPUT);
  pinMode(SW_PIN,INPUT);
  pinMode(10,1);
  digitalWrite(10,1);
  digitalWrite(DIR_PIN,dir);
  digitalWrite(SW_PIN,1);
  delay(10);
  pstate=digitalRead(SW_PIN);
  Serial.println(F("Homing..."));
  motion(38,1000,ACCEL,0); //"Manual" homing because the homing is not yet implemented.
  motion(-38,1000,ACCEL,0);//"Manual" homing because the homing is not yet implemented.
  Serial.println(F("Homing completed!"));
  Serial.println(F("Moving to the default position."));
  motion(24,200,ACCEL,0);
}

void loop() {
  double exctime=millis();
  if(isnan(tempC))
  {
    analogWrite(PIN_OUTPUT, 255);
  }
  else
  {
    if(millis()-pmillis>100)
    {
      pmillis=millis();
      tempC = thermocouple.readCelsius(); // read the temperature from the thermocouple
      ptemp=(tempC+ptemp)/2.0; // smoothing the temperature otherwise the D parameter will over react
      
        if ((Setpoint-10)<=ptemp && PIDset==0)
        {
          PIDset=1;
          myPID.SetTunings(10,0.2,0.4);
        }
        Input = ptemp; // this will pass the ptemp to the PID object  
        myPID.Compute(); // compute the PID
        analogWrite(PIN_OUTPUT, 255-Output); // 255-out because the output is inverted so -*-=+       
        Serial.print("Temperature: ");
        Serial.print(ptemp);  
        Serial.print("\t");
        Serial.print("Power ");
        Serial.println(Output); 
     // Serial.print("Execution time is: ");
     // Serial.println(millis()-exctime);
    }
  }
   
  if (process==0)
  {
    tstate=millis();      
  }
  
  if((digitalRead(SW_PIN)==1 && pstate==0) || process==1)
  {   
    process=1;  
    if(stp1==1 && millis()-tstate>=35000 && done==0 )
    {
      pstate1=millis();
      motion(-20,200,ACCEL,0);
      done=1;
      stp1=0;
      stp2=1;
    }
    else if(stp2==1 && millis()-pstate1>=60000 && done==1)
    {
      done=0;
      motion(34,200,ACCEL,0); 
      stp2=0;
      stp3=1;
      process=0;
    }
    else if(stp3==1 && millis()-tstate>=35000 && done==0)
    {
      pstate1=millis();
      done=1;
      motion(-34,200,ACCEL,0);
      stp3=0;
      stp4=1;
    }
    else if(stp4==1 && millis()-pstate1>=60000 && done==1 )
    {
      done=0;
      motion(20,200,ACCEL,0); 
      stp4=0;
      stp1=1; 
      process=0;
    }
  }

  pstate=digitalRead(SW_PIN);
  delay(10);
  
}
