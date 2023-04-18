/*
 * THIS IS CODE TO MAKE A MICROCAVITY 
 */
#include <PID_v1.h>
#include <OneWire.h>
#include <DallasTemperature.h>

#define FRONT_DBR_LAYERS 17   // How many pairs it will deposit for the first DBR
#define CAVITY_LAYERS 2  //How many layers of the next material it will deposit (usually its 2)
#define RETRACTION_TIME 36000//35000 //In ms
#define HEATING_TIME  15000//90000 //in ms 
//#define BACK_DBR_PAIRS  //How many layers it will deposit for the second DBR

#define PIN_OUTPUT 3 //PID  PWM Output pin
#define ONE_WIRE_BUS 2 // pin for onewire bus for the temp sensor
#define stp 13
#define inp 11
#define pin_dir 12
//#define maxpulses 1700//4865
#define pulses_per_mm 128//4865//128 pulses per mm
#define travel 38 //in mm
//#define spd 100
#define init_speed 800
#define a 1.004
#define resolution 9 //default resolution of the temp sensor
#define step1 300
#define step2
#define step3
#define step4n
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
float acc=init_speed;
boolean PIDset=0;
boolean stp1=1,stp2=0,stp3=0,stp4=0,process=0;;
boolean done=0;
boolean next=0;
int MOVE=16;
boolean interlayer=1;
//int i;
OneWire oneWire(ONE_WIRE_BUS); //onewire struct
DallasTemperature sensors(&oneWire); //temperature sensor struct
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);


void setup() {
  Serial.begin(9600);
  //=====PID and temp sensor=====
  sensors.begin(); //for the temperature sensor
  Setpoint = 80; //PID setpoint i.e set temperature
  myPID.SetMode(AUTOMATIC); //Set the PID
  //myPID.SetSampleTime(250);
  ptemp = sensors.getTempCByIndex(0); //initialize ptemp for the averaging or the PID will get confused
  //=============================
  pinMode(pin_dir,1);
  pinMode(stp,OUTPUT);
  pinMode(inp,0);
  pinMode(10,1);
  digitalWrite(10,1);
  digitalWrite(pin_dir,dir);
  digitalWrite(inp,1);
  delay(10);
  pstate=digitalRead(inp);
  motion(24,200,a,0);
}

void loop() {
  double exctime=millis();
  if (millis()-pmillis>=(750/ (1 << (12-resolution))))
    {
      pmillis=millis();
      // get temperature
      tempC=sensors.getTempCByIndex(0);
      ptemp=(tempC+ptemp)/2.0; // smoothing the temperature otherwise the D parameter will over react
      //Serial.print("Temp= ");
      //Serial.println(ptemp); 
        
      // Request temperature conversion - non-blocking / async
      sensors.setWaitForConversion(false);  // makes it async
      sensors.requestTemperatures();
      sensors.setWaitForConversion(true);
    }
    if ((Setpoint-10)<=ptemp && PIDset==0)
    {
      PIDset=1;
      myPID.SetTunings(10,0.1,5);
    }
    Input = ptemp; // this will pass the ptemp to the PID object  
    myPID.Compute(); // compute the PID
    analogWrite(PIN_OUTPUT, 255-Output); // 255-out because the output is inverted so -*-=+     
    /*  
    Serial.print("Temperature: ");
    Serial.print(ptemp);  
    Serial.print("\t");
    Serial.print("Power ");
    Serial.println(Output); 
 // Serial.print("Execution time is: ");
 // Serial.println(millis()-exctime); */
  
   
  if (process==0)
  {
    tstate=millis();      
  }
  
  if((digitalRead(inp)==1 && pstate==0) || process==1)
  {   
    process=1;
    int static button_state;
    if ((button_state<FRONT_DBR_LAYERS) || interlayer==0)
    {  
      
      if(stp1==1 && millis()-tstate>=RETRACTION_TIME && done==0 ) // moving to the heater for the firsts solutions layer
      {
        pstate1=millis();
        motion(-20,200,a,0);
        done=1;
        stp1=0;
        stp2=1;
      }
      else if(stp2==1 && millis()-pstate1>=HEATING_TIME && done==1) // moving back to the second solution
      {
        done=0;
        motion(34,200,a,0); 
        stp2=0;
        stp3=1;
        process=0;
        button_state++;
        Serial.print("mphke");
        Serial.println(button_state);
      }
      else if(stp3==1 && millis()-tstate>=RETRACTION_TIME && done==0) // moving to the heater for the second solutions layer
      {
        pstate1=millis();
        done=1;
        motion(-34,200,a,0);
        stp3=0;
        stp4=1;
      }
      else if(stp4==1 && millis()-pstate1>=HEATING_TIME && done==1 ) // moving back to the first solution 
      {
        done=0;
        motion(20,200,a,0); 
        stp4=0;
        stp1=1; 
        process=0;
        button_state++;
        Serial.print("mphke");
        Serial.println(button_state);
      }
    }
    else if(FRONT_DBR_LAYERS%2==0) // if we are on the first layer do the second as many times as required
    {
      interlayer=1;
      int static cavdep_t;
      if(cavdep_t<CAVITY_LAYERS-1)
     {
      if(millis()-tstate>=HEATING_TIME && done==0 ) // moving to the heater for the first solutions layer
      {
        pstate1=millis();
        motion(-20,200,a,0);
        done=1;
      }
      else if(millis()-pstate1>=RETRACTION_TIME && done==1) // moving back to the second solution
      {
        done=0;
        motion(20,200,a,0); 
        process=0;
        cavdep_t++;
        Serial.println("got in");
      } 
     }
      else interlayer=0;
    }
    else // if we are on the second layer do the first as many times as required
    {
      int static cavdep_t;
      if(cavdep_t<CAVITY_LAYERS-1)
      {
        if(millis()-tstate>=RETRACTION_TIME && done==0) // moving to the heater for the second solutions layer
        {
          pstate1=millis();
          done=1;
          motion(-34,200,a,0);
        }
        else if(millis()-pstate1>=HEATING_TIME && done==1 ) // moving back to the first solution 
        {
          done=0;
          motion(34,200,a,0); 
          process=0;
          cavdep_t++;
        }  
      }
      else interlayer=0;
    }     
  }

  pstate=digitalRead(inp);
  delay(10);
  
}
