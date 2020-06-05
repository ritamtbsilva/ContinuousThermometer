/*************************************************** 
IAS - Instituto Superior Técnico
MSc in Biomedical Engineering

Rita Silva 86805
Vicente Garção 86810

This code uses MPC9808 IC contact temperature sensor and 
NodeMCU ESP32 
 ****************************************************/

#include <Wire.h>
#include "Adafruit_MCP9808.h"

// Create the MCP9808 temperature sensor object
Adafruit_MCP9808 tempsensor = Adafruit_MCP9808();

  //variables for kalman 1st pass
  double Q=0.00001;
  double R=1.000;
  double xhat=36.000; 
  double P=1.000;
  double xhatminus=0.000;
  double Pminus=0.000;
  double K=0.000;
  double M=36.00;
  double vm[]={xhat,P,xhatminus,Pminus,K,M,Q,R};

  //variables for kalman 2nd pass
  double QCPost=0.00001;
  double RCPost=1.000;
  double xhatCPost=36.000; 
  double PCPost=1.000;
  double xhatminusCPost=0.000;
  double PminusCPost=0.000;
  double KCPost=0.000;
  double MCPost=34.00;
  double vmCPost[]={xhatCPost,PCPost,xhatminusCPost,PminusCPost,KCPost,MCPost,QCPost,RCPost};

  //variables for point measurement kalman
  double Q_Pont=0.2;
  double R_Pont=1.000;
  double xhat_Pont=36.500; 
  double P_Pont=1.000;
  double xhatminus_Pont=0.000;
  double Pminus_Pont=0.000;
  double K_Pont=0.000;
  double M_Pont=36.500;
  double vm_Pont[]={xhat_Pont,P_Pont,xhatminus_Pont,Pminus_Pont,K_Pont,M_Pont,Q_Pont,R_Pont};

  //variables for point measurement
  int j=0;
  double Res=0.00;
  int kk=0;

  boolean i=false;
  int pin_LED = 2;
  int pin_switch = 5;

  boolean x=false;
  int pin_LEDON = 19;
  int pin_switchON = 4;

// variables to hold the new and old switch states
  boolean oldSwitchState = HIGH;
  boolean newSwitchState1 = HIGH;
  boolean newSwitchState2 = HIGH;
  boolean newSwitchState3 = HIGH;

  boolean oldSwitchStateON = HIGH;
  boolean newSwitchState1ON = HIGH;
  boolean newSwitchState2ON = HIGH;
  boolean newSwitchState3ON = HIGH;
   
  boolean LEDstatus = LOW;
  boolean LEDstatusON = LOW;

  //variables for when the sensor outputs nan
  float d=0.0;
  int nanI=0;

  //variables for initial stabilization
  int counter=0;
  int counter1=0;
  int counter2=1;
  double rr=0.0;
  double counteraux=1.0;
  double result=36.00;
  
void setup() {
  Serial.begin(115200);

  pinMode(pin_LED, OUTPUT);  
  digitalWrite(pin_LED,LOW); 
  pinMode(pin_switch, INPUT_PULLUP); 
  pinMode(pin_LEDON, OUTPUT);  
  digitalWrite(pin_LEDON,LOW); 
  pinMode(pin_switchON, INPUT_PULLUP); 
  
  while (!Serial); //waits for serial terminal to be open, necessary in newer arduino boards.
  Serial.println("MCP9808 demo");
  
  // Make sure the sensor is found, you can also pass in a different i2c
  // address with tempsensor.begin(0x19) for example, also can be left in blank for default address use
  // Also there is a table with all addres possible for this sensor, you can connect multiple sensors
  // to the same i2c bus, just configure each sensor with a different address and define multiple objects for that
  //  A2 A1 A0 address
  //  0  0  0   0x18  this is the default address
  //  0  0  1   0x19
  //  0  1  0   0x1A
  //  0  1  1   0x1B
  //  1  0  0   0x1C
  //  1  0  1   0x1D
  //  1  1  0   0x1E
  //  1  1  1   0x1F
  if (!tempsensor.begin(0x18)) {
    Serial.println("Couldn't find MCP9808! Check your connections and verify the address is correct.");
    while (1);
  }

   Serial.println("Found MCP9808!");

  tempsensor.setResolution(3); // sets the resolution mode of reading, the modes are defined in the table bellow:
  // Mode Resolution SampleTime
  //  0    0.5°C       30 ms
  //  1    0.25°C      65 ms
  //  2    0.125°C     130 ms
  //  3    0.0625°C    250 ms
}

static void kalman(double v[]) {
  v[2]=v[0];
  v[3]=v[1]+v[6];
  v[4]=(v[3])/(v[3]+v[7]);
  v[0]=v[2]+v[4]*(v[5]-v[0]);
  v[1]=(1-v[4])*v[3];
}

void loop() {

  //code for on/off button, from http://www.martyncurrey.com/
  newSwitchState1ON = digitalRead(pin_switchON);
  delay(1);
  newSwitchState2ON = digitalRead(pin_switchON);
  delay(1);
  newSwitchState3ON = digitalRead(pin_switchON);
 
  // if all 3 values are the same we can continue
  if (  (newSwitchState1ON==newSwitchState2ON) && (newSwitchState1ON==newSwitchState3ON) ) {
    if ( newSwitchState1ON != oldSwitchStateON ) {
      // has the button switch been closed?
      if ( newSwitchState1ON == LOW ) {
        if ( LEDstatusON == LOW ) { digitalWrite(pin_LEDON, HIGH);  LEDstatusON = HIGH; x=true; tempsensor.wake();}
        else                      { digitalWrite(pin_LEDON, LOW);   LEDstatusON = LOW;  x=false; tempsensor.shutdown_wake(1);}
      }
    oldSwitchStateON = newSwitchState1ON;
    }
  }
  
  if(x==true) {
    
    //code for continuous/punctual button, from http://www.martyncurrey.com/
    newSwitchState1 = digitalRead(pin_switch);
    delay(1);
    newSwitchState2 = digitalRead(pin_switch);
    delay(1);
    newSwitchState3 = digitalRead(pin_switch);
     
      // if all 3 values are the same we can continue
    if (  (newSwitchState1==newSwitchState2) && (newSwitchState1==newSwitchState3) ) {
      if ( newSwitchState1 != oldSwitchState ) {
      // has the button switch been closed?
        if ( newSwitchState1 == LOW ) {
          if ( LEDstatus == LOW ) { digitalWrite(pin_LED, HIGH);  LEDstatus = HIGH; i=true; }
          else                    { digitalWrite(pin_LED, LOW);   LEDstatus = LOW; i=false; }
        }
        oldSwitchState = newSwitchState1;
      }
    }
  
  
    if(i==false) {
      
      float c = tempsensor.readTempC();

      //checking for very low or very high values or nan
      if(c>44.0 || isnan(c) || c<15.0){
        if(isnan(c)){
          Serial.println("nan");
        }
        c=d;
        nanI+=1;
        if(nanI==20) {
          Serial.println("ERROR: Please check the connection.");
          digitalWrite(pin_LEDON, LOW);
          LEDstatusON = LOW;
          x=false;
          nanI=0;
          }
       }
       d=c;

       //filtering
       
       //kalman 1st pass
       vm[5]=c;
       kalman(vm);
       
       //kalman 2nd pass
       vmCPost[5]=vm[0];
       kalman(vmCPost);

       //code for initial stabilization
       if(counter1<30) {
         counter1+=1;
       }
       else {
         if (counter2<=10){
           rr+=vmCPost[0]/10;
           counter2+=1;
         }
         else {
          counteraux=(120-counter)/120;
          result=counteraux*rr+(1-counteraux)*vm[0];
          if (counter<120){
              counter+=1;
          }

          //fever alert message
          if(result>37.5) {
              Serial.println("ALERT: POSSIBLE FEVER DETECTED");
          } 
          else {
             if(result>37.2) {
               Serial.println("ALERT: POSSIBLE PRE-FEBRILE STATE DETECTED");
             }
          } 
          //raw temp
          //Serial.print(c, 3);
          //Serial.print(","); 
          //result:
          Serial.println(result, 3);
          }
        }
    }
    
    //for point measurement
    else {
      //continue continuous measurement, without printing
      float c = tempsensor.readTempC();

      //checking for very low or very high values or nan
      if(c>42.0 || isnan(c) || c<15.0){
        if(isnan(c)){
          Serial.println("nan");
        }
        c=d;
        nanI+=1;
        if(nanI==20) {
          Serial.println("Ocorreu um erro. Verificar a conexão.");
          digitalWrite(pin_LEDON, LOW);
          LEDstatusON = LOW;
          x=false;
          nanI=0;
          }
       }
       d=c;
       
       vm[5]=c;
       kalman(vm);
       vmCPost[5]=vm[0];
       kalman(vmCPost);
      
       if(counter1<30) {
        counter1+=1;
       }
       else {
         if (counter2<=10){
           rr+=vm[0]/10;
           counter2+=1;
           }
           else {
            counteraux=(120-counter)/120;
             result=counteraux*rr+(1-counteraux)*vm[0];
           if (counter<120){
            counter+=1;
          }
        }
      }

      //point measurement
      vm_Pont[5]=c;
      kalman(vm_Pont);
      Serial.println("Measuring: ");
      Serial.println(vm_Pont[0],1);
      
      if (vm_Pont[0]<35.0 || vm_Pont[0]>41.5) {
        j=0;
        kk=0;
      }
      else {
        if (j<25) {
          //checking if stable
          if ((fabs(vm_Pont[0]-vm_Pont[2]))<0.002) {
            j+=1;
          }
        }
        else {
          if (kk<5) {
            Res+=(vm_Pont[0]/5);
            kk=kk+1;
          }
          else {
            Serial.print("Your Temperature is ");
            Serial.print(Res, 1);
            Serial.println("C");
            kk=0;
            j=0;
            Res=0;
            i=false;
            digitalWrite(pin_LED, LOW);
            LEDstatus = LOW;
          }
        }
      }
    }
  }
  delay(1000);
}
