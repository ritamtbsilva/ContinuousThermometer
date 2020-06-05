/*************************************************** 
IAS - Instituto Superior Técnico
MSc in Biomedical Engineering

Rita Silva 86805
Vicente Garção 86810

This code uses MPC9808 IC contact temperature sensor, MLX90614 
IR non-contact temperature sensor and NodeMCU ESP32 
 ****************************************************/

#include <Wire.h>
#include <Adafruit_MLX90614.h>

#include <Wire.h>
#include "Adafruit_MCP9808.h"

Adafruit_MCP9808 tempsensor = Adafruit_MCP9808();
Adafruit_MLX90614 mlx = Adafruit_MLX90614();

  //variables for kalman 1st pass IR
  double Q=0.004;
  double R=1.000;
  double xhat=36.500; 
  double P=2.000;
  double xhatminus=0.000;
  double Pminus=0.000;
  double K=0.000;
  double M=36.500;
  double vm[]={xhat,P,xhatminus,Pminus,K,M,Q,R};

  //variables for ambient kalman IR
  double xhatA=36.000; 
  double PA=1.000;
  double xhatminusA=0.000;
  double PminusA=0.000;
  double KA=0.000;
  double MA=36.000;
  double vmA[]={xhatA,PA,xhatminusA,PminusA,KA,MA,Q,R};

  //variables for IIR filter IR
  double y=36.000;
  double alpha=0.000;
  double a=0.00004;
  double tamb=1.000;
  double vmf[]={tamb,alpha,y,a};

  //variables for kalman 2st pass IR
  double QPOST=0.0001;
  double RPOST=1.000;
  double xhatPOST=36.000; 
  double PPOST=1.000;
  double xhatminusPOST=0.000;
  double PminusPOST=0.000;
  double KPOST=0.000;
  double MPOST=36.000;
  double vmPOST[]={xhatPOST,PPOST,xhatminusPOST,PminusPOST,KPOST,MPOST,QPOST,RPOST};

  //variables for kalman 1st pass IC
  double QC=0.00001;
  double RC=1.000;
  double xhatC=36.000; 
  double PC=1.000;
  double xhatminusC=0.000;
  double PminusC=0.000;
  double KC=0.000;
  double MC=36.00;
  double vmC[]={xhatC,PC,xhatminusC,PminusC,KC,MC,QC,RC};

  //variables for kalman 2st pass IC
  double QCPost=0.00001;
  double RCPost=1.000;
  double xhatCPost=36.000; 
  double PCPost=1.000;
  double xhatminusCPost=0.000;
  double PminusCPost=0.000;
  double KCPost=0.000;
  double MCPost=36.00;
  double vmCPost[]={xhatCPost,PCPost,xhatminusCPost,PminusCPost,KCPost,MCPost,QCPost,RCPost};

  boolean x=false;
  int pin_LEDON = 19;
  int pin_switchON = 4;

  // variables to hold the new and old switch states
  boolean oldSwitchStateON = HIGH;
  boolean newSwitchState1ON = HIGH;
  boolean newSwitchState2ON = HIGH;
  boolean newSwitchState3ON = HIGH;

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
  int counter1I=0;
  int counter2I=0;
  int counter3I=0;
  double rrI=0.0;

  
void setup() {
  Serial.begin(115200);
  mlx.begin();  
  
  pinMode(pin_LEDON, OUTPUT);  
  digitalWrite(pin_LEDON,LOW); 
  pinMode(pin_switchON, INPUT_PULLUP); 
  
  while (!Serial); //waits for serial terminal to be open, necessary in newer arduino boards.
  Serial.println("Both Sensors Test");
  
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

static void filtertemp(double vmIIR[], double v[], double vA[]) {
  double deriv=fabs(vA[0]-vmIIR[0]);
  vmIIR[1]=(2/(1+exp(-vmIIR[3]/deriv)))-1;
  double aux=vmIIR[2];
  vmIIR[2]=vmIIR[1]*v[0]+(1-vmIIR[1])*aux;
  vmIIR[0]=vA[0];
}


void loop() {

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
    //IC
    float c = tempsensor.readTempC();

    //checking for very low or very high values or nan
    if(c<15.0 || c>44.0 || isnan(c)){
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

    //filtering
       
    //kalman 1st pass
    vmC[5]=c;
    kalman(vmC);

    //kalman 2nd pass
    vmCPost[5]=vmC[0];
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
        Serial.print(result, 3);
        Serial.print(",");       
      }
    }
    
    //IR

    //initial stabilization
      if(counter1I<10) {
          counter1I+=1;
      }
      else {
        if (counter2I<10){
          rrI+=mlx.readObjectTempC()/10;
          counter2I+=1;
        }
        else {
          if(counter3I<1) {
            vmf[2]=rrI;
            vm[0]=rrI;
            vm[5]=rrI;
            vmPOST[0]=rrI;
            vmPOST[5]=rrI;
            counter3I+=1;
          }
          else {
            
            //ambient temperature Kalman
            vmA[5]=mlx.readAmbientTempC();
            kalman(vmA);
      
            //object temperature Kalman 1st pass
            vm[5]=mlx.readObjectTempC();
            kalman(vm);
      
            //object temperature IIR filter
            filtertemp(vmf,vm,vmA);
      
            //object temperature Kalman 2nd pass
            vmPOST[5]=vmf[2];
            kalman(vmPOST);
            if(vmPOST[0]>37.5) {
              Serial.println("ALERT: POSSIBLE FEVER DETECTED");
            }
            else {
             if(vmPOST[0]>37.2) {
               Serial.println("ALERT: POSSIBLE PRE-FEBRILE STATE DETECTED");
               }
            }
          }
            //raw temp
            //Serial.print(mlx.readObjectTempC(), 3);
            //Serial.print(",");
            //result
            Serial.println(vmPOST[0], 3);
            }
        }
      }
  delay(1000);
}
