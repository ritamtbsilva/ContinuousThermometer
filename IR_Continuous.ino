/*************************************************** 
IAS - Instituto Superior Técnico
MSc in Biomedical Engineering

Rita Silva 86805
Vicente Garção 86810

This code uses MLX90614 IR non-contact temperature sensor and 
NodeMCU ESP32 
 ****************************************************/

#include <Wire.h>
#include <Adafruit_MLX90614.h>

Adafruit_MLX90614 mlx = Adafruit_MLX90614();

  //variables for kalman 1st pass
  double Q=0.004;
  double R=1.000;
  double xhat=36.500; 
  double P=2.000;
  double xhatminus=0.000;
  double Pminus=0.000;
  double K=0.000;
  double M=36.500;
  double vm[]={xhat,P,xhatminus,Pminus,K,M,Q,R};

  //variables for ambient kalman
  double xhatA=36.000; 
  double PA=1.000;
  double xhatminusA=0.000;
  double PminusA=0.000;
  double KA=0.000;
  double MA=36.000;
  double vmA[]={xhatA,PA,xhatminusA,PminusA,KA,MA,Q,R};

  //variables for IIR
  double y=36.000;
  double alpha=0.000;
  double a=0.00004;
  double tamb=1.000;
  double vmf[]={tamb,alpha,y,a};

  //variables for kalman 2nd pass
  double QPOST=0.0001;
  double RPOST=1.000;
  double xhatPOST=36.000; 
  double PPOST=1.000;
  double xhatminusPOST=0.000;
  double PminusPOST=0.000;
  double KPOST=0.000;
  double MPOST=36.000;
  double vmPOST[]={xhatPOST,PPOST,xhatminusPOST,PminusPOST,KPOST,MPOST,QPOST,RPOST};

  //variables for initial stabilization
  int counter1=0;
  int counter2=0;
  int counter3=0;
  double rr=0.0;
  double d=30.0;
  double b=20.0;

void setup() {
  Serial.begin(115200);
  mlx.begin();
  Serial.println("IR Sensor Test");  
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

      double c=mlx.readObjectTempC();
      double a=mlx.readObjectTempC();

      //checking for very low or very high values
      if(c>42.0 || c<30.0) {
        c=d;
      }
      d=c;
      if(a>40.0 || a<20.0) {
        a=b;
      }
      b=a;

      //code for initial stabilization
      if(counter1<10) {
          counter1+=1;
      }
      else {
        if (counter2<10){
          rr+=c/10;
          counter2+=1;
        }
        else {
          if(counter3<1) {
            vmf[2]=rr;
            vm[0]=rr;
            vm[5]=rr;
            vmPOST[0]=rr;
            vmPOST[5]=rr;
            counter3+=1;
          }
          else {
            
            //ambient temperature Kalman
            vmA[5]=a;
            kalman(vmA);
      
            //object temperature Kalman 1st pass
            vm[5]=c;
            kalman(vm);
      
            //object temperature IIR filter
            filtertemp(vmf,vm,vmA);
      
            //object temperature Kalman 2nd pass
            vmPOST[5]=vmf[2];
            kalman(vmPOST);
            
            //fever alert message
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
            //result:
            Serial.println(vmPOST[0], 3);
            
          }
        }
  delay(1000);
}
