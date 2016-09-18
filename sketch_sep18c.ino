#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <math.h>

/* This driver reads raw data from the BNO055

   Connections
   ===========
   Connect SCL to analog 5
   Connect SDA to analog 4
   Connect VDD to 3.3V DC
   Connect GROUND to common ground

   History
   =======
   2015/MAR/03  - First release (KTOWN)
*/

/* Set the delay between fresh samples */
#define BNO055_SAMPLERATE_DELAY_MS (10)

Adafruit_BNO055 bno = Adafruit_BNO055();

/**************************************************************************/
/*
    Arduino setup function (automatically called at startup)
*/
/**************************************************************************/
void setup(void)
{
  pinMode(2,OUTPUT);
  pinMode(5,OUTPUT);
  Serial.begin(9600);
  Serial.println("Orientation Sensor Raw Data Test"); Serial.println("");

  /* Initialise the sensor */
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }

  delay(1000);

  /* Display the current temperature */
  int8_t temp = bno.getTemp();
  Serial.print("Current Temperature: ");
  Serial.print(temp);
  Serial.println(" C");
  Serial.println("");

  bno.setExtCrystalUse(true);

  Serial.println("Calibration status values: 0=uncalibrated, 3=fully calibrated");
}


/////THRESHOLDS///////////
double thresh_PK=4600;
double R1_thresh=6000;
double R2_thresh=0.1;
double thresh_A=1600;
double vel_thresh=5;
double ZERO=1010;
//////////////////////////

int   state=1;
double buf1=0;
double buf2=0;
double dif=0;
double count=0;
double base=0;
double peak=0;
double R1;
double R2;
double vel=0;
double vel_cnt=0;
double acc_nog;
double A=0;


float t1,t2, delta;

void loop(void)

{
  t1 = micros();
  // Possible vector values can be:
  // - VECTOR_ACCELEROMETER - m/s^2
  // - VECTOR_MAGNETOMETER  - uT
  // - VECTOR_GYROSCOPE     - rad/s
  // - VECTOR_EULER         - degrees
  // - VECTOR_LINEARACCEL   - m/s^2
  // - VECTOR_GRAVITY       - m/s^2



  imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  //imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  imu::Vector<3> accel_nog = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
double acc;
double SV;
acc_nog=sqrt(accel_nog.x()*accel_nog.x()+accel_nog.y()*accel_nog.y()+accel_nog.z()*accel_nog.z());
acc=sqrt(accel.x()*accel.x()+accel.y()*accel.y()+accel.z()*accel.z()); //acceleratia in m/s^2
SV=(1000*acc)/9.81;                                                      //acceleratia in mg
//Serial.println(SV);
switch (state) {
  
   case 1:
       buf1=buf2;
       buf2=SV;
       dif=buf2-buf1;
       if(dif>=0) count++;
       else count=0;
       if(SV>thresh_PK)
       {
        state=2;
        Serial.println(2);
       }
     break;
     
   case 2:
     count++;
     buf1=buf2;
     buf2=SV;
     dif=buf2-buf1;
     if(dif<0) { 
      peak=SV;
      state=3;
      Serial.println(3);
     }
     
     break;

   case 3:
   count++;
   if(SV<ZERO){ 
     
      base=count*BNO055_SAMPLERATE_DELAY_MS/1000; //Base length in ms
      Serial.print("base");
      Serial.println(base);
      R1=peak/base;
      if(R1>=R1_thresh){
      state=4;
      Serial.println(4);
      }
      else {
        state=1;
        reset();
      }
     }
     break;

     case 4:
     if(vel_cnt++<500) vel = acc_nog;    //accel
     else 
  { 
     vel=vel*BNO055_SAMPLERATE_DELAY_MS/1000;  // velocity in m/s in m/s
     vel=0.1;
     state=5;
      Serial.print("vel");
     Serial.println(vel);
      Serial.print("R1");
     Serial.println(R1);     R2=1000*vel/R1;  
     Serial.print("R2 ");
     Serial.println(100*R2);
     if(R2<R2_thresh & vel<vel_thresh) 
     {
      Serial.println(5);
     state=5;
     delay(1500);
     count=0;
     }
     else state=1;
  reset();
 }
     break;
     
    case 5:
    if(SV>1900)
  break;
   count++;
   if(count==1000) {
    
    state=6;
   }
    else state=1;
    reset();
     break;


  case 6:
 Serial.print("fall occured");
 digitalWrite(2, HIGH);
 tone(5,1000,2000);
 digitalWrite(2,LOW);
 delay(1000);
 state=1;
 reset();
  break;
   default: 

   break;
}

t2=micros();
delta=t2-t1;
//Serial.println(delta);
delayMicroseconds(10000-delta);

}


void reset () 
{

buf1=0;
buf2=0;
count=0;
base=0;
peak=0;
vel=0;
vel_cnt=0;
A=0;
Serial.println();
Serial.println();
Serial.println();
Serial.println();
  
}


void calibration () 
{

  uint8_t sys, gyro, accel, mag = 0;
  bno.getCalibration(&sys, &gyro, &accel, &mag);
  Serial.print(F("Calibration: "));
  Serial.print(sys, DEC);
  Serial.print(F(" "));
  Serial.print(gyro, DEC);
  Serial.print(F(" "));
  Serial.print(accel, DEC);
  Serial.print(F(" "));
  Serial.println(mag, DEC);
  
  
}

