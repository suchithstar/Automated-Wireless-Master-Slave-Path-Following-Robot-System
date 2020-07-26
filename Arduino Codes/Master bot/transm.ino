const int d0 = 22;
const int d1 = 23;
const int d2 = 24;
const int d3 = 25;
int j=0,o,t,u,y,r,zl=0,zr=0;
int a[4];
float  yaw;
boolean ll=LOW , rr=LOW , flag=LOW;
char *data;
int pid_vall,pid_valr;
int kp=200, ki=0 , kd=-5;
float prop,integral,previntegral,diff,preverr;

#include <VirtualWire.h>
#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps20.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

#include <AFMotor.h>
AF_DCMotor motor2(4);
AF_DCMotor motor1(2);

MPU6050 mpu;

#define OUTPUT_READABLE_YAWPITCHROLL


#define INTERRUPT_PIN 18 // use pin 2 on Arduino Uno & most boards
 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount,p;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer


float previ,i=0,set;
// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };



// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}



// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {
  for (j=22; j<=25; j++)
        {
          pinMode(j,INPUT);
        }



    // i I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    // initialize serial communication
    // (115200 chosen because it is required for Teapot Demo output, but it's
    // really up to you depending on your project)
    Serial.begin(115200);
    while (!Serial); // wait for Leonardo enumeration, others continue immediately

    // NOTE: 8MHz or slower host processors, like the Teensy @ 3.3V or Arduino
    // Pro Mini running at 3.3V, cannot handle this baud rate reliably due to
    // the baud timing being too misaligned with processor ticks. You must use
    // 38400 or slower in these cases, or use some kind of external separate
    // crystal solution for the UART timer.

    // initialize device
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);
    pinMode(36,INPUT);
    digitalWrite(36,LOW);
    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // wait for ready
    Serial.println(F("\nSend any character to begin DMP programming and demo: "));
    // empty buffer again
   
    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();
   

    // supply your own gyro offsets here, scaled for min sensitivity
      mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
        Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
        Serial.println(F(")..."));
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }
  
    // configure LED for output
    pinMode(27, OUTPUT);
    digitalWrite(27,LOW);
 //   attachInterrupt(digitalPinToInterrupt(19),sender,FALLING);
    flag = LOW;
    o=0;
    t=0;
    u=0;
    y=0;
    r=0;
    vw_set_tx_pin(26);
    vw_setup(370);
    
}



// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void  loop() {
    if (!dmpReady) return;
    while (!mpuInterrupt && fifoCount < packetSize) 
    {
        if (mpuInterrupt && fifoCount < packetSize) 
        {
          fifoCount = mpu.getFIFOCount();
        }   
    }
    
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();
    fifoCount = mpu.getFIFOCount();

    if ((mpuIntStatus & _BV(MPU6050_INTERRUPT_FIFO_OFLOW_BIT)) || fifoCount >= 1024) 
    {
        mpu.resetFIFO();
        fifoCount = mpu.getFIFOCount();
        Serial.println(F("FIFO overflow!"));
    }
    
    else if (mpuIntStatus & _BV(MPU6050_INTERRUPT_DMP_INT_BIT)) 
    {
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        fifoCount -= packetSize;
        
        #ifdef OUTPUT_READABLE_YAWPITCHROLL
            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            Serial.print("ypr\t");
            Serial.print(ypr[0] * 180/M_PI);
            Serial.print("\t");
            Serial.print(ypr[1] * 180/M_PI);
            Serial.print("\t");
            Serial.print(ypr[2] * 180/M_PI);
            if(ypr[0]<0)
            {
            yaw = (ypr[0]*180/M_PI) + 360 ;
            Serial.print("\t");
            Serial.println(yaw);  
            }
            else if(ypr[0]>0)
            {
             yaw =   ypr[0]*180/M_PI ;
              Serial.print("\t");
            Serial.println(yaw);
            }
            i++;
            for (j=22; j<=25 ;j++)//read from pin 4, 5, 6 & 7
            {
             a[j-22] = digitalRead(j);
            }
            if(i==2000)
            {
             set = yaw;  
            }
            if(i>2000)
            {
             if(a[0] == 0 && a[1] == 1 && a[2] == 0 && a[3] == 0)
             {
                  Serial.print(ll);
                  Serial.print("   ");
                  Serial.println(rr);
                  if(ll==HIGH)
                  { 
                    
                    if(set<90)
                    {
                     set = set + 360;
                    }
                     set = set - 90;
                  }
                  ll=LOW;
                  if(rr==HIGH)
                  {
                    if(set>270)
                    {
                     set = set - 360 ;
                    }
                    set = set + 90;
                  }
                  rr= LOW;
                   zl=0;
                   zr=0;              
                  forward(yaw , set);

                  if(o<2)
                  {
                   data="0";
                   vw_send((uint8_t *)data, strlen(data)); 
                   o++; 
                   t=0;
                   y=0;
                   u=0;
                   r=0;
                  
                  }
                  
              }
              
              if(a[0] == 0 && a[1] == 0 && a[2] == 1 && a[3] == 0)
              {
                ll=HIGH;
                rr=LOW;
               
                leftward(yaw , set);
                  if(t<2)
                  {
                   
                   data="4";
                   vw_send((uint8_t *)data, strlen(data)); 
                   t++;                     
                   o=0;
                   y=0;
                   u=0;
                   r=0;
                   zl=0;
                   zr=0;
                  }
                  
              }
              if(a[0] == 0 && a[1] == 1 && a[2] == 1 && a[3] == 0)
              {
                  rr = HIGH;
                  ll= LOW;
                  
                  rightward(yaw , set);
                  if(u<2)
                  {
                  data="6";
                  vw_send((uint8_t *)data, strlen(data)); 
                  u++; 
                  o=0;
                  y=0;
                  t=0;
                  r=0;
                  zl=0;
                  zr=0;
                  }
                  
              }
              if(a[0] == 0 && a[1] == 0 && a[2] == 0 && a[3] == 1)
              {
                  
                  backward(yaw,set);
                  if(y<2)
                  { 
                  data="8";
                  vw_send((uint8_t *)data, strlen(data)); 
                  y++; 
                  o=0;
                  t=0;
                  u=0;
                  r=0; 
                  zl=0;
                  zr=0;               
                  }   
              }
               if(a[0] == 1 && a[1] == 0 && a[2] == 1 && a[3] == 0)
              {
                ll=LOW;
                rr=LOW;
               stopb();
               if(r<2)
               {
                data="5";
                vw_send((uint8_t *)data, strlen(data));   
                u=0;
                y=0;   
                t=0;
                o=0;
                zl=0;
                zr=0;
                r++;
               }
              }
             i = 20000;
            }
         
            #endif
    }
 
}

void forward(float x,float y)
{  
 Serial.print("set = ");
 Serial.print(y);
 Serial.print("Actual set = ");
 Serial.println(set);
 float l ;
 l = x-y;
 error(l); 

}

void pid(float x)
{
 prop = 1;
 integral = integral + previntegral  ; 
 diff = x;

 pid_vall = (kp*prop) + (ki*integral) + (kd*diff);
 pid_valr = (kp*prop) + (ki*integral) - (kd*diff);
 previntegral = integral;
 Serial.print("pid_vall  ");
 Serial.println(pid_vall);
 Serial.print("pid_valr  ");
 Serial.println(pid_valr);
}

void error(float k)
{
float p;
Serial.print("ferror = ");
Serial.println(k);
if(-6 < k && k<-0.5 )
{
  p=k;
  pid(p);
  correct();
}

else if( k>0.5 && k<6)
{
    p=k;
  pid(p);
  correct();
   
}
 
else if(-0.5<k && k<0.5)
{
  p=k;
  pid(p);
 straight();  
}
}


void correct()
{

motor1.setSpeed(pid_vall);
motor2.setSpeed(pid_valr);
motor1.run(FORWARD);
motor2.run(FORWARD);

}

void straight()
{
motor1.setSpeed(pid_vall);
motor2.setSpeed(pid_valr);  
motor1.run(FORWARD);
motor2.run(FORWARD);
}

void leftward(float x, float y )
{
 float a,b;
 a=x-y;
 b = y-90;
 if(y<90 && x>y)
 {
  a = a-360;
  b = b+360;
 }
 Serial.println(a);
 if ( a < -88 && a > -93 )
 {
  forward( x , b);
 }
else if(a < 90 && a >-88 )
 {

motor1.setSpeed(190);
motor2.setSpeed(190);
motor1.run(BACKWARD);
motor2.run(FORWARD);
 }
 
}

void rightward(float x, float y )
{
 float a,b;
 a = x-y;
 b = y+90;
 if(y>270 && x<y)
 {
  a = a+360;
  b = b-360;
 }
 Serial.println(a);
 if (a>88 && a< 93 )
 {
 forward(x , b);
 }
 else if(a>-90 && a<88)
 {
 motor1.setSpeed(200);
motor2.setSpeed(200);
motor1.run(FORWARD);
motor2.run(BACKWARD);
 }
}

void backward(float x , float y)
{
float a = x-y ;
  errorb(a); 
}

void errorb(float k)
{
float p;
Serial.print
("berror = ");
Serial.println(k);
if(k>-6 && k<-0.5)
{
  p=k*(-1);
  pid(p);
  correctb();
}
else if( k>0.5 && k<6 )
{
  p=k*(-1);
  pid(p);
  correctb();
}
else if(-0.5<k && k<0.5)
{
 back();  
}
}

void correctb()
{
motor1.setSpeed(pid_vall);
motor2.setSpeed(pid_valr);
motor1.run(BACKWARD);
motor2.run(BACKWARD);

}

void back()
{
motor1.setSpeed(pid_vall);
motor2.setSpeed(pid_valr);  
motor1.run(BACKWARD);
motor2.run(BACKWARD);
}

void stopb()
{
motor1.run(RELEASE);
motor2.run(RELEASE);  
}




