//#define USE_USBCON
#include <ros.h>
#include <geometry_msgs/Vector3.h>
#include <Arduino.h>

#define Encoder1A  2  
#define Encoder1B  3
#define Encoder2A  19  //left encoder19
#define Encoder2B  18  //left encoder18

#define motorIn1 4
#define motorIn2 5
#define motorIn3 6
#define motorIn4 7
//OUT1,OUT2 leftmotor
//OUT3,OUT4 rightmotor
//#define LOOPTIME        100                     // PID loop time
#define LOOPTIME        50
//#define LOOPTIME        10

ros::NodeHandle nh;

bool set_; 

geometry_msgs::Vector3 vel_msg;
ros::Publisher p("feedback_wheel_angularVel", &vel_msg);

int pin1A = 0;
int pin1AOld = 0;
int pin1B = 0;
int pin1BOld = 0;
int pin2A = 0;
int pin2AOld = 0;
int pin2B = 0;
int pin2BOld = 0;

volatile long Encoder1pos = 0;
volatile long Encoder2pos = 0;
unsigned long lastMilli = 0;                    // loop timing 
long dT = 0;

//Max linear velocity from cmd_vel is 0.112

//double omega_left_target = -0.87;
//double omega_right_target = -0.87;

double omega_left_target = 0;
double omega_right_target = 0;

double omega_left_actual = 0;                              // speed (actual value)
double omega_right_actual = 0;                              // speed (actual value)

int PWM_val1 = 0;                                // (25% = 64; 50% = 127; 75% = 191; 100% = 255)
int PWM_val2 = 0;
int CPR = 48;                                   // encoder count per revolution
int gear_ratio =226;                            // motor gear ratio

double sum_error, d_error, pidTerm=0;

//float Kp = 1;
float Kp = 1;
float Ki = 0.01;
//float Ki = 0;
float Kd = 1;
//float Kd = 10;

void messageCb(const geometry_msgs::Vector3& msg)
{
  omega_left_target = msg.x;  
  omega_right_target = msg.y;
}

ros::Subscriber<geometry_msgs::Vector3> s("cmd_wheel_angularVel",messageCb);


void left_encoder() 
{   pin1A = digitalRead(2); pin1B = digitalRead(3);
    if (pin1A == 0 && pin1B == 0) 
    {   
      if (pin1AOld == 1 && pin1BOld == 0)   Encoder1pos ++;
      if (pin1AOld == 0 && pin1BOld == 1)   Encoder1pos --;
    }
    if (pin1A == 0 && pin1B == 1) 
    {
      if (pin1AOld == 0 && pin1BOld == 0)   Encoder1pos ++;
      if (pin1AOld == 1 && pin1BOld == 1)   Encoder1pos --;
    }
    if (pin1A == 1 && pin1B == 1) 
    {
      if (pin1AOld == 0 && pin1BOld == 1)   Encoder1pos ++;
      if (pin1AOld == 1 && pin1BOld == 0)   Encoder1pos --;
    }
    if (pin1A == 1 && pin1B == 0) 
    {
      if (pin1AOld == 1 && pin1BOld == 1)   Encoder1pos ++;
      if (pin1AOld == 0 && pin1BOld == 0)   Encoder1pos --;
    }
    pin1AOld = pin1A; pin1BOld = pin1B;
}

void right_encoder() 
{   pin2A = digitalRead(19); pin2B = digitalRead(18);
    if (pin2A == 0 && pin2B == 0) 
    {
      if (pin2AOld == 1 && pin2BOld == 0)   Encoder2pos --;
      if (pin2AOld == 0 && pin2BOld == 1)   Encoder2pos ++;
    }
    if (pin2A == 0 && pin2B == 1) 
    {
      if (pin2AOld == 0 && pin2BOld == 0)   Encoder2pos --;
      if (pin2AOld == 1 && pin2BOld == 1)   Encoder2pos ++;
    }
    if (pin2A == 1 && pin2B == 1) 
    {
      if (pin2AOld == 0 && pin2BOld == 1)   Encoder2pos --;
      if (pin2AOld == 1 && pin2BOld == 0)   Encoder2pos ++;
    }
    if (pin2A == 1 && pin2B == 0) 
    {
      if (pin2AOld == 1 && pin2BOld == 1)   Encoder2pos --;
      if (pin2AOld == 0 && pin2BOld == 0)   Encoder2pos ++;
    }
  pin2AOld = pin2A; pin2BOld = pin2B;
}


void getMotorData()  
{                               
  static long Encoder1posPre = 0;       
  static long Encoder2posPre = 0;

  omega_left_actual  = ((Encoder1pos - Encoder1posPre)*(1000/dT))*2*PI/(CPR*gear_ratio);  //ticks/s to rad/s
  omega_right_actual = ((Encoder2pos - Encoder2posPre)*(1000/dT))*2*PI/(CPR*gear_ratio);  //ticks/s to rad/s
  
  Encoder1posPre = Encoder1pos;         
  Encoder2posPre = Encoder2pos;         
}
 
double updatePid(double targetValue,double currentValue)   
{            
  //double pidTerm = 0;                                                            // PID correction
  //double error, last_error, sum_error, d_error;  
  double error=0;
  static double last_error=0;                            
  error = targetValue - currentValue; 
  sum_error = sum_error + error * dT;
  d_error = (error - last_error) / dT;
  pidTerm = Kp * error + Ki * sum_error + Kd * d_error;                           
  last_error = error;  
  //return constrain(int(pidTerm/0.00682352941), 0, 255);
  return constrain(int(pidTerm/0.00682352941), -255, 255);
}

void setup() 
{
  nh.initNode();
  nh.subscribe(s);
  nh.advertise(p);
  
  pinMode(motorIn1,OUTPUT);   pinMode(motorIn2,OUTPUT);   
  pinMode(motorIn3,OUTPUT);   pinMode(motorIn4,OUTPUT);
  pinMode(Encoder1A, INPUT);  pinMode(Encoder1B, INPUT);  
  pinMode(Encoder2A, INPUT);  pinMode(Encoder2B, INPUT);
  
  digitalWrite(Encoder1A, HIGH);  digitalWrite(Encoder1B, HIGH);  
  digitalWrite(Encoder2A, HIGH);  digitalWrite(Encoder2B, HIGH);       // turn on pullup resistor

  attachInterrupt(0, left_encoder, CHANGE); attachInterrupt(1, left_encoder, CHANGE); 
  attachInterrupt(4, right_encoder, CHANGE); attachInterrupt(5, right_encoder, CHANGE);
  
  //Serial.begin (115200);
  //Serial.println("start"); 
}

void loop() 
{
     //vel_msg.x=omega_left_actual;
     //vel_msg.y=omega_right_actual;
     //nh.spinOnce();
     //p.publish(&vel_msg);
     
     if((millis()-lastMilli) >= LOOPTIME)   
     {                                    // enter tmed loop
        dT = millis()-lastMilli;
        lastMilli = millis();
        getMotorData();                                                           // calculate speed

        PWM_val1= (updatePid(omega_left_target, omega_left_actual));                       // compute PWM value from rad/s
        PWM_val2= (updatePid(omega_right_target, omega_right_actual)); 

        if (PWM_val1<=0)   { analogWrite(motorIn1,abs(PWM_val1));  analogWrite(motorIn2,0);}
        if (PWM_val1>0)    { analogWrite(motorIn1,0);  analogWrite(motorIn2,abs(PWM_val1));}
        if (PWM_val2<=0)  { analogWrite(motorIn3,abs(PWM_val2));  analogWrite(motorIn4,0);}
        if (PWM_val2>0)   { analogWrite(motorIn3,0);  analogWrite(motorIn4,abs(PWM_val2));}
        
        vel_msg.x=omega_left_actual;
        vel_msg.y=omega_right_actual;
        p.publish(&vel_msg);
        nh.spinOnce();
        //printMotorInfo();
     }

}

