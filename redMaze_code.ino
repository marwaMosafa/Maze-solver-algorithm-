/*
Author : Marwa mostafa 
edited in : 24/3/2018
*/

#include <PID_v1.h>
#include <LiquidCrystal.h> 

int Contrast=10 ;
//replace with your connection pins 
LiquidCrystal lcd(0, 1, 8, 9, 10, 11);   
                                                                                                             
//MOTOR A 
int enA = 3;
int in1 = 12;
int in2 = 13;

// Motor B

int enB = 5;
int in3 = 2;
int in4 = 7;

//sensor pins 
#define trig_pin A0 //analog input 1
#define echo_pin A1 //analog input 2

#define trig_pin2 A2 //analog input 1
#define echo_pin2 A3 //analog input 2

#define trig_pin3 A4 //analog input 1
#define echo_pin3 A5 //analog input 2

long duration, distance, distanceFront, distanceRight,distanceLeft,r1,r2,x ;
long LastDistanceFront = 0 , LastDistanceRight =0 ,LastDistanceLeft =0; 
 
  double Setpoint, Input1, Output=0.0;
  
  // first make KI and KD=0 and try Kp values that matches your robot 
  double Kp=2.0, Ki=0.2, Kd=0.5; 
  PID myPID(&Input1, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
  
void setup() {
  analogWrite(6,Contrast); 
  lcd.begin(16, 2);

  pinMode(in3, OUTPUT);  //make initially all pins as output
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in4, OUTPUT);

  pinMode(trig_pin, OUTPUT);
  pinMode(echo_pin, INPUT);
  pinMode(trig_pin2, OUTPUT);
  pinMode(echo_pin2, INPUT);
  pinMode(trig_pin3, OUTPUT);
  pinMode(echo_pin3, INPUT);
 
 // Serial.begin (9600);
   
     Setpoint = 8;
   
    distanceRight=SonarSensor(trig_pin2, echo_pin2);
    distanceFront=SonarSensor(trig_pin, echo_pin);
    distanceLeft=SonarSensor(trig_pin3, echo_pin3);
   
   Input1 = distanceRight;
    myPID.SetMode(AUTOMATIC);
    myPID.SetOutputLimits(-255,255);
    myPID.SetTunings(Kp, Ki, Kd);
    
    lcd.clear();
    lcd.setCursor(1 ,0);
    lcd.write("Mohsen");  //make your robot name :)
    delay(10);
    
    moveForward(100,Output);
    
   }
    
  void loop(){
    distanceRight=SonarSensor(trig_pin2, echo_pin2);
    distanceFront=SonarSensor(trig_pin, echo_pin);
    distanceLeft=SonarSensor(trig_pin3, echo_pin3);

    if(distanceRight < 30 && distanceFront >= 25 ){
    //  Serial.println("move forward");
      distanceRight=SonarSensor(trig_pin2, echo_pin2);
      distanceFront=SonarSensor(trig_pin, echo_pin);
      distanceLeft=SonarSensor(trig_pin3, echo_pin3);
      
      Input1 = distanceRight;
      myPID.Compute();
      moveForward(110,Output);
      
       //forward callibration
       if(distanceLeft>=20 && distanceRight >2){
            distanceRight = LastDistanceRight;
        }else{
            LastDistanceRight = distanceRight;
        }
        if(distanceRight>=20 && distanceLeft>2){
            distanceLeft = LastDistanceLeft;
          }else{
            LastDistanceLeft = distanceLeft;
               }
               
       
   
    }               //moving left 
    else if(distanceFront <= 27 && distanceRight <= 30 && distanceLeft >=30){
           r1=distanceLeft;
           r2=r1+12;
           x =r1/r2;
      do{
          turnLeft(x);
         // Serial.println(" moving left ");
          distanceRight=SonarSensor(trig_pin2, echo_pin2);
          distanceFront=SonarSensor(trig_pin, echo_pin);
          distanceLeft=SonarSensor(trig_pin3, echo_pin3);
            
        //callibration   
         if(distanceFront >= 70 || distanceRight >= 40 ){
               distanceFront = LastDistanceFront; 
               distanceRight = LastDistanceRight;
                
          }else {
                LastDistanceFront = distanceFront ;
                LastDistanceRight = distanceRight;
                }
          
      }while(distanceFront < 45 && distanceRight > 13&&distanceFront!=0);
            
  }else if(distanceRight > 30 && distanceFront <= 27 && distanceLeft<30){      
                r1=distanceRight;
                r2=r1+12;
                x =r1/r2;
              do{
                 distanceRight=SonarSensor(trig_pin2, echo_pin2);
                 distanceFront=SonarSensor(trig_pin, echo_pin);
                 distanceLeft=SonarSensor(trig_pin3, echo_pin3);
                 turnRight(x);
                 //Serial.println(" moving right");

                 //callibration
                if(distanceFront >= 70 || distanceRight >= 40  ){
                distanceFront = LastDistanceFront; 
                distanceRight = LastDistanceRight;
             
                }else {
                LastDistanceFront = distanceFront ;
                LastDistanceRight = distanceRight;
                 }
                }while(distanceFront<45 && distanceRight > 13 && !(distanceLeft > 13));
    }
    
    else if(distanceLeft>=30 &&distanceRight>=30 && distanceFront<=27){
                r1=distanceLeft;
                r2=r1+12;
                x =r1/r2;
         do{
            turnLeft(x);
            distanceRight=SonarSensor(trig_pin2, echo_pin2);
            distanceFront=SonarSensor(trig_pin, echo_pin);
            distanceLeft=SonarSensor(trig_pin3, echo_pin3);
            
           // Serial.println(" center turning left ");
             //callibration
         if(distanceFront >= 70 || distanceRight >= 40 ){
               distanceFront = LastDistanceFront; 
               distanceRight = LastDistanceRight;
          }else {
                LastDistanceFront = distanceFront ;
                LastDistanceRight = distanceRight;
                }        
      }while(distanceFront < 45 && distanceRight > 13 && !(distanceLeft > 13)&&distanceFront!=0);
    
    }else{
          distanceRight=SonarSensor(trig_pin2, echo_pin2);
          distanceFront=SonarSensor(trig_pin, echo_pin);
          distanceLeft=SonarSensor(trig_pin3, echo_pin3);
         // Serial.println("movestop ELSE");
          moveStop();
          moveForward(100,Output);
     
         }  
      
  }// end void loop

void printSerial(long front,long right,long left){
       Serial.print(front);
       Serial.print(" - ");
       Serial.print(right);
       Serial.print(" - ");
       Serial.println(left);
  }
long SonarSensor(int trigPin,int echoPin)
{
digitalWrite(trigPin, LOW);
delayMicroseconds(2);
digitalWrite(trigPin, HIGH);
delayMicroseconds(10);
digitalWrite(trigPin, LOW);
duration = pulseIn(echoPin, HIGH);
distance = (duration/2) / 29.1;
 return distance ;
}
void moveStop(){  //stop motors 
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);  
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
}

void moveForward(int i,double x){
    digitalWrite(in1, HIGH);
    digitalWrite(in3, HIGH);
    analogWrite(enA, (i-20)-x); //the speed left
    
    digitalWrite(in2, LOW);
    digitalWrite(in4, LOW);
    analogWrite(enB, i+x); //the speed  right
}

void moveBackward(){
  digitalWrite(in2, HIGH);
  digitalWrite(in4, HIGH);
  analogWrite(enA, 100); //the speed
  
  digitalWrite(in1, LOW);
  digitalWrite(in3, LOW);
  analogWrite(enB, 100); //the speed
}

void turnRight(int x){
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
    
    if(100<x<140){
    analogWrite(enA, 180); //the speed
    analogWrite(enB, 60); //the speed  
        }else{
      analogWrite(enB, 50); //the speed
      analogWrite(enA, 50*(1/x)); //the speed  
      
      }
      
  //delay(5);
}

void turnLeft(int x){
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW); 
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW); 
    if(140<x<180){
    analogWrite(enA, 50); 
    analogWrite(enB, 230); 
        }else{
      analogWrite(enA, 50);
      analogWrite(enB, 50*(1/x));
      }
        //delay(5);
} 
