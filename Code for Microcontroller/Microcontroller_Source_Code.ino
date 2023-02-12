#include <VarSpeedServo.h>
#include <AccelStepper.h>
#include <MultiStepper.h>


#define limit1 4
#define limit2 5
#define limit3 6
#define limit4 7
AccelStepper stepper1(AccelStepper::HALF4WIRE,22,24,26,28);
AccelStepper stepper2(AccelStepper::HALF4WIRE,23,25,27,29);
AccelStepper stepper3(AccelStepper::HALF4WIRE,31,33,35,37);
AccelStepper stepper4(AccelStepper::FULL4WIRE,41,45,39,43);
VarSpeedServo Gripper;
String masuk,keluar;
int mode = 0;
int motor1[5];
int motor2[5];
int motor3[5];
int motor4[5];
unsigned long counter,previouscounter;
void setup() {
Gripper.attach(9);
Serial.begin(57600);
pinMode(limit1, INPUT_PULLUP);
pinMode(limit2, INPUT_PULLUP);
pinMode(limit3, INPUT_PULLUP);
pinMode(limit4, INPUT_PULLUP);
}

void loop() {
if (Serial.available()>0){masuk = Serial.readString();}
mode = (getValue(masuk,';',0)).toInt();
if (mode==1){
  Calibration(masuk);
  masuk = "";
  Serial.read();
}
else if (mode==2){
  kinematic(masuk);
  masuk="";
  Serial.read();
}
else if(mode==3){
  single_pick(masuk);
  masuk="";
  Serial.read();
}
counter = millis();
if ((counter-previouscounter)>=2000){
updateProgram();
previouscounter = millis();}
}
void Calibration(String masuk){
    motor1[0] = (getValue(masuk,';',1)).toInt();
    motor1[1] = (getValue(masuk,';',2)).toInt();
    motor1[2] = stepper1.currentPosition();
    motor1[3] = (getValue(masuk,';',3)).toInt();
    motor1[4] = (getValue(masuk,';',4)).toInt();
    motor2[0] = (getValue(masuk,';',5)).toInt();
    motor2[1] = (getValue(masuk,';',6)).toInt();
    motor2[2] = stepper2.currentPosition();
    motor2[3] = (getValue(masuk,';',7)).toInt();
    motor2[4] = (getValue(masuk,';',8)).toInt();
    motor3[0] = (getValue(masuk,';',9)).toInt();
    motor3[1] = (getValue(masuk,';',10)).toInt();
    motor3[2] = stepper3.currentPosition();
    motor3[3] = (getValue(masuk,';',11)).toInt();
    motor3[4] = (getValue(masuk,';',12)).toInt();
    motor4[0] = (getValue(masuk,';',13)).toInt();
    motor4[1] = (getValue(masuk,';',14)).toInt();
    motor4[2] = stepper4.currentPosition();
    motor4[3] = (getValue(masuk,';',15)).toInt();
    motor4[4] = (getValue(masuk,';',16)).toInt();
    stepper1.setMaxSpeed(motor1[0]);
    stepper1.setAcceleration(motor1[1]);
    stepper2.setMaxSpeed(motor2[0]);
    stepper2.setAcceleration(motor2[1]);
    stepper3.setMaxSpeed(motor3[0]);
    stepper3.setAcceleration(motor3[1]);
    stepper4.setMaxSpeed(motor4[0]);
    stepper4.setAcceleration(motor4[1]);
    //===========================================================================================================
    if(motor1[3] == 1){
      while(!digitalRead(limit1)){
        stepper1.setSpeed(1000);stepper1.runSpeed();
      }
      stepper1.setCurrentPosition(0);
      stepper1.moveTo(-9500);
      stepper1.runToPosition();
      stepper1.setCurrentPosition(0);
      stepper1.moveTo(1000);
      stepper1.runToPosition();
    }
    else if (motor1[3] == 2){
      stepper1.moveTo(stepper1.currentPosition()+(motor1[4]/0.006666));
      stepper1.runToPosition();
      stepper1.setCurrentPosition(0);
    }
    //============================================================================================================
    if(motor2[3] == 1){
      while(!digitalRead(limit2)){
        stepper2.setSpeed(-100);stepper2.runSpeed();
      }
      stepper2.setCurrentPosition(-410);
      stepper2.moveTo(0);
      stepper2.runToPosition();
    }
    else if (motor2[3] == 2){
      stepper2.moveTo(stepper3.currentPosition()+(motor2[4]/0.2));
      stepper2.runToPosition();
      stepper2.setCurrentPosition(0);
    }  
    //============================================================================================================ 
    if(motor3[3] == 1){
      while(!digitalRead(limit2)){
        stepper2.setSpeed(-100);stepper2.runSpeed();
      }
      stepper2.setCurrentPosition(-400);
      stepper2.moveTo(-405);
      stepper2.runToPosition();
      while(!digitalRead(limit3)){
        stepper3.setSpeed(100);stepper3.runSpeed();
      }
      stepper3.setCurrentPosition(850);
      stepper2.moveTo(0);
      stepper2.runToPosition();
      stepper3.moveTo(0);
      stepper3.runToPosition();
    }
    else if (motor3[3] == 2){
      stepper3.moveTo(stepper3.currentPosition()+(motor3[4]/0.2));
      stepper3.runToPosition();
      stepper3.setCurrentPosition(0);
    }   
    //============================================================================================================
    if(motor4[3] == 2){
      stepper4.moveTo(stepper4.currentPosition()+(motor4[4]/0.142));
      stepper4.runToPosition();
      stepper4.setCurrentPosition(0);
    }   
    
}
void updateProgram(){
  int S1s = stepper1.maxSpeed();
  int S1p = stepper1.currentPosition()*0.006666;
  int S2s = stepper2.maxSpeed();
  int S2p = stepper2.currentPosition()*0.2;
  int S3s = stepper3.maxSpeed();
  int S3p = stepper3.currentPosition()*0.2;
  int S4s = stepper4.maxSpeed();
  int S4p = stepper4.currentPosition()*0.142;
  String data = String(mode)+";"+String(S1s)+";"+String(S1p)+";"+String(S2s)+";"+String(S2p)+";"+String(S3s)+";"+String(S3p)+";"+String(S4s)+";"+String(S4p);
  Serial.println(data);
}
void kinematic(String masuk){
  stepper1.moveTo((getValue(masuk,';',1)).toInt()/0.006666);
  stepper2.moveTo((getValue(masuk,';',2)).toInt()/0.2);
  stepper3.moveTo((getValue(masuk,';',3)).toInt()/0.2);
  stepper4.moveTo((getValue(masuk,';',4)).toInt()/0.142);
  int grip = (getValue(masuk,';',5)).toInt();
  grip = map(grip,20,50,180,70);
  while(!(stepper1.distanceToGo()==0 & stepper2.distanceToGo()==0 & stepper3.distanceToGo()==0 & stepper4.distanceToGo()==0)){
    stepper1.run();
    stepper2.run();
    stepper3.run();
    stepper4.run();
  }
  Gripper.write(grip,30,true);
}

void single_pick(String masuk){
  stepper1.moveTo(30/0.006666);
  stepper2.moveTo((getValue(masuk,';',2)).toInt()/0.2);
  stepper3.moveTo((getValue(masuk,';',3)).toInt()/0.2);
  stepper4.moveTo((getValue(masuk,';',4)).toInt()/0.142);
  int grip = 30 ;
  grip = map(grip,20,50,180,70);
  while(!(stepper1.distanceToGo()==0 & stepper2.distanceToGo()==0 & stepper3.distanceToGo()==0 & stepper4.distanceToGo()==0)){
    stepper1.run();
    stepper2.run();
    stepper3.run();
    stepper4.run();
  }
  Gripper.write(70,50,true);
  stepper1.moveTo(0);
  stepper1.runToPosition();
  Gripper.write(grip,50,true);
  stepper1.moveTo(5/0.006666);
  stepper1.runToPosition();
  stepper1.moveTo(10/0.006666);
  stepper2.moveTo(45/0.2);
  stepper3.moveTo(80/0.2);
  stepper4.moveTo(-125/0.142);
  while(!(stepper1.distanceToGo()==0 & stepper2.distanceToGo()==0 & stepper3.distanceToGo()==0 & stepper4.distanceToGo()==0)){
    stepper1.run();
    stepper2.run();
    stepper3.run();
    stepper4.run();
  }
  stepper1.moveTo(0);
  stepper1.runToPosition();
  Gripper.write(70,50,true);
  stepper1.moveTo(10/0.006666);
  stepper1.runToPosition();
  stepper1.moveTo(40/0.006666);
  stepper2.moveTo(90/0.2);
  stepper3.moveTo(90/0.2);
  stepper4.moveTo(45/0.142);
  while(!(stepper1.distanceToGo()==0 & stepper2.distanceToGo()==0 & stepper3.distanceToGo()==0 & stepper4.distanceToGo()==0)){
    stepper1.run();
    stepper2.run();
    stepper3.run();
    stepper4.run();
  }
}
String getValue(String data, char separator, int index)
{
  int found = 0;
  int strIndex[] = {0, -1};
  int maxIndex = data.length()-1;

  for(int i=0; i<=maxIndex && found<=index; i++){
    if(data.charAt(i)==separator || i==maxIndex){
        found++;
        strIndex[0] = strIndex[1]+1;
        strIndex[1] = (i == maxIndex) ? i+1 : i;
    }
  }
  return found>index ? data.substring(strIndex[0], strIndex[1]) : "";
}
