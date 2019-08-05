 
#include <AFMotor.h>
#include <math.h>;
const float pi = 3.1415; //aproximation of pi
float va[]={0,0}; // velocidade angular de cada roda
float v[]={0,0}; //velocidade real de cada roda em centímetros por segundo
float ta[]={0,0}; //Tempo da última atualização de encoder
int encoder[]={6,7}; //Pins for the encoders
bool aa[]={0,0};//valor do sensor do encoder anterior
float radius = 5; //raio das rodas em centímetros

//Motors
AF_DCMotor motor1(1); //Defines a motor on module 1 
AF_DCMotor motor2(3); 
int pt[] = {255,255}; //current wheel powers
float k = 0.01;//Pwr changing constant


//calibration
bool calibr = true; // Ativa o modo calibração
int siz = 4; //number of IR sensors
int sE[]={A0,A1,A2,A3}; //Pin numbers for the sensors
float sMin[]={1024,1024,1024,1024}; //Max value of each sensor
float sMax[]={0,0,0,0};//minimum value of each sensor
float s[]={0,0,0,0};//current value of each sensor

//motor1.run(FORWARD); 
//motor1.run(RELEASE);

void Encode(int i){
    bool a = digitalRead(encoder[i]);//If the sensor is on a slit or on an wall
    if(a!=aa[i]){//If a changed (the wheel moved)
      float t = millis(); //t receives the current time
      va[i] = (pi/20)*(t-ta[i]); //calculates the angular velocity
      ta[i] = t; //ta is updated to t
      v[i]=va[i]*radius; //calculates the real velocity
      aa[i]=a; //the old value of a is switched by the current a
      }
  }

void getSen(){ //Changes s to the actual value of the sensor
  for(int i=0;i<siz;i++){
    s[i]=analogRead(sE[i]); 
    }
  }
  
float norm(int i){
  return((s[i]-sMin[i])/(sMax[i]-sMin[i])); //returns a value between -1 and 1 for the sensor
  }
void setup() {
  //motors
  motor1.setSpeed(255); 
  motor2.setSpeed(255); 

  //Encoders
  pinMode(encoder[0],INPUT);
  pinMode(encoder[1],INPUT);

  Serial.begin(9600);
  for(int i = 0; i<siz;i++){
    pinMode(sE[i], INPUT);
    }
  pinMode(8,INPUT);
  
}

void ChangeSpeed(float sp, int wheel){//Changes the speed o the wanted one. wheel 0 is left and 1 is right
  pt[wheel]+=(sp-v[wheel])*k;

}
void loop() {
  getSen();

  calibr = digitalRead(8);
  if(calibr){
    for(int i=0;i<siz;i++){
      if(s[i]>sMax[i]){
        sMax[i]=s[i];  
      }else if(s[i]<sMin[i]){
        sMin[i]=s[i];
      }
      }
    }
  else{
    motor1.setSpeed(pt[0]); 
    motor2.setSpeed(pt[1]); 
    delay(10);
    Encode(0);
    Encode(1);
    
    float maior = norm(3)-norm(0);
    float meio = norm(2)-norm(1);
    float dif = (1*meio+2*maior);//Parei aqui
    
    
    }
}

