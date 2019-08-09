 
#include <AFMotor.h>
#include <math.h>;
#include <Ultrasonic.h>

//Info importante, é sempre direita e 1 é sempre esquerda

const float pi = 3.1415; //aproximation of pi
float va[]={0,0}; // velocidade angular de cada roda
float v[]={0,0}; //velocidade real de cada roda em centímetros por segundo
float ta[]={0,0}; //Tempo da última atualização de encoder
int encoder[]={7,8}; //Pins for the encoders
bool aa[]={0,0};//valor do sensor do encoder anterior
float radius = 5; //raio das rodas em centímetros
float dist; //Dist para ativar o obstáculo

//Ultrassom
int trig = 9; //Porta do trigger
int echo = 10; //porta do Echo
Ultrasonic ultrasonic(trig, echo); //Definindo o Sensor para a biblioteca
  
//Motors
//0 = Right 1 = Left
int rM[]={5,6}; //portas pwm do motor da direita
int lM[] = {10,11}; //portas pwm do motor da esquerda
int pt[] = {255,255}; //Potencia atual dos dois motores
float k = 0.01;//Pwr changing constant


//calibration
bool calibr = true; // Ativa o modo calibração
int sE[]={A0,A1}; //Pin numbers for the sensors
float sMin[]={1024,1024}; //Max value of each sensor
float sMax[]={0,0};//minimum value of each sensor
float s[]={0,0};//current value of each sensor

//Color
int rL[]= {11,12,13}; //Portas dos leds da direita RGB
int lL[] = {0,1,2}; //porta dos leds da esquerda RGB

void Encode(int i){ //Calcula a velocidade da roda i
    bool a = digitalRead(encoder[i]);//If the sensor is on a slit or on an wall
    if(a!=aa[i]){//If a changed (the wheel moved)
      float t = millis(); //t receives the current time
      va[i] = (pi/20)*(t-ta[i]); //calculates the angular velocity
      ta[i] = t; //ta is updated to t
      v[i]=va[i]*radius; //calculates the real velocity
      aa[i]=a; //the old value of a is switched by the current a
      }
  }

void GetSen(){ //Changes s to the actual value of the sensor
    s[0]=analogRead(sE[0]);
    s[1]=analogRead(sE[1]); 
    }
  
float Norm(int i){
  return((s[i]-sMin[i])/(sMax[i]-sMin[i])); //returns a value between 0 and 1 for the sensor
  }
void setup() {
  //motors: Configura as portas e para os motores
  pinMode(lM[0],OUTPUT);
  pinMode(lM[1],OUTPUT);
  pinMode(rM[0],OUTPUT);
  pinMode(rM[1],OUTPUT);
  analogWrite(lM[0],0);
  analogWrite(lM[1],0);
  analogWrite(rM[0],0);
  analogWrite(rM[1],0);
  
  //Encoders
  pinMode(encoder[0],INPUT);
  pinMode(encoder[1],INPUT);

  Serial.begin(9600);
  pinMode(sE[0], INPUT); //porta do ldr da direita
  pinMode(sE[1], INPUT); //porta do ldr da esquerda
  pinMode(8,INPUT);//porta da calibração

  //Portas dos LEDs RGB
  for(int i;i<3;i++){
    pinMode(rL[i],OUTPUT);
    pinMode(lL[i],OUTPUT);
    }
  //Ligar os LEDS vermelhos
  digitalWrite(rL[0],HIGH); 
  digitalWrite(lL[0],HIGH);
  
}

void ChangeSpeed(float sp, int wheel){//Changes the speed o the wanted one. wheel 0 is left and 1 is right
  pt[wheel]+=(sp-v[wheel])*k;
}

void Ultrassom(){//Vê se tem obstáculo
  float cmMsec, inMsec;
  long microsec = ultrasonic.timing();
  cmMsec = ultrasonic.convert(microsec, Ultrasonic::CM);
  if(cmMsec<dist){
    Obstaculo();
    }
  }
void Obstaculo(){//Desvia do Obstáculo
  Serial.println("OBSTACULOOOOOO");
  
  }
void AllOff(){ //Desliga todos os LEDs
  for(int i;i<3;i++){
    digitalWrite(rL[i],LOW);
    digitalWrite(lL[i],LOW);
    }
  }
void CheckGreen(){//Checa pela encruzilhada
  int sEs[] = {0,0,0};//Armazena o valor do sensor para cada led ligado
  int sDir[] = {0,0,0};
  float taxa = 1.4; //O quão maior o verde tem de ser para considerar encruzilhada
  for(int i;i<3;i++){
  AllOff();//Desliga todos os LEDs
  digitalWrite(rL[i],HIGH);//Liga apenas uma cor
  digitalWrite(lL[i],HIGH);
  delay(10);//Espera o ldr mudar
  sEs[i] = analogRead(s[1]);//Salva o valor do ldr
  sDir[i]=analogRead(s[0]);
  }
  //Liga somente o vermelho para o sensor de luz funcionar
  AllOff();
  digitalWrite(rL[0],HIGH);
  digitalWrite(lL[0],HIGH);
  //Confere se o valor indica verde
  if(sEs[1]>sEs[0]*taxa&&sEs[1]>sEs[2]*taxa){Encruzilhada(1);}
  else if(sDir[1]>sDir[0]*taxa&&sDir[1]>sDir[2]*taxa)Encruzilhada(0);
  }
  
void Motors(){ //Altera a potência dos motores para o valor pt
  analogWrite(lM[0],pt[0]/2);
  analogWrite(lM[1],-pt[0]/2);
  analogWrite(rM[0],pt[1]/2);
  analogWrite(rM[1],-pt[1]/2);
  }
void SegueLinha(){ //Define a velocidade das rodas para seguir linha
  float s0 = Norm(0); //Salva o valor atual do sensor normalizado (Entre 0 e 1)
  float s1 = Norm(1);
  float d = s1-s0; //Acha a diferença entre os sensores
  ChangeSpeed(d*5,0);//Altera a velocidade das rodas de acordo com os sensores
  ChangeSpeed(-d*5,1);
  
  }
void loop() {
  GetSen();
  calibr = digitalRead(8);
 
  if(calibr){ // Calibração 
    for(int i=0;i<2;i++){
      if(s[i]>sMax[i]){
        sMax[i]=s[i];  
      }else if(s[i]<sMin[i]){
        sMin[i]=s[i];
      }
      }
    }
  else{ //Ação
    Motors(); //Atualiza a potência dos motores
    //Ultrassom();
    //CheckGreen();
    Encode(0);
    Encode(1);
    SegueLinha();
    }
}

