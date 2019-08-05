#include <math.h>;
const float pi = 3.1415;
float va[]={0,0};
float v[]={0,0};
float ta[]={0,0};
int encoder[]={6,7};
bool aa[]={0,0};//valor do sensor do encoder anterior
float radius = 5; 

void Encode(int i){
    bool a = digitalRead(encoder[i]);
    if(a!=aa[i]){
      Serial.println("MUDOU");
      float t = millis();
      va[i] = (pi/20)*(t-ta[i]);
      ta[i] = t;
      v[i]=va[i]*radius;
      aa[i]=a;
      //Serial.println("va: "+String(va[0]));
      }
  }

void setup() {
  // put your setup code here, to run once:
pinMode(encoder[0],INPUT);
pinMode(encoder[1],INPUT);

Serial.begin(9600);
}
void loop() {
  // put your main code here, to run repeatedly:
  Encode(0);
  //Serial.println("aa: "+String(aa[0])); 
   
 //Encode(1);
 
}
