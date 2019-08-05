bool calibr = true;
int siz = 6;
int sE[]={A0,A1,A2,A3,A4,A5};
float sMin[]={1024,1024,1024,1024,1024,1024};
float sMax[]={0,0,0,0,0,0};
float s[]={0,0,0,0,0,0};


void getSen(){
  for(int i=0;i<siz;i++){
    s[i]=analogRead(sE[i]);
    }
  }
  
float norm(int i){
  return((s[i]-sMin[i])/(sMax[i]-sMin[i]));
  }
void setup() {
  Serial.begin(9600);
  for(int i = 0; i<siz;i++){
    pinMode(sE[i], INPUT);
    }
  pinMode(8,INPUT);
  // put your setup code here, to run once:
  
}

void loop() {
  // put your main code here, to run repeatedly:
  getSen();
  
  if(digitalRead(8)){
    calibr = !calibr;
    delay(500);
    Serial.println("BOTÃO!!!!!!!");
    }
  if(calibr){
    for(int i=0;i<siz;i++){
      if(s[i]>sMax[i]){
        sMax[i]=s[i];  
      }else if(s[i]<sMin[i]){
        sMin[i]=s[i];
      }
      }
      for(int i;i<siz;i++){
      Serial.print(" s"+String(i)+": "+String(s[i])+" mnMx: "+String(sMin[i])+"/"+String(sMax[i]));
      }
      Serial.println(' ');
    }
  else{
    delay(500);
    Serial.println("Normal");
    float maior = norm(0)-norm(5);
    float meio = norm(1)-norm(4);
    float menor = norm(2)-norm(3);
    Serial.println("s0: "+String(norm(0))+"s1: "+String(norm(1))+"s2: "+String(norm(2))+"s3: "+String(norm(3))+"s4: "+String(norm(4)))+"s5: "+String(norm(5));
    Serial.println("Maior: "+String(maior)+" Meio: "+String(meio)+" Menor: "+String(menor));
    }
}
