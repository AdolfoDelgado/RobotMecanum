///////*configuración para mis drivers*///////// 
//driver para el motor 1
#define M1forward 11
#define M1backward 10
//Driver para el motor 2
#define M2forward 17
#define M2backward 16
//Driver para el motor 3
#define M3forward 12
#define M3backward 35
//Driver para el motor 4
#define M4forward 14
#define M4backward 36

///////*configuración para mis encoders*///////// 
//encoder para el motor 1
#define M1Aencoder 7
#define M1Bencoder 6
//encoder para el motor 2
#define M2Aencoder 4
#define M2Bencoder 5
//encoder para el motor 3
#define M3Aencoder 40
#define M3Bencoder 41
//encoder para el motor 4
#define M4Aencoder 38
#define M4Bencoder 39

///////*variables para el conteo de pulsos*///////// 
unsigned long LastTime = 0;
unsigned long SampleTime = 100;

//variables para el encoder1
volatile int rpm1 = 0;
volatile int Delta1 = 0;
volatile int n1Ant = 0;
volatile int n1=0;
volatile byte ant1=0;
volatile byte act1=0;
//variables para el encoder2
volatile int rpm2 = 0;
volatile int Delta2 = 0;
volatile int n2Ant = 0;
volatile int n2=0;
volatile byte ant2=0;
volatile byte act2=0;
//variables para el encoder3
volatile int rpm3 = 0;
volatile int Delta3 = 0;
volatile int n3Ant = 0;
volatile int n3=0;
volatile byte ant3=0;
volatile byte act3=0;
//variables para el encoder4
volatile int rpm4 = 0;
volatile int Delta4 = 0;
volatile int n4Ant = 0;
volatile int n4=0;
volatile byte ant4=0;
volatile byte act4=0;

///////*funciones para los encoders*///////// 
void encoder1 (void);
void encoder2 (void);
void encoder3 (void);
void encoder4 (void);
void DeltaEncoders (void);
void RPM (void);

void setup(){
   Serial.begin(9600); 
   pinMode(M1Aencoder, INPUT);
   pinMode(M1Bencoder, INPUT);
   pinMode(M2Aencoder, INPUT);
   pinMode(M2Bencoder, INPUT);
   pinMode(M3Aencoder, INPUT);
   pinMode(M3Bencoder, INPUT);
   pinMode(M4Aencoder, INPUT);
   pinMode(M4Bencoder, INPUT);

   attachInterrupt (digitalPinToInterrupt(M1Aencoder), encoder1, CHANGE);
   attachInterrupt (digitalPinToInterrupt(M1Bencoder), encoder1, CHANGE);
   attachInterrupt (digitalPinToInterrupt(M2Aencoder), encoder2, CHANGE);
   attachInterrupt (digitalPinToInterrupt(M2Bencoder), encoder2, CHANGE);
   attachInterrupt (digitalPinToInterrupt(M3Aencoder), encoder3, CHANGE);
   attachInterrupt (digitalPinToInterrupt(M3Bencoder), encoder3, CHANGE);
   attachInterrupt (digitalPinToInterrupt(M4Aencoder), encoder4, CHANGE);
   attachInterrupt (digitalPinToInterrupt(M4Bencoder), encoder4, CHANGE);
    
}


void loop(){
   if(millis()- LastTime >= SampleTime || LastTime == 0 ){
      LastTime = millis();
      DeltaEncoders();
      RPM();
      Serial.print("D1: ");Serial.print(Delta1);Serial.print("||  RPM1: ");Serial.println(rpm1);
      Serial.print("D2: ");Serial.print(Delta2);Serial.print("||  RPM2: ");Serial.println(rpm2);
      Serial.print("D3: ");Serial.print(Delta3);Serial.print("||  RPM3: ");Serial.println(rpm3);
      Serial.print("D4: ");Serial.print(Delta4);Serial.print("||  RPM4: ");Serial.println(rpm4)
   }
}

void encoder1 (void){
    ant1=act1;
    if (digitalRead(M1Aencoder)) 
        bitSet(act1,1);
    else bitClear(act1,1);
    if (digitalRead(M1Bencoder))
        bitSet(act1,0);
    else bitClear(act1,0);
    
    if(ant1 ==2 && act1 == 0 ) n1++;
    if(ant1 ==0 && act1 == 1 ) n1++;
    if(ant1 ==3 && act1 == 2 ) n1++;
    if(ant1 ==1 && act1 == 3 ) n1++;


    if(ant1 ==1 && act1 == 0 ) n1--;
    if(ant1 ==3 && act1 == 1 ) n1--;
    if(ant1 ==0 && act1 == 2 ) n1--;
    if(ant1 ==2 && act1 == 3 ) n1--;

}
void encoder2 (void){
    ant2=act2;
    if (digitalRead(M2Aencoder)) 
        bitSet(act2,1);
    else bitClear(act2,1);
    if (digitalRead(M2Bencoder))
        bitSet(act2,0);
    else bitClear(act2,0);
    
    if(ant2 ==2 && act2 == 0 ) n2--;
    if(ant2 ==0 && act2 == 1 ) n2--;
    if(ant2 ==3 && act2 == 2 ) n2--;
    if(ant2 ==1 && act2 == 3 ) n2--;


    if(ant2 ==1 && act2 == 0 ) n2++;
    if(ant2 ==3 && act2 == 1 ) n2++;
    if(ant2 ==0 && act2 == 2 ) n2++;
    if(ant2 ==2 && act2 == 3 ) n2++;

}
void encoder3 (void){
    ant3=act3;
    if (digitalRead(M3Aencoder)) 
        bitSet(act3,1);
    else bitClear(act3,1);
    if (digitalRead(M3Bencoder))
        bitSet(act3,0);
    else bitClear(act3,0);
    
    if(ant3 ==2 && act3 == 0 ) n3--;
    if(ant3 ==0 && act3 == 1 ) n3--;
    if(ant3 ==3 && act3 == 2 ) n3--;
    if(ant3 ==1 && act3 == 3 ) n3--;


    if(ant3 ==1 && act3 == 0 ) n3++;
    if(ant3 ==3 && act3 == 1 ) n3++;
    if(ant3 ==0 && act3 == 2 ) n3++;
    if(ant3 ==2 && act3 == 3 ) n3++;

}
void encoder4 (void){
    ant4=act4;
    if (digitalRead(M4Aencoder)) 
        bitSet(act4,1);
    else bitClear(act4,1);
    if (digitalRead(M4Bencoder))
        bitSet(act4,0);
    else bitClear(act4,0);
    
    if(ant4 ==2 && act4 == 0 ) n4++;
    if(ant4 ==0 && act4 == 1 ) n4++;
    if(ant4 ==3 && act4 == 2 ) n4++;
    if(ant4 ==1 && act4 == 3 ) n4++;


    if(ant4 ==1 && act4 == 0 ) n4--;
    if(ant4 ==3 && act4 == 1 ) n4--;
    if(ant4 ==0 && act4 == 2 ) n4--;
    if(ant4 ==2 && act4 == 3 ) n4--;

}
void DeltaEncoders (void){
   Delta1 = n1 - n1Ant;
   Delta2 = n2 - n2Ant;
   Delta3 = n3 - n3Ant;
   Delta4 = n4 - n4Ant;

   n1Ant = n1;
   n2Ant = n2;
   n3Ant = n3;
   n4Ant = n4;
/*    Serial.print("D1: ");Serial.print(Delta1);
   Serial.print("D2: ");Serial.print(Delta2);
   Serial.print("D3: ");Serial.print(Delta3);
   Serial.print("D4: ");Serial.print(Delta4); */
}
void RPM (void){
   rpm1 = (Delta1 * 600.0) / 410.0;
   rpm2 = (Delta2 * 600.0) / 410.0;
   rpm3 = (Delta3 * 600.0) / 410.0;
   rpm4 = (Delta4 * 600.0) / 410.0;

/*    Serial.print("RPM1: ");Serial.println(rpm1);
   Serial.print("RPM2: ");Serial.println(rpm2);
   Serial.print("RPM3: ");Serial.println(rpm3);
   Serial.print("RPM4: ");Serial.println(rpm4); */
}