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

////////*variables para la implementación de cinematica*////////
float w=0;
float vx=0;
float vy=0;



volatile int n=0;
volatile byte ant=0;
volatile byte act=0;

unsigned long lastTime=0;
unsigned long sampleTime=100;

void setup(){
    Serial.begin (9600);
    pinMode(M2Aencoder, INPUT);
    pinMode(M2Bencoder, INPUT);

    attachInterrupt (digitalPinToInterrupt(M2Aencoder), encoder, CHANGE);
    attachInterrupt (digitalPinToInterrupt(M2Bencoder), encoder, CHANGE);

    Serial.println("Numero de conteos");
}

void loop(){
    if(millis()-lastTime >= sampleTime || lastTime==0){
        lastTime = millis();
        Serial.print("Numero de cuentas: ");Serial.println(n);
    }
}

void encoder (void){
    ant=act;
    if (digitalRead(M2Aencoder)) 
        bitSet(act,1);
    else bitClear(act,1);
    if (digitalRead(M2Bencoder))
        bitSet(act,0);
    else bitClear(act,0);
    
    if(ant==2 && act == 0 ) n++;
    if(ant==0 && act == 1 ) n++;
    if(ant==3 && act == 2 ) n++;
    if(ant==1 && act == 3 ) n++;


    if(ant==1 && act == 0 ) n--;
    if(ant==3 && act == 1 ) n--;
    if(ant==0 && act == 2 ) n--;
    if(ant==2 && act == 3 ) n--;

}