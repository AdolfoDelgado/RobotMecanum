/* 
Esta version ya ejecuta correctamente la cinematica y lee los encoders, aun me falta integrar el IMU, pero los encoders ya hacen su trabajo
al igual ya lo estoy conectando con ROS, hasta aqui mi reporte joaquin
ah por cierto me falta ajustar mas finamente el PI
 */
#include <math.h>
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



///////*variables para el conteo de pulsos e impresion serial*///////// 
bool primeraMuestra = true;
unsigned long LastTime = 0;
unsigned long SampleTime = 20;

unsigned long lastPrint = 0;
unsigned long PrintTime = 20;



//variables para el encoder1
volatile float angulo1 = 0;
volatile float rpm1 = 0;
volatile long Delta1 = 0;
volatile long muestran1 = 0;
volatile long n1Ant = 0;
volatile long n1=0;
volatile byte ant1=0;
volatile byte act1=0;
//variables para el encoder2
volatile float angulo2 = 0;
volatile float rpm2 = 0;
volatile long Delta2 = 0;
volatile long muestran2 = 0;
volatile long n2Ant = 0;
volatile long n2=0;
volatile byte ant2=0;
volatile byte act2=0;
//variables para el encoder3
volatile float angulo3 = 0;
volatile float rpm3 = 0;
volatile long Delta3 = 0;
volatile long muestran3 = 0;
volatile long n3Ant = 0;
volatile long n3=0;
volatile byte ant3=0;
volatile byte act3=0;
//variables para el encoder4
volatile float angulo4 = 0;
volatile float rpm4 = 0;
volatile long Delta4 = 0;
volatile long muestran4 = 0;
volatile long n4Ant = 0;
volatile long n4=0;
volatile byte ant4=0;
volatile byte act4=0;



///////*VARIABLES PARA EL CONTROL Y LECTURA VELOCIDADES LINEALES, ANGULARES*///////// 
float v1 = 0;
float v2 = 0;
float v3 = 0;
float v4 = 0;

volatile float w1 = 0;
volatile float w2 = 0;
volatile float w3 = 0;
volatile float w4 = 0;

float vx = 0;
float vy = 0;
float w = 0;

float x = 0.0;
float y = 0.0;
float Theta = 0.0;


///////*VARIABLES PARA EL SEGUIMIENTO DE TRAYECTORIAS Y ODOMETRIA *///////// 
float w1_d=0;
float w2_d=0;
float w3_d=0;
float w4_d=0;

float vx_d = 0;
float vy_d = 0;
float w_d = 0;

float x_d=0;
float y_d=0;
float Theta_d=0;

float Kp_pos = 0.8;
float Kp_ang = 1.5;

float Error_Xglobal;
float Error_Yglobal;
float Error_Theta;

                                            
///////*Informacióo para las variables para el calculo de cinematica*///////// 
float radioRueda = 0.03;
float LX = 0.0605; 
float LY = 0.0964;



///////*variables para el PID de los motores*///////// 
float error1=0;
float error2=0;
float error3=0;
float error4=0;

/* float error1Ant = 0;
float error2Ant = 0;
float error3Ant = 0;
float error4Ant = 0; */

float integral1=0;
float integral2=0;
float integral3=0;
float integral4=0;

/* float derivada1 = 0;
float derivada2 = 0;
float derivada3 = 0;
float derivada4 = 0;
 */
int pwm1=0;
int pwm2=0;
int pwm3=0;
int pwm4=0;

float kp = 22;
float Ki = 3.5;
//float kd = 0.1;
float dt = SampleTime / 1000.0;



///////*DECLARACIÓN DE FUNCIONES*///////// 
void encoder1 (void);
void encoder2 (void);
void encoder3 (void);
void encoder4 (void);

void angulos();
void muestras (void);
void DeltaEncoders (void);
void RPM (void);
void VelocidadAngular(void);
void VelocidadLineal(void);
void CinematicaDirecta(void);
void CinematicaInversa();
void Odometria();
void Motor(int motor, int pwm);
void PIDmotores();
void PSerial();

///////*FUNCIONES PRINCIPALES*///////// 
void setup(){
   Serial.begin(115200); 
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

    ledcAttach(M1forward, 1000, 10);
    ledcAttach(M1backward, 1000, 10); 
    ledcAttach(M2forward, 1000, 10);
    ledcAttach(M2backward, 1000, 10); 
    ledcAttach(M3forward, 1000, 10);
    ledcAttach(M3backward, 1000, 10); 
    ledcAttach(M4forward, 1000, 10);
    ledcAttach(M4backward, 1000, 10); 
    
}


void loop(){
    PSerial ();
    if(millis()- LastTime >= SampleTime || LastTime == 0 ){
        LastTime = millis();
        muestras();
        if(primeraMuestra){
            n1Ant = muestran1;
            n2Ant = muestran2;
            n3Ant = muestran3;
            n4Ant = muestran4;
            primeraMuestra = false;
        }
        else {
            DeltaEncoders();
            RPM();
            VelocidadAngular();
            VelocidadLineal();
            CinematicaDirecta();
            Odometria();
            angulos();
            
            //Objetivo(x_d,y_d,Theta_d);
            CinematicaInversa(vx_d, vy_d, w_d);
            PIDmotores();
        }
    }
    if (millis() - lastPrint >= PrintTime){
        lastPrint = millis();

        Serial.print(x); Serial.print(","); 
        Serial.print(y); Serial.print(",");
        Serial.print(Theta); Serial.print(",");
        Serial.print(vx); Serial.print(",");
        Serial.print(vy); Serial.print(",");
        Serial.print(w); Serial.print(",");
        Serial.print(angulo1); Serial.print(",");
        Serial.print(angulo2); Serial.print(",");
        Serial.print(angulo3); Serial.print(",");
        Serial.println(angulo4);
    } 
}


///////*DEFINICIÓN DE LAS FUNCIONES*///////// 
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
void angulos (void){
    angulo1 = (muestran1 / 410.0) * 2.0 * PI;
    angulo2 = (muestran2 / 410.0) * 2.0 * PI;
    angulo3 = (muestran3 / 410.0) * 2.0 * PI;
    angulo4 = (muestran4 / 410.0) * 2.0 * PI;
}
void muestras(void){
    noInterrupts();
    muestran1 = n1;
    muestran2 = n2;
    muestran3 = n3;
    muestran4 = n4;
    interrupts();
}
void DeltaEncoders (void){
   Delta1 = muestran1 - n1Ant;
   Delta2 = muestran2 - n2Ant;
   Delta3 = muestran3 - n3Ant;
   Delta4 = muestran4 - n4Ant;

   n1Ant = muestran1;
   n2Ant = muestran2;
   n3Ant = muestran3;
   n4Ant = muestran4;
}
void RPM (void){
   rpm1 = (Delta1 *  60000.0) / (410.0*SampleTime);
   rpm2 = (Delta2 *  60000.0) / (410.0*SampleTime);
   rpm3 = (Delta3 *  60000.0) / (410.0*SampleTime);
   rpm4 = (Delta4 *  60000.0) / (410.0*SampleTime);
}
void VelocidadAngular(void){
   w1 = rpm1 * 2.0 * PI / 60.0;
   w2 = rpm2 * 2.0 * PI / 60.0;
   w3 = rpm3 * 2.0 * PI / 60.0;
   w4 = rpm4 * 2.0 * PI / 60.0;
}
void VelocidadLineal(void){
   v1 = w1 * radioRueda;
   v2 = w2 * radioRueda;
   v3 = w3 * radioRueda;
   v4 = w4 * radioRueda;
}
void CinematicaDirecta(void){
   vx = (v1 + v2 + v3 + v4) / 4.0;
   vy = (-v1 + v2 - v3 + v4) / 4.0;
   w  = (-v1 + v2 + v3 - v4) / (4.0 * (LX+LY));
}
void CinematicaInversa(float vxD, float vyD, float wD){
    w1_d = (vxD - vyD - (LX + LY) * wD) / radioRueda;
    w2_d = (vxD + vyD + (LX + LY) * wD) / radioRueda;
    w3_d = (vxD - vyD + (LX + LY) * wD) / radioRueda;
    w4_d = (vxD + vyD - (LX + LY) * wD) / radioRueda;
}
void Odometria(){    
    x += (vx * cos(Theta) - vy * sin(Theta)) * dt;
    y += (vx * sin(Theta) + vy * cos(Theta)) * dt;
    Theta += w * dt;
}
void Motor(int motor, int pwm){
    int pwmMin = 60;

    if(abs(pwm) <= 20) pwm = 0;
    else if(pwm >= 21) pwm += pwmMin;
    else pwm -= pwmMin;

    pwm = constrain(pwm, -1023, 1023);

    int pinFwd, pinBwd;

    if(motor == 1){ pinFwd = M1forward; pinBwd = M1backward; }
    else if(motor == 2){ pinFwd = M2forward; pinBwd = M2backward; }
    else if(motor == 3){ pinFwd = M3forward; pinBwd = M3backward; }
    else if(motor == 4){ pinFwd = M4forward; pinBwd = M4backward; }

    if(pwm > 0){
        ledcWrite(pinFwd, pwm);
        ledcWrite(pinBwd, 0);
    }
    else if(pwm < 0){
        ledcWrite(pinFwd, 0);
        ledcWrite(pinBwd, -pwm);
    }
    else{
        ledcWrite(pinFwd, 0);
        ledcWrite(pinBwd, 0);
    }
}
void PIDmotores(){
    if(w1_d == 0) integral1 = 0;
    if(w2_d == 0) integral2 = 0;
    if(w3_d == 0) integral3 = 0;
    if(w4_d == 0) integral4 = 0;

    error1=w1_d-w1;
    integral1 += error1 * dt;
    integral1 = constrain (integral1, -300, 300);
    //derivada1 = (error1 - error1Ant) / dt;
    pwm1=kp * error1 + Ki * integral1; //+ kd * derivada1;

    error2=w2_d-w2;
    integral2 += error2 * dt;
    integral2 = constrain (integral2, -300, 300);
    //derivada2 = (error2 - error2Ant) / dt;
    pwm2=kp * error2 + Ki * integral2 ;//+ kd * derivada2;

    error3=w3_d-w3;
    integral3 += error3 * dt;
    integral3 = constrain (integral3, -300, 300);
    //derivada3 = (error3 - error3Ant) / dt;
    pwm3=kp * error3 + Ki * integral3; //+ kd * derivada3;

    error4=w4_d-w4;
    integral4 += error4 * dt;
    integral4 = constrain (integral4, -300, 300);
    //derivada4 = (error4 - error4Ant) / dt;
    pwm4=kp * error4 + Ki * integral4; //+ kd * derivada4;

    Motor(1, pwm1);
    Motor(2, pwm2);
    Motor(3, pwm3);
    Motor(4, pwm4);
/*     error1Ant = error1;
    error2Ant = error2;
    error3Ant = error3;
    error4Ant = error4; */
}
void Objetivo(float xD, float yD, float ThetaD){
    Error_Xglobal = xD - x;
    Error_Yglobal = yD - y;
    Error_Theta = ThetaD - Theta;
    Error_Theta = atan2(sin(Error_Theta), cos(Error_Theta));

    float distancia = sqrt(Error_Xglobal * Error_Xglobal + Error_Yglobal * Error_Yglobal);

    if(distancia < 0.05 && fabs(Error_Theta) < 0.05)
    {
        vx_d = 0;
        vy_d = 0;
        w_d = 0;
        return;
    }

    float cosT = cos(Theta);
    float sinT = sin(Theta);

    // Error global -> error en marco del robot
    float Error_Xrobot =  cosT * Error_Xglobal + sinT * Error_Yglobal;
    float Error_Yrobot = -sinT * Error_Xglobal+ cosT * Error_Yglobal;

    vx_d = Kp_pos * Error_Xrobot;
    vy_d = Kp_pos * Error_Yrobot;
    w_d  = Kp_ang * Error_Theta;

    vx_d = constrain(vx_d, -0.4, 0.4);
    vy_d = constrain(vy_d, -0.4, 0.4);
    w_d  = constrain(w_d, -1.0, 1.0);
}
void PSerial (){
    if(Serial.available())
    {
        String comando = Serial.readStringUntil('\n');
        comando.trim();

        int coma1 = comando.indexOf(',');
        int coma2 = comando.indexOf(',', coma1 + 1);
        ///para el ir a objetivo
/*         if(coma1 > 0 && coma2 > coma1)
        {
            float nuevoX = comando.substring(0, coma1).toFloat();
            float nuevoY = comando.substring(coma1 + 1, coma2).toFloat();
            float nuevoTheta = comando.substring(coma2 + 1).toFloat();

            x_d = nuevoX;
            y_d = nuevoY;
            Theta_d = nuevoTheta * PI / 180.0;

            Serial.println("Nuevo objetivo recibido:");
            Serial.print("x_d: "); Serial.println(x_d);
            Serial.print("y_d: "); Serial.println(y_d);
            Serial.print("Theta_d rad: "); Serial.println(Theta_d);
        } */
        ///para velocidades deseadas
        if(coma1 > 0 && coma2 > coma1)
        {
            float nuevoVXD = comando.substring(0, coma1).toFloat();
            float nuevoVYD = comando.substring(coma1 + 1, coma2).toFloat();
            float nuevoWD = comando.substring(coma2 + 1).toFloat();

            vx_d = nuevoVXD;
            vy_d = nuevoVYD;
            w_d = nuevoWD;

            //Serial.println("Nueva velocidad recibida:");
            //Serial.print("vx_d: "); 
            Serial.println(vx_d);
            //Serial.print("vy_d: "); 
            Serial.println(vy_d);
            //Serial.print("w_d rad: "); 
            Serial.println(w_d);
        }
    }
}