 
#ifndef LIBRERIA_H
#define LIBRERIA_H
#include "mbed.h"


//------------------ Constantes Glovales -----------------------
const double senGyro = 0.0007575;        // traducir a mascara la sensivilidad del gyro : EJ: 0.0025 /3.3 *4095 = 3.10227272
const double senAcc = 0.2424242;        // traducir a mascara la sensivilidad del gyro : EJ: 0.8000 /3.3 *4095 = 992.7272727
const double RAD_A_GRADOS = 57.2957795;    // coeficiente para transformar Radianes en grados 
const double PI = 3.141;


// ------------ Variables glovales de la funcion IMU -----------
// Variables para uso del timer
unsigned long timer;

// Variables gloval del Angulo obtenido por el gyroscopio
double gyroAngle;

// Variables angulo del filtro complementario 
double F_Comp_Angulo;
double Angulo_Filtrado;

// Variables de lectura Zero
double zeroValue[2] = { 0 };

// Variables de modulo de control PID
float Kp = .1;
float Ki = 0;
float Kd = 0;
double lastError = 0;
double PID = 0;
double iTerm = 0;

// Variables de la funcion Torque
float Tolerancia_de_Parada = 0.5; 

// -------------------Define los pine de entrada --------------- 
AnalogIn accZ(p17);
AnalogIn accY(p18);
AnalogIn accX(p19);
AnalogIn gyro(p20);

// Define los pines conectados al driver de los motores
DigitalOut Mot1In1(p7);
DigitalOut Mot1In2(p6);        
DigitalOut Mot1STBY1(p5);
DigitalOut Mot2In1(p8);
DigitalOut Mot2In2(p9);
DigitalOut Mot2STBY2(p10);
PwmOut Mot1PWM(p21);
PwmOut Mot2PWM(p22);        
// ------------------ Declaraciones de funeciones --------------
void Torque(double PWM, float Max_Min);
double constrain(double Var,float Min,float Max);
double map(double x, double in_min, double in_max, double out_min, double out_max);

#endif
