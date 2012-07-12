#include "mbed.h"
#include "Libreria.h"

Timer t;
Serial mac(USBTX,USBRX);        // configura el purto usb 

int main(){
    mac.baud(115200);
    wait_ms(200);                        // espera al sensor para que se estabilice

    for (int i = 0; i < 200; ++i)
    {
        zeroValue[0] += gyro.read();
        zeroValue[1] += accY.read();

        wait_ms(10);
    }
    zeroValue[0] /= 200;
    zeroValue[1] /= 200;
    
    t.start();
 
     while(1){
    
        double gyroRate = (gyro.read()-zeroValue[0]) / senGyro;        // el resultado va a salir en &#65533;/s  (ojo correjir la sensibilidad )
        gyroAngle += gyroRate * ((double) (t.read_us()-timer)/1000000);        // ahora si devuelve el angulo en &#65533;             // siempre y cuando se trate de mili segundo
        
        double accYval = constrain(((double)(accY.read()-zeroValue[1])/senAcc), -1, 1);
        double accXangle = (asin(accYval)*RAD_A_GRADOS); 

        F_Comp_Angulo = (0.98*(F_Comp_Angulo+(gyroAngle*(double)(t.read_us()-timer)/1000000)))+(0.02*(accXangle)); 
        F_Comp_Angulo = constrain(F_Comp_Angulo, -10, 10);
       
        mac.printf("\n ACC ASin:    %f",F_Comp_Angulo);          

        double error = F_Comp_Angulo;
        double pTerm = Kp * error;
        iTerm += Ki * error;
        double dTerm = Kd * (error - lastError);
        lastError = error;
        PID = constrain((pTerm + iTerm + dTerm), -0.8, 0.8);

        Torque(PID,Tolerancia_de_Parada);
        
        timer = t.read_us();
    }  
}

// ------------------- Funciones --------------------------

void Torque(double PWM, float Max_Min){

        if( PWM < -Max_Min){
            Mot1STBY1 = 1;
            Mot2STBY2 = 1;
            Mot1In1 = 1;
            Mot1In2 = 0;
            Mot2In1 = 0;
            Mot2In2 = 1;    
                }
        else if (PWM > Max_Min){
            Mot1STBY1 = 1;
            Mot2STBY2 = 1;
            Mot1In1 = 0;
            Mot1In2 = 1;
            Mot2In1 = 1;
            Mot2In2 = 0;
                }
        else{
            Mot1STBY1 = 0;
            Mot2STBY2 = 0;
            Mot1In1 = 0;
            Mot1In2 = 0;
            Mot2In1 = 0;
            Mot2In2 = 0;
        }

        double torque_rueda = map(abs(PWM), 0, 1, 0.51, 0.84);         
        mac.printf("\n                        PID:%f",torque_rueda);   

        Mot1PWM = abs(torque_rueda);
        Mot2PWM = abs(torque_rueda);
}

double constrain(double Var,float Min,float Max){
    if (Var > Max)
    {
        return  Var = Max;
    }
    else if (Var < Min)
    {
        return Var = Min;
    }
    else 
        return Var; 
}

/*
//map(value, fromLow, fromHigh, toLow, toHigh)


value: el número (valor) a mapear.
fromLow: el límite inferior del rango actual del valor.
fromHigh: el límite superior del rango actual del valor.
toLow: límite inferior del rango deseado.
toHigh: límite superior del rango deseado.
*/
double map(double x, double in_min, double in_max, double out_min, double out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
