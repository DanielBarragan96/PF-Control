
#include <TimerOne.h>
#include "Adafruit_VL53L0X.h"

//----------------------------------------------
//-----------------Constants--------------------
//----------------------------------------------
#define X_MIN_LIM 5
#define X_MAX_LIM 30
#define Y_MIN_LIM 10
#define Y_MAX_LIM 25
#define T_STEP    0.0005  //T usada para el PID
#define PWM1 6
#define PWM1 5

//----------------------------------------------
//-------------GANANCIAS PID--------------------
//----------------------------------------------
#define KP        2    //valor de KP
#define KI        0.1   //valor de KI
#define KD        0.0 //valor de KD

//----------------------------------------------
//--------------Step Sequence-------------------
//----------------------------------------------
int paso [4][4] =
{
  {1, 0, 0, 0},
  {0, 1, 0, 0},
  {0, 0, 1, 0},
  {0, 0, 0, 1}
};

//----------------------------------------------
//-----------------Functions--------------------
//----------------------------------------------
void rotate_clockwise();
void rotate_counterclockwise();
int Medir_distancia_x();
int Medir_distancia_y();
void motor_x_near(int freq_x);
void motor_x_far(int freq_x);
void motor_y_near(int freq_y);
void motor_y_far(int freq_y);
void stop_motor_x();
void stop_motor_y();
int motor_x(int ref);
int motor_y(int ref_y);
int scan();

//----------------------------------------------
//--------------Global Variables----------------
//----------------------------------------------
int ref_y = 15;
int ref_x = 15;
Adafruit_VL53L0X lox = Adafruit_VL53L0X();

//----------------------------------------------
//--------------------PINS----------------------
//----------------------------------------------
int Mtr_ctr1_x   = 5;    int Mtr_ctr2_x   = 6;    
int Mtr_ctr1_y   = 10;   int Mtr_ctr2_y   = 11;    
int Echo_1       = 2;    int Trigger_1    = 3;   
int Echo_2       = 7;    int Trigger_2    = 8;    
int StepMtr_ctr1 = 4;    int StepMtr_ctr2 = 9;   
int StepMtr_ctr3 = 12;   int StepMtr_ctr4 = 13;

//----------------------------------------------
//-----------------Variables--------------------
//----------------------------------------------
long distancia_x;        long tiempo_x;           
long distancia_y;        long tiempo_y;
int ir_sensor0 = A0;
int counter =                     0.00;
float ref =                       0.00; 
//Errores
float ek =                       0.0;    //Error actual
float ek_1 =                     0.0;    //Error anterior
//Variables del PID
double UP =                       0.0;
double UI =                       0.0;
double UD =                       0.0;
//Entradas
double Uk =                       0.0;    //Entrada actual
double Uk_1 =                     0.0;    //Entrada pasada
double Uk_calc =                  0.0;    //Entrada actual


//----------------------------------------------
//---------------Interrupción-------------------
//----------------------------------------------
void ISR_func() {
//  counter++;
//  if(counter >= SIN_SIZE)
//  {
//    counter = 1;
//  }

    //ref = SIN_FUNC[counter];
    Serial.print(" Inside ");
    int pos_x = scan();//Medir_distancia_x();
    Serial.println(pos_x);
    ek = ref_x - pos_x;                      //Error actual

    if(ek==0)
    {
      if(ref_x==25)
        ref_x = 15;
      else
        ref_x = 25;   
    }
    //PID
    UP = KP*ek;
    UI = (Uk_1 + KI*T_STEP*ek);
    UD = (KD*(ek - ek_1)/T_STEP);
    Uk = UP;// + UI + UD;//Entrada necesaria
    Uk_1 = UI;                                            //Actualizar valor de la entrada
    Uk_calc = Uk;
         
    int cicle = (255*abs(Uk)/8.5);
    
    if(255<cicle) cicle = 255;
    if(100>cicle) cicle = 100;

    if(ek>0)
    {
      motor_y_far(cicle);
    }
    else if(ek<0)
    {
      motor_y_near(cicle);
    }
    Serial.print(pos_x);
    Serial.print(" - Uk: ");
    Serial.print(Uk);
    Serial.print(" - cicles: ");
    Serial.println(cicle);
}


//----------------------------------------------
//--------------Initialization------------------
//----------------------------------------------
void setup() {
  //asignacion de pines 
  //imprimir en puerto serial
  Serial.begin(9600);
  
  // Para el ultrasonico 1
  pinMode(Trigger_1, OUTPUT); 
  pinMode(Echo_1, INPUT); 

  // Para el ultrasonico 2
  pinMode(Trigger_2, OUTPUT); 
  pinMode(Echo_2, INPUT); 
  
  //Para el motor a pasos 
  pinMode(StepMtr_ctr1, OUTPUT); 
  pinMode(StepMtr_ctr2, OUTPUT); 
  pinMode(StepMtr_ctr3, OUTPUT); 
  pinMode(StepMtr_ctr4, OUTPUT);  

  // Iniciar sensor
  Serial.println("VL53L0X test");
  if (!lox.begin()) {
    Serial.println(F("Error al iniciar VL53L0X"));
    while(1);
  }

//  Timer1.initialize(500);
//  Timer1.attachInterrupt(ISR_func);
}

//----------------------------------------------
//-------------------Loop-----------------------
//----------------------------------------------
void loop() {
ISR_func();
delay(400);
//  scan();
//  Serial.println(Medir_distancia_x());
//  Serial.println("   ");
//  delay(500);
//  int lectura = analogRead(ir_sensor0); // lectura del sensor 0
//  int cm = pow(3027.4 / lectura, 1.2134); // conversión a centímetros
//  Serial.print("Sensor 0: ");
//  Serial.println(cm); // lectura del sensor 0
//  delay(500); // tiempo de espera

  
//    int x = Medir_distancia_x();
//    int y = Medir_distancia_y();
//  Serial.print(" Distancia X: "); //Imprimimos "Distancia" sobre el Monitor Serial
//  Serial.print(x); //Mostramos el Valor de la distancia real sobre el Monitor Serial  
//  Serial.print(" - Distancia Y: "); //Imprimimos "Distancia" sobre el Monitor Serial
//  Serial.println(y); //Mostramos el Valor de la distancia real sobre el Monitor Serial  

  
//  int motor_status_x = motor_x(ref_x);
//  Serial.print(" Distancia X: "); //Imprimimos "Distancia" sobre el Monitor Serial
//  Serial.print(motor_status_x); //Mostramos el Valor de la distancia real sobre el Monitor Serial  
//  if(motor_status_x==-1 && ref_y==20)
//  {
//    ref_y = 25;
//    rotate_clockwise();
//  }
//  else if(motor_status_x==-1 && ref_y==25)
//  {
//    ref_y = 20;
//    rotate_counterclockwise();
//  }
//  delay(200); //Cada que Tiempo se imprimira el valor de la distancia
//  
//  int motor_status_y = motor_y(ref_y);
//  Serial.print(" - Distancia Y: "); //Imprimimos "Distancia" sobre el Monitor Serial
//  Serial.println(motor_status_y); //Mostramos el Valor de la distancia real sobre el Monitor Serial  
//  if(motor_status_y==-1 && ref_y==15)
//  {
//    ref_y = 20;
//    rotate_clockwise();
//  }
//  else if(motor_status_y==-1 && ref_y==20)
//  {
//    ref_y = 15;
//    rotate_counterclockwise();
//  }  
//  delay(200); //Cada que Tiempo se imprimira el valor de la distancia
}

//----------------------------------------------
//------------Distance Measurements-------------
//----------------------------------------------
int Medir_distancia_x(){
  int distancia_x_array[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
  for(int i = 1; i<=10; i++)
  {
    //Para el ultrasonico 1 "x"
    digitalWrite(Trigger_1,LOW); //Para darle estabilización al sensor
    delayMicroseconds(5); //Tiempo de 5 micro segundos
    digitalWrite(Trigger_1, HIGH); //Enviamos el pulso ultrasónico para activar el sensor
    delayMicroseconds(10); //Con una duracion de 10 micro segundos
    tiempo_x = pulseIn(Echo_1, HIGH); //Función para medir la longitud del pulso entrante, mide el tiempo transcurrido de ida y vuelta
    distancia_x = int(0.017*tiempo_x); //Fórmula para calcular la distancia obteniendo un valor entero
    if(distancia_x>35 && distancia_x<1)
    {
      i--;
    }
    else
    {
      distancia_x_array[i-1] = (int) distancia_x;
    }
  }
//  Serial.println(distancia_x_array[0]);
//  Serial.println(distancia_x_array[1]);
//  Serial.println(distancia_x_array[2]);
//  Serial.println(distancia_x_array[3]);
//  Serial.println(distancia_x_array[4]);
  return (int)(distancia_x_array[0]+distancia_x_array[1]+distancia_x_array[2]+distancia_x_array[3]+distancia_x_array[4]+distancia_x_array[5]+distancia_x_array[6]+distancia_x_array[7]+distancia_x_array[8]+distancia_x_array[9])/10; 
}
int Medir_distancia_y(){
  int distancia_y_array[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
  int i = 0;
  for(i = 1; i<=10; i++)
  {
    //Para el ultrasonico 1 "y"
    digitalWrite(Trigger_2,LOW); //Para darle estabilización al sensor
    delayMicroseconds(5); //Tiempo de 5 micro segundos
    digitalWrite(Trigger_2, HIGH); //Enviamos el pulso ultrasónico para activar el sensor
    delayMicroseconds(10); //Con una duracion de 10 micro segundos
    tiempo_y = pulseIn(Echo_2, HIGH); //Función para medir la longitud del pulso entrante, mide el tiempo transcurrido de ida y vuelta
    distancia_y = int(0.017*tiempo_y); //Fórmula para calcular la distancia obteniendo un valor entero
    if(distancia_y>35 && distancia_y<1)
    {
      i--;
    }
    else
    {
      distancia_y_array[i-1] = (int) distancia_y;
    }
  }
  
  return (int)(distancia_y_array[0]+distancia_y_array[1]+distancia_y_array[2]+distancia_y_array[3]+distancia_y_array[4]+distancia_y_array[5]+distancia_y_array[6]+distancia_y_array[7]+distancia_y_array[8]+distancia_y_array[9])/10; 
}
int scan()
{
  VL53L0X_RangingMeasurementData_t measure;
    
  //Serial.print("Leyendo sensor... ");
  lox.rangingTest(&measure, false); // si se pasa true como parametro, muestra por puerto serie datos de debug
 
  if (measure.RangeStatus != 4)
  {
    //Serial.print("Distancia (mm): ");
   //Serial.println(measure.RangeMilliMeter);
  } 
  else
  {
    Serial.println("  Fuera de rango ");
  }
    
  delay(100);
  return measure.RangeMilliMeter/10;
}

//----------------------------------------------
//-----------------Step Motor-------------------
//----------------------------------------------
void rotate_clockwise()  {
  int index = 0;
  while(index<500)  {
  for (int i = 0; i < 4; i++)
    {
      digitalWrite(StepMtr_ctr1, paso[i][0]);
      digitalWrite(StepMtr_ctr2, paso[i][1]);
      digitalWrite(StepMtr_ctr3, paso[i][2]);
      digitalWrite(StepMtr_ctr4, paso[i][3]);
      index++;
      delayMicroseconds(2250);
    }
  }
 }
void rotate_counterclockwise()  {
    int index = 0;
    while(index<500)  {
    for (int i = 3; i >= 0; i--)
    {
      digitalWrite(StepMtr_ctr1, paso[i][0]);
      digitalWrite(StepMtr_ctr2, paso[i][1]);
      digitalWrite(StepMtr_ctr3, paso[i][2]);
      digitalWrite(StepMtr_ctr4, paso[i][3]);
      index++;
      delayMicroseconds(2250);
    }
    }
 }

//----------------------------------------------
//------Motors Reference Instrumentation--------
//----------------------------------------------
//int motor_x(int ref_x)
//{
//    int i = Medir_distancia_x();
//     
//    if(i>X_MIN_LIM && i>ref_x)
//    {  //Move in
//      motor_x_near();
//    }
//    else if(i<X_MAX_LIM && i<ref_x)
//    {  //Move out
//      motor_x_far();
//    }
//    else 
//    {
//      stop_motor_x();
//      return -1;
//    }
//    return i;
//}
//int motor_y(int ref_y)
//{
//    int j = Medir_distancia_y();
//     
//    if(j>Y_MIN_LIM && j>ref_y)
//    {  //Move in
//      motor_y_near();
//    }
//    else if(j<Y_MAX_LIM && j<ref_y)
//    {  //Move out
//      motor_y_far();
//    }
//    else 
//    {
//      stop_motor_y();
//      return -1;
//    }
//    return j;
//}

//----------------------------------------------
//--------------Motors Movement-----------------
//----------------------------------------------
 void motor_x_near(int freq_x)
 {
    analogWrite(Mtr_ctr1_x,freq_x);
    analogWrite(Mtr_ctr2_x,LOW);
 }
 void motor_x_far(int freq_x)
 {
    analogWrite(Mtr_ctr1_x,LOW);
    analogWrite(Mtr_ctr2_x,freq_x);
 }
 void stop_motor_x()
{
    analogWrite(Mtr_ctr1_x,LOW);
    analogWrite(Mtr_ctr2_x,LOW);  
}
  void motor_y_near(int freq_y)
 {
    analogWrite(Mtr_ctr1_y,LOW);
    analogWrite(Mtr_ctr2_y,freq_y);
 }
 void motor_y_far(int freq_y)
 {
    analogWrite(Mtr_ctr1_y,freq_y);
    analogWrite(Mtr_ctr2_y,LOW);
 }
void stop_motor_y()
{
    analogWrite(Mtr_ctr1_y,LOW);
    analogWrite(Mtr_ctr2_y,LOW);  
}

