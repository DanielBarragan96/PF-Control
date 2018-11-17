//----------------------------------------------
//-----------------Constants--------------------
//----------------------------------------------
#define X_MIN_LIM 5
#define X_MAX_LIM 30
#define Y_MIN_LIM 10
#define Y_MAX_LIM 25

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
int motor_x_near(int ref);
int motor_x_far(int ref);
void motor_y_near();
void motor_y_far();
void stop_motor_x();
void stop_motor_y();
int motor_x(int ref);
int motor_y(int ref_y);

//----------------------------------------------
//--------------Global Variables----------------
//----------------------------------------------
int ref = 15;

//----------------------------------------------
//-----------------Variables--------------------
//----------------------------------------------
int Mtr_ctr1_x   = 2;    int Mtr_ctr2_x   = 3;    int Mtr_ctr1_y   = 4; 
int Mtr_ctr2_y   = 5;    int Echo_1       = 6;    int Trigger_1    = 7;   
int Echo_2       = 8;    int Trigger_2    = 9;    int StepMtr_ctr1 = 10; 
int StepMtr_ctr2 = 11;   int StepMtr_ctr3 = 12;   int StepMtr_ctr4 = 13;
long distancia_x;        long tiempo_x;           long distancia_y; 
long tiempo_y;

//----------------------------------------------
//--------------Initialization------------------
//----------------------------------------------
void setup() {
  //asignacion de pines 
  //imprimir en puerto serial
  Serial.begin(9600);
  
  // Para el motor 1
  pinMode(Mtr_ctr1_x, OUTPUT);
  pinMode(Mtr_ctr1_y, OUTPUT);
  
  // Para el motor 2
  pinMode(Mtr_ctr2_x, OUTPUT);
  pinMode(Mtr_ctr2_y, OUTPUT);

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
}

//----------------------------------------------
//-------------------Loop-----------------------
//----------------------------------------------
void loop() {
  int motor_status = motor_x(ref);
  Serial.println("Distancia "); //Imprimimos "Distancia" sobre el Monitor Serial
  Serial.println(motor_status); //Mostramos el Valor de la distancia real sobre el Monitor Serial  
  if(motor_status==-1 && ref==15)
  {
    ref = 25;
    rotate_clockwise();
  }
  else if(motor_status==-1 && ref==25)
  {
    ref = 15;
    rotate_counterclockwise();
  }
  delay(1000); //Cada que Tiempo se imprimira el valor de la distancia
}

//----------------------------------------------
//------------Distance Measurements-------------
//----------------------------------------------
int Medir_distancia_x(){
  //Para el ultrasonico 1 "x"
  digitalWrite(Trigger_1,LOW); //Para darle estabilización al sensor
  delayMicroseconds(5); //Tiempo de 5 micro segundos
  digitalWrite(Trigger_1, HIGH); //Enviamos el pulso ultrasónico para activar el sensor
  delayMicroseconds(10); //Con una duracion de 10 micro segundos
  tiempo_x = pulseIn(Echo_1, HIGH); //Función para medir la longitud del pulso entrante, mide el tiempo transcurrido de ida y vuelta
  distancia_x = int(0.017*tiempo_x); //Fórmula para calcular la distancia obteniendo un valor entero
  return distancia_x; 
}
int Medir_distancia_y(){
  //Para el ultrasonico 1 "y"
  digitalWrite(Trigger_2,LOW); //Para darle estabilización al sensor
  delayMicroseconds(5); //Tiempo de 5 micro segundos
  digitalWrite(Trigger_2, HIGH); //Enviamos el pulso ultrasónico para activar el sensor
  delayMicroseconds(10); //Con una duracion de 10 micro segundos
  tiempo_y = pulseIn(Echo_2, HIGH); //Función para medir la longitud del pulso entrante, mide el tiempo transcurrido de ida y vuelta
  distancia_y = int(0.017*tiempo_y); //Fórmula para calcular la distancia obteniendo un valor entero
  return distancia_y; 
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
//-------------Motors Reference-----------------
//----------------------------------------------
int motor_x(int ref_x)
{
    int i = Medir_distancia_x();
     
    if(i>X_MIN_LIM && i>ref_x)
    {  //Move in
      motor_x_near();
    }
    else if(i<X_MAX_LIM && i<ref_x)
    {  //Move out
      motor_x_far();
    }
    else 
    {
      stop_motor_x();
      return -1;
    }
    return i;
}
int motor_y(int ref_y)
{
    int j = Medir_distancia_y();
     
    if(j>Y_MIN_LIM && j>ref_y)
    {  //Move in
      motor_y_near();
    }
    else if(j<Y_MAX_LIM && j<ref_y)
    {  //Move out
      motor_y_far();
    }
    else 
    {
      stop_motor_y();
      return -1;
    }
    return j;
}

//----------------------------------------------
//--------------Motors Movement-----------------
//----------------------------------------------
 void motor_x_near()
 {
    digitalWrite(Mtr_ctr1_x,HIGH);
    digitalWrite(Mtr_ctr2_x,LOW);
 }
 void motor_x_far()
 {
    digitalWrite(Mtr_ctr1_x,LOW);
    digitalWrite(Mtr_ctr2_x,HIGH);
 }
 void stop_motor_x()
{
    digitalWrite(Mtr_ctr1_x,LOW);
    digitalWrite(Mtr_ctr2_x,LOW);  
}
  void motor_y_near()
 {
    digitalWrite(Mtr_ctr1_y,HIGH);
    digitalWrite(Mtr_ctr2_y,LOW);
 }
 void motor_y_far()
 {
    digitalWrite(Mtr_ctr1_y,LOW);
    digitalWrite(Mtr_ctr2_y,HIGH);
 }
void stop_motor_y()
{
    digitalWrite(Mtr_ctr1_y,LOW);
    digitalWrite(Mtr_ctr2_y,LOW);  
}

