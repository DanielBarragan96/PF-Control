
#include "Adafruit_VL53L0X.h"

//----------------------------------------------
//--------------------PINS----------------------
//----------------------------------------------
#define Mtr_ctr1_x   5    
#define Mtr_ctr2_x   6    
#define Mtr_ctr1_y   10   
#define Mtr_ctr2_y   11    
#define Echo_1       2    
#define Trigger_1    3   
#define Echo_2       7    
#define Trigger_2    8    
#define StepMtr_ctr1 4    
#define StepMtr_ctr2 9   
#define StepMtr_ctr3 12   
#define StepMtr_ctr4 13
#define SHT_LOX1     16       
#define SHT_LOX2     15

//----------------------------------------------
//-----------------Constants--------------------
//----------------------------------------------
#define X_MIN_LIM 5
#define X_MAX_LIM 30
#define Y_MIN_LIM 10
#define Y_MAX_LIM 25
#define MIN_CICLE_LIM 130
#define MAX_CICLE_LIM 255
#define ROTATE_LENGTH 500
#define REF_SIZE 5
#define T_STEP    0.0005  //T usada para el PID
#define PWM1 6
#define PWM1 5
#define LOX1_ADDRESS 0x30
#define LOX2_ADDRESS 0x31

//----------------------------------------------
//-------------GANANCIAS PID--------------------
//----------------------------------------------
#define KP        3    //valor de KP
#define KI        0.001   //valor de KI
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
void matlabSerial();
void rotate_clockwise();
void rotate_counterclockwise();
void read_dual_sensors();
int scan_y_y();
void initVL53L0X();
void setID();
void motor_x_near(int freq_x);
void motor_x_far(int freq_x);
void motor_y_near(int freq_y);
void motor_y_far(int freq_y);
void stop_motor_x();
void stop_motor_y();

//----------------------------------------------
//--------------Global Variables----------------
//----------------------------------------------
int ref_x[REF_SIZE] = {-1, -1, -1, -1, -1};
int ref_y[REF_SIZE] = {-1, -1, -1, -1, -1};
int ref_counter = 0;
bool ref_x_pos = false;
bool ref_y_pos = false;
Adafruit_VL53L0X lox1 = Adafruit_VL53L0X();
Adafruit_VL53L0X lox2 = Adafruit_VL53L0X();

//----------------------------------------------
//-----------------Variables--------------------
//----------------------------------------------
long distancia_x;        long distancia_y;
//int counter =                     0.00;
//Errores
float ek_x =                       0.0;    //Error actual
float ek_1_x =                     0.0;    //Error anterior
//Entradas
double Uk_x =                       0.0;    //Entrada actual
double Uk_1_x =                     0.0;    //Entrada pasada
double Uk_calc_x =                  0.0;    //Entrada actual
//Errores
float ek_y =                       0.0;    //Error actual
float ek_1_y =                     0.0;    //Error anterior
//Entradas
double Uk_y =                       0.0;    //Entrada actual
double Uk_1_y =                     0.0;    //Entrada pasada
double Uk_calc_y =                  0.0;    //Entrada actual


//----------------------------------------------
//---------------Interrupción-------------------
//----------------------------------------------
void PID_x() {
    //Variables del PID
    double UP_x =                       0.0;
    double UI_x =                       0.0;
    double UD_x =                       0.0;
    //  counter++;
    //  if(counter >= SIN_SIZE)
    //  {
    //    counter = 1;
    //  }

    //ref = SIN_FUNC[counter];

    ek_x = ref_x[ref_counter] - distancia_x;                      //Error actual

    if(ek_x==0)
    {
      stop_motor_x();
      ref_x_pos = true;
      return;
    }
    //PID
    UP_x = KP*ek_x;
    UI_x = (Uk_1_x + KI*T_STEP*ek_x);
    UD_x = (KD*(ek_x - ek_1_x)/T_STEP);
    Uk_x = UP_x + UI_x + UD_x;//Entrada necesaria
    Uk_1_x = UI_x;                                            //Actualizar valor de la entrada
    Uk_calc_x = Uk_x;
         
    int cicle = (MAX_CICLE_LIM*abs(Uk_x)/8.5);
    
    if(MAX_CICLE_LIM<cicle) cicle = MAX_CICLE_LIM;
    if(MIN_CICLE_LIM>cicle) cicle = MIN_CICLE_LIM;

    if(ek_x>0)
    {
      motor_x_far(cicle);
    }
    else if(ek_x<0)
    {
      motor_x_near(cicle);
    }
    Serial.print(F("X:  "));
    Serial.print(distancia_x);
    Serial.print(F(" - Uk: "));
    Serial.print(Uk_x);
    Serial.print(F(" - cicles: "));
    Serial.println(cicle);
}
void PID_y() {
    //Variables del PID
    double UP_y =                       0.0;
    double UI_y =                       0.0;
    double UD_y =                       0.0;
    //  counter++;
    //  if(counter >= SIN_SIZE)
    //  {
    //    counter = 1;
    //  }

    //ref = SIN_FUNC[counter];

    ek_y = ref_y[ref_counter] - distancia_y;                      //Error actual

    if(ek_y==0)
    {
      stop_motor_y();
      ref_y_pos = true;
      return;
    }
    //PID
    UP_y = KP*ek_y;
    UI_y = (Uk_1_y + KI*T_STEP*ek_y);
    UD_y = (KD*(ek_y - ek_1_y)/T_STEP);
    Uk_y = UP_y + UI_y + UD_y;//Entrada necesaria
    Uk_1_y = UI_y;                                            //Actualizar valor de la entrada
    Uk_calc_y = Uk_y;
         
    int cicle = (MAX_CICLE_LIM*abs(Uk_y)/8.5);
    
    if(MAX_CICLE_LIM<cicle) cicle = MAX_CICLE_LIM;
    if(MIN_CICLE_LIM>cicle) cicle = MIN_CICLE_LIM;

    if(ek_y>0)
    {
      motor_y_far(cicle);
    }
    else if(ek_y<0)
    {
      motor_y_near(cicle);
    }
    Serial.print(F("Y:  "));
    Serial.print(distancia_y);
    Serial.print(F(" - Uk: "));
    Serial.print(Uk_y);
    Serial.print(F(" - cicles: "));
    Serial.println(cicle);
}

//----------------------------------------------
//--------------Initialization------------------
//----------------------------------------------
void setup() {
  //asignacion de pines 
  //imprimir en puerto serial
  Serial.begin(9600);

  //init distance sensors
  while (! Serial) { delay(1); }
  pinMode(SHT_LOX1, OUTPUT);
  pinMode(SHT_LOX2, OUTPUT);
  Serial.println(F("Shutdown pins inited..."));
  digitalWrite(SHT_LOX1, LOW);
  digitalWrite(SHT_LOX2, LOW);
  Serial.println(F("Both in reset mode...(pins are low)"));
  Serial.println(F("Starting..."));
  setID();
    
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
  
  matlabSerial();
}

//----------------------------------------------
//-------------------Loop-----------------------
//----------------------------------------------
void loop() {
  





//  while(ref_counter==-1){delay(1000);};
//  
//  read_dual_sensors();
//  if(!ref_x_pos && ref_x[ref_counter]!=-1)
//  {
//    PID_x();  
//  }
//  if(!ref_y_pos && ref_y[ref_counter]!=-1)
//  {
//    PID_y(); 
//  }
//  if(ref_x_pos && ref_y_pos)
//  {
//    if(ref_counter==0)//punto de inicio
//    {
//      //rotate_clockwise();
//    }
//    ref_counter++;
//    if(ref_counter==REF_SIZE)//punto final
//    {
//      rotate_counterclockwise();
//      ref_counter = -1;
//    }
//    else 
//    {
//      if(ref_x[ref_counter-1]!=ref_x[ref_counter])
//      {
//        ref_x_pos = false;  
//      }
//      if(ref_y[ref_counter-1]!=ref_y[ref_counter])
//      {
//        ref_y_pos = false;  
//      }
//    }
//  }
//  delay(400);
}

//----------------------------------------------
//--------------------Matlab--------------------
//----------------------------------------------
void matlabSerial()
{
  while(Serial.available() == 0);
  int vector_size = Serial.read();
  if(REF_SIZE>vector_size)
  {
    Serial.println(F("Cantidad de referencias mayor que REF_SIZE"));
    while(1){};
  }
  for(int i = 0; i<vector_size; i++)
  {
    ref_x[i] = Serial.read();
  }
  for(int j = 0; j<vector_size; j++)
  {
    ref_y[j] = Serial.read();
  }
}

//----------------------------------------------
//------------Distance Measurements-------------
//----------------------------------------------
void read_dual_sensors() {
  VL53L0X_RangingMeasurementData_t measure1;
  VL53L0X_RangingMeasurementData_t measure2;

  lox1.rangingTest(&measure1, false); // pass in 'true' to get debug data printout!
  lox2.rangingTest(&measure2, false); // pass in 'true' to get debug data printout!

  // print sensor one reading
//  Serial.print(F("1: "));
  if(measure1.RangeStatus != 4) {     // if not out of range
    distancia_x = (int) measure1.RangeMilliMeter/10;
//    Serial.print(distancia_x);
  } else {
    Serial.println(F("1:   Out of range"));
  }
  
  Serial.print(" ");

  // print sensor two reading
//  Serial.print(F("2: "));
  if(measure2.RangeStatus != 4) {
    distancia_y = (int) measure2.RangeMilliMeter/10;
//    Serial.print(distancia_y);
  } else {
    Serial.print(F("2: Out of range"));
  }
//  Serial.println();
}

//----------------------------------------------
//-----------------Step Motor-------------------
//----------------------------------------------
void rotate_clockwise()  {
  int index = 0;
  stop_motor_x();
  stop_motor_y();
  while(index<ROTATE_LENGTH)  {
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
    stop_motor_x();
    stop_motor_y();
    while(index<ROTATE_LENGTH)  {
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
//--------------Motors Movement-----------------
//----------------------------------------------
 void motor_x_near(int freq_x)
 {
  if(X_MIN_LIM<distancia_x)
  {
    analogWrite(Mtr_ctr1_x,freq_x);
    analogWrite(Mtr_ctr2_x,LOW); 
  }
  else
  {
    stop_motor_x();
  }
 }
 void motor_x_far(int freq_x)
 {
  if(X_MAX_LIM>distancia_x)
  {
    analogWrite(Mtr_ctr1_x,LOW);
    analogWrite(Mtr_ctr2_x,freq_x);
  }
  else
  {
    stop_motor_x();
  }
 }
 void stop_motor_x()
{
    analogWrite(Mtr_ctr1_x,LOW);
    analogWrite(Mtr_ctr2_x,LOW);  
}
  void motor_y_near(int freq_y)
 {
  if(Y_MIN_LIM<distancia_y)
  {
    analogWrite(Mtr_ctr1_y,LOW);
    analogWrite(Mtr_ctr2_y,freq_y);
  }
  else
  {
    stop_motor_y();
  }
 }
 void motor_y_far(int freq_y)
 {
  if(Y_MAX_LIM>distancia_y)
  {
    analogWrite(Mtr_ctr1_y,freq_y);
    analogWrite(Mtr_ctr2_y,LOW);
  }
  else
  {
    stop_motor_y();
  }
 }
void stop_motor_y()
{
    analogWrite(Mtr_ctr1_y,LOW);
    analogWrite(Mtr_ctr2_y,LOW);  
}

//----------------------------------------------
//----------VL53L0X Initialization--------------
//----------------------------------------------
void initVL53L0X()
{
  // wait until serial port opens for native USB devices
  while (! Serial) { delay(1); }

  pinMode(SHT_LOX1, OUTPUT);
  pinMode(SHT_LOX2, OUTPUT);

  Serial.println(F("Shutdown pins inited..."));

  digitalWrite(SHT_LOX1, LOW);
  digitalWrite(SHT_LOX2, LOW);

  Serial.println(F("Both in reset mode...(pins are low)"));
  
  
  Serial.println(F("Starting..."));
}
void setID() 
{
  // all reset
  digitalWrite(SHT_LOX1, LOW);    
  digitalWrite(SHT_LOX2, LOW);
  delay(10);
  // all unreset
  digitalWrite(SHT_LOX1, HIGH);
  digitalWrite(SHT_LOX2, HIGH);
  delay(10);

  // activating LOX1 and reseting LOX2
  digitalWrite(SHT_LOX1, HIGH);
  digitalWrite(SHT_LOX2, LOW);

  // initing LOX1
  if(!lox1.begin(LOX1_ADDRESS)) {
    Serial.println(F("Failed to boot first VL53L0X"));
    while(1);
  }
  delay(10);

  // activating LOX2
  digitalWrite(SHT_LOX2, HIGH);
  delay(10);

  //initing LOX2
  if(!lox2.begin(LOX2_ADDRESS)) {
    Serial.println(F("Failed to boot second VL53L0X"));
    while(1);
  }
}

