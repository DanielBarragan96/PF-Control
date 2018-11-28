
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
  motor_y_near();
  motor_x_near();
}

//----------------------------------------------
//-------------------Loop-----------------------
//----------------------------------------------
void loop() {

    int lectura, cm;
 
  lectura = analogRead(ir_sensor0); // lectura del sensor 0
  cm = pow(3027.4 / lectura, 1.2134); // conversión a centímetros
  Serial.print("Sensor 0: ");
  Serial.println(cm); // lectura del sensor 0
  delay(500); // tiempo de espera
  

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
  int distancia_x_array[] = {0, 0, 0, 0, 0};
  for(int i = 1; i<=5; i++)
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
  return (int)(distancia_x_array[0]+distancia_x_array[1]+distancia_x_array[2]+distancia_x_array[3]+distancia_x_array[4])/5; 
}
int Medir_distancia_y(){
  int distancia_y_array[5] = {0, 0, 0, 0, 0};
  int i = 0;
  for(i = 1; i<=5; i++)
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
//  Serial.println(distancia_y_array[0]);
//  Serial.println(distancia_y_array[1]);
//  Serial.println(distancia_y_array[2]);
//  Serial.println(distancia_y_array[3]);
//  Serial.println(distancia_y_array[4]);
  return (int)(distancia_y_array[0]+distancia_y_array[1]+distancia_y_array[2]+distancia_y_array[3]+distancia_y_array[4])/5; 
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
    digitalWrite(Mtr_ctr1_y,LOW);
    digitalWrite(Mtr_ctr2_y,HIGH);
 }
 void motor_y_far()
 {
    digitalWrite(Mtr_ctr1_y,HIGH);
    digitalWrite(Mtr_ctr2_y,LOW);
 }
void stop_motor_y()
{
    digitalWrite(Mtr_ctr1_y,LOW);
    digitalWrite(Mtr_ctr2_y,LOW);  
}
