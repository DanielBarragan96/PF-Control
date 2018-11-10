// Definimos los pines donde tenemos conectadas las bobinas
#define IN1  4
#define IN2  5
#define IN3  6
#define IN4  7
#define INTERRUPT_PIN_WISE     2
#define INTERRUPT_PIN_COUNTER  3
 
// Secuencia de pasos (par máximo)
int paso [4][4] =
{
  {1, 0, 0, 0},
  {0, 1, 0, 0},
  {0, 0, 1, 0},
  {0, 0, 0, 1}
};
 
void setup()
{
  // Todos los pines en modo salida
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  pinMode(INTERRUPT_PIN_WISE, INPUT_PULLUP );
  attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN_WISE), rotate_clockwise, HIGH   );

  pinMode(INTERRUPT_PIN_COUNTER, INPUT_PULLUP );
  attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN_COUNTER), rotate_counterclockwise, HIGH   );
}
 
 void rotate_clockwise()  {
  while(1)  {
  for (int i = 0; i < 4; i++)
    {
      digitalWrite(IN1, paso[i][0]);
      digitalWrite(IN2, paso[i][1]);
      digitalWrite(IN3, paso[i][2]);
      digitalWrite(IN4, paso[i][3]);
      if(1 == digitalRead(INTERRUPT_PIN_WISE)) {
        return 0;
      }
      delayMicroseconds(2250);
    }
  }
 }

  void rotate_counterclockwise()  {
    while(1)  {
    for (int i = 3; i >= 0; i--)
    {
      digitalWrite(IN1, paso[i][0]);
      digitalWrite(IN2, paso[i][1]);
      digitalWrite(IN3, paso[i][2]);
      digitalWrite(IN4, paso[i][3]);
      if(1 == digitalRead(INTERRUPT_PIN_COUNTER)) {
        return 0;
      }
      delayMicroseconds(2250);
    }
    }
 }
 
void loop()
{ 

}
