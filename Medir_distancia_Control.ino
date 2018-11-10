long distancia;
long tiempo;
void setup(){
  Serial.begin(9600);
  pinMode(9, OUTPUT); //Configuracion del pin 9 como salida: Trigger - Disparo
  pinMode(8, INPUT); //Configuracion del pin 8 como entrada: Recibira el Echo
}

void loop(){
  digitalWrite(9,LOW); //Para darle estabilización al sensor
  delayMicroseconds(5); //Tiempo de 5 micro segundos
  digitalWrite(9, HIGH); //Enviamos el pulso ultrasónico para activar el sensor
  delayMicroseconds(10); //Con una duracion de 10 micro segundos
  tiempo=pulseIn(8, HIGH); //Función para medir la longitud del pulso entrante, mide el tiempo transcurrido de ida y vuelta
  distancia= int(0.017*tiempo); //Fórmula para calcular la distancia obteniendo un valor entero
  //Monitorización en centímetros por el monitor serial
  Serial.println("Distancia "); //Imprimimos "Distancia" sobre el Monitor Serial
  Serial.println(distancia); //Mostramos el Valor de la distancia real sobre el Monitor Serial 
  Serial.println(" cm"); //Imprimimos " cm" sobre el Monitor Serial
  delay(1000); //Cada que Tiempo se imprimira el valor de la distancia 
}
