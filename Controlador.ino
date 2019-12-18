// --- Biblioteca Auxiliar ---
#include <dht.h>   //biblioteca do sensor de umidade e temperatura
#include <Wire.h>
#include <HMC5883L.h>
#include <Servo.h> 


// --- Mapeamento de Hardware ---
//Sinaliza a comunicação serial esta ativa
bool status_serial = false;
#define dht_pin 2 //pino de sinal do dht11 ligado no digital 5
//Pin sensor ultrasônico
int trigPin = 12;
int echoPin = 11;
float enviDist =0;
int flag_control = 0;
// Definicoes dos pinos ligados ao sensor de fumaça MQ-2
int pin_a0 = A0;
//Definições dos pino de led
const int pin_ledblue = 10;
//Definicoes dos pinos ligados aos servos motores
#define pinSentido1MotorA 5
#define pinSentido2MotorA 4
#define pinEnableMotorA 9
#define pinSentido1MotorB 7
#define pinSentido2MotorB 6
#define pinEnableMotorB 3



// --- Declaração de Objetos ---
dht   my_dht;   //objeto para o sensor
Servo motorDir; // Motor diretio
Servo motorEsq; // Motor esquerdo


// --- Variáveis Globais ---
int    temperatura = 0x00,   //armazena a temperatura em inteiro
       umidade     = 0x00;   //armazena a umidade em inteiro

//nivel para alarme
int nivel_sensor = 250;

void setup() {
  //Configuração porta serial
  Serial.begin(9600);
 //Configuração dos motores
  pinMode(pinSentido1MotorA, OUTPUT);
  pinMode(pinSentido2MotorA, OUTPUT);
  pinMode(pinSentido1MotorB, OUTPUT);
  pinMode(pinSentido2MotorB, OUTPUT);
  pinMode(pinEnableMotorA, OUTPUT);
  pinMode(pinEnableMotorB, OUTPUT);
  
  //Configuração dos pinos do sensor ultrassonico
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  // Define os pinos de leitura do sensor como entrada fumaça MQ-2
  pinMode(pin_a0, INPUT);
  //Configuração do pino para o led
  pinMode(pin_ledblue, OUTPUT);
  analogWrite(pin_ledblue,255);
   //Define a posição inicial do robo como "parado"
  move(0);
 
}

void loop() {
  static unsigned long ult_tempo = 0;
  if(Serial.available()) {
       int serial = Serial.read() - '0';
       move(serial);
       if(serial == 9){
          analogWrite(pin_ledblue,0);
          ult_tempo = millis();
       }
          
      }
   if((millis() - ult_tempo)>=60000)
   {
      analogWrite(pin_ledblue,255);
   }     
  //Caculando distancia
  float TempPulso, Dist1, Dist2, Dist3, medDist;
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  TempPulso = pulseIn(echoPin, HIGH);
  Dist1 = (TempPulso / 2) / 29.1; //29.1 microssegundos por centimento

  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  TempPulso = pulseIn(echoPin, HIGH);
  Dist2 = (TempPulso / 2) / 29.1; //29.1 microssegundos por centimento

  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  TempPulso = pulseIn(echoPin, HIGH);
  Dist3 = (TempPulso / 2) / 29.1; //29.1 microssegundos por centimento

  medDist = (Dist1 + Dist2 + Dist3)/3;
  medDist = medDist/100;
  

  //+++++++++++++++++++++++++++++++++++++++++++++++++++

  //Leitura dos dados do sensor de umidade e temperatura DHT11
  my_dht.read11(dht_pin);

  temperatura = my_dht.temperature;
  umidade     = my_dht.humidity;

  // Le os dados do pino analogico A0 do sensor MQ-2
  int valor_analogico_mq_2 = analogRead(pin_a0);
  //Envio dos dados
  String envio = "";
  if(medDist<=4){
  envio = medDist; //distancia em metros
  }else{
    envio="--";
    }
  envio += ",";
  envio += valor_analogico_mq_2;//Sensor mq2, fumaça, gases inflamaveis
  envio += ",";
  envio += temperatura;//temperatura
  envio += ",";
  envio += umidade;//umidade

  Serial.println(envio);

  delay(1);

}


void move(int direc) {

switch(direc){
   //PARADO
  case 0:
//    analogWrite(pinEnableMotorB,0);
//    analogWrite(pinEnableMotorA,0);
    digitalWrite(pinSentido1MotorA, LOW);
    digitalWrite(pinSentido2MotorA, LOW); 
    digitalWrite(pinSentido1MotorB, LOW);
    digitalWrite(pinSentido2MotorB, LOW);   
    
    break;
    //frente 
  case 1:
//    analogWrite(pinEnableMotorA,255);
//    analogWrite(pinEnableMotorB,255);
    digitalWrite(pinSentido1MotorA, LOW);
    digitalWrite(pinSentido2MotorA, HIGH);
    digitalWrite(pinSentido1MotorB, LOW);
    digitalWrite(pinSentido2MotorB, HIGH);
    
    break;
     //Direita
  case 2:
//     analogWrite(pinEnableMotorA,100);
//     analogWrite(pinEnableMotorB,0);
     digitalWrite(pinSentido1MotorA, LOW);
     digitalWrite(pinSentido2MotorA, HIGH);  
     digitalWrite(pinSentido1MotorB, LOW);
      digitalWrite(pinSentido2MotorB, LOW);
     break;
     //Esquerda
  case 3:
//     analogWrite(pinEnableMotorB,80);
//     analogWrite(pinEnableMotorA,0);
     digitalWrite(pinSentido1MotorA, LOW);
     digitalWrite(pinSentido2MotorA, LOW);  
     digitalWrite(pinSentido1MotorB, LOW);
     digitalWrite(pinSentido2MotorB, HIGH);
    
    break;
   //Para tras
  case 4:
//     analogWrite(pinEnableMotorA,255);
//     analogWrite(pinEnableMotorB,255);
     digitalWrite(pinSentido1MotorA, HIGH);
     digitalWrite(pinSentido2MotorA, LOW);  
     digitalWrite(pinSentido1MotorB, HIGH);
     digitalWrite(pinSentido2MotorB, LOW); 
    
    break;
      
  }
}
 







