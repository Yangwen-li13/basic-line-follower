// Kodu yüklemeden önce indirmemiz gereken kütüphane
#include <QTRSensors.h>
#include <L298NX2.h>
// Öncelikle motor sürücünün ve QTR Sensörünün pinlerini belirlememiz lazım
const unsigned int EN_A = 10;
const unsigned int IN1_A = 9;
const unsigned int IN2_A = 8;

const unsigned int IN1_B = 7;
const unsigned int IN2_B = 6;
const unsigned int EN_B = 5;


#define Sensor1 14
#define Sensor2 15
#define Sensor3 16
#define Sensor4 17
#define Sensor5 18
#define Sensor6 19

#define DEBUG 0 // Output almak istiyorsan 1 yapabilirsin

// Kod üzerinde tanımladığım hız değerleri
#define rightMaxSpeed 170
#define leftMaxSpeed 170
#define rightBaseSpeed 75 
#define leftBaseSpeed 76
// Buradaki sayılar robotun temel algoritmasında kullandığım hareketini belirleyecek sayılar, tamamen deneme yanılma ile hesaplanabilir. (Note: Kp < Kd) 
float Kp = 0.055;
float Kd = 0.3;
// QTR ve L298N kütüphanesi için gerekli kodlar
QTRSensors qtr;
#define TIMEOUT 2500
const uint8_t SensorCount = 6;
uint16_t sensorValues[SensorCount];

L298NX2 motors(EN_A, IN1_A, IN2_A, EN_B, IN1_B, IN2_B);



void setup() {
  // input ve outputları belirleyip kütüphanede sensörler için gerekili bazı fonksiyonları çağırmamız gerekiyor.

  pinMode(Sensor1, INPUT);
  pinMode(Sensor2, INPUT);
  pinMode(Sensor3, INPUT);
  pinMode(Sensor4, INPUT);
  pinMode(Sensor5, INPUT);
  pinMode(Sensor6, INPUT);

  pinMode(LED_BUILTIN, OUTPUT);
  qtr.setTypeRC();
  qtr.setSensorPins((const uint8_t[]){14, 15, 16, 17, 18, 19}, SensorCount);
  // qtr.setTimeout(TIMEOUT);
  delay(500);

  digitalWrite(LED_BUILTIN, HIGH); // Arduino'nun ledini kalibrasyon modu sırasında açar. 
  
  int i;
  for (int i = 0; i < 100; i++) { // tercihe bağlı alandır ya robotunuzu hatta yürüterek kalibre edin veya bunu otomatik yapın.

    qtr.calibrate();
    delay(20); }
  motors.stop(); //motorları durdur
  digitalWrite(LED_BUILTIN, LOW); // LEDin ışığını kalibrasyon bitişinde kapatır.
  delay(5000); // Ana döngüye girmeden önce botu konumlandırmak için 5 saniye bekleyin
}



int lastError = 0;


void loop() {
  // Eğer izlenecek çizgi beyaz olacaksa alttaki fonksiyon değiştirilmelidir.
  uint16_t position = qtr.readLineWhite(sensorValues); // Her bir sensör çizgideki ışık yansımalarına göre 0'dan 1000'e kadar değer verir,bu fonksiyonda 0 max, 1000 in yansımayı temsil eder.
  // Robotun temel algoritması:

  int error = position - 2500;
  int motorSpeed = Kp * error + Kd * (error - lastError) ;
  lastError = error;

  int rightMotorSpeed = rightBaseSpeed - motorSpeed;
  int leftMotorSpeed = leftBaseSpeed + motorSpeed;

  if (rightMotorSpeed > rightMaxSpeed ) rightMotorSpeed = rightMaxSpeed;
  if (leftMotorSpeed > leftMaxSpeed ) leftMotorSpeed = leftMaxSpeed;
  if (rightMotorSpeed < 0) rightMotorSpeed = 0;
  if (leftMotorSpeed < 0) leftMotorSpeed = 0;
  
  motors.forward();
  motors.setSpeedA(rightMotorSpeed);
  motors.setSpeedB(leftMotorSpeed);



  if (DEBUG) { // if true, generate sensor dats via serial output
    Serial.begin(9600);
    for (int i = 0; i < SensorCount; i++)
  {
      Serial.print(qtr.calibrationOn.minimum[i]);
      Serial.print(' ');
  }
      Serial.println();
    for (int i = 0; i < SensorCount; i++)
  {
      Serial.print(qtr.calibrationOn.maximum[i]);
      Serial.print(' ');
  }
  Serial.println();
  Serial.println();
  Serial.print(position);
  Serial.println();
  Serial.print(rightMotorSpeed);
  Serial.print(' ');
  Serial.print(leftMotorSpeed);
  delay(500);
  }
  
  /*
  char rc;
  while(Serial.available()>0){
  rc = Serial.read(); 
  Serial.println(rc);                 //Motor Sürücü ve Motorları denemek için
  delay(500);
  if(rc == 'u'){ileri();}
  if(rc == 'd'){geri();}
  if(rc == 'l'){sol();}
  if(rc == 'r'){sag();}     } */  

}

