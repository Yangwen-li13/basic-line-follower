// Kodu yüklemeden önce indirmemiz gereken kütüphane
#include <QTRSensors.h>
// Öncelikle motor sürücünün ve QTR Sensörünün pinlerini belirlememiz lazım
#define MotorR1 11
#define MotorR2 12 
#define MotorRE 13
#define MotorL1 10
#define MotorL2 9
#define MotorLE 8
#define Sensor1 A0
#define Sensor2 A1
#define Sensor3 A2
#define Sensor4 A3
#define Sensor5 A4
#define Sensor6 A5
// Kod üzerinde tanımladığım hız değerleri
#define rightMaxSpeed 114
#define leftMaxSpeed 115
#define rightBaseSpeed 85 
#define leftBaseSpeed 86 
// Buradaki sayılar robotun temel algoritmasında kullandığım hareketini belirleyecek sayılar, tamamen deneme yanılma ile hesaplanabilir.
float Kp = 0.038;
float Kd = 0.76;
// QTR kütüphanesindeki sensörler için gerekli kodlar
QTRSensors qtr;
const uint8_t SensorCount = 6;
uint16_t sensorValues[SensorCount];



void setup() {
  // input ve outputları belirleyip kütüphanede sensörler için gerekili bazı fonksiyonları çağırmamız gerekiyor.
  pinMode(MotorR1, OUTPUT);
  pinMode(MotorR2, OUTPUT);
  pinMode(MotorRE, OUTPUT);
  pinMode(MotorL1, OUTPUT);
  pinMode(MotorL2, OUTPUT);
  pinMode(MotorLE, OUTPUT);
  pinMode(Sensor1, INPUT);
  pinMode(Sensor2, INPUT);
  pinMode(Sensor3, INPUT);
  pinMode(Sensor4, INPUT);
  pinMode(Sensor5, INPUT);
  pinMode(Sensor6, INPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){A0, A1, A2, A3, A4, A5}, SensorCount);
  delay(500);

  digitalWrite(LED_BUILTIN, HIGH); // Arduino'nun ledini kalibrasyon modu sırasında açar. 
  
  int i;
  for (int i = 0; i < 100; i++) { // tercihe bağlı alandır ya robotunuzu hatta yürüterek kalibre edin veya bunu otomatik yapın.
    if ( i  < 25 || i >= 75 ) { sag();}
    else {sol();}
    qtr.calibrate();
    delay(20); }
  bekle(); //motorları durdur
  digitalWrite(LED_BUILTIN, LOW); // LEDin ışığını kalibrasyon bitişinde kapatır.
  delay(5000); // Ana döngüye girmeden önce botu konumlandırmak için 5 saniye bekleyin
  }

int lastError = 0;

void loop() {
  // Eğer izlenecek çizgi beyaz olacaksa alttaki fonksiyon değiştirilmelidir.
  uint16_t position = qtr.readLineBlack(sensorValues); // Her bir sensör çizgideki ışık yansımalarına göre 0'dan 1000'e kadar değer verir,bu fonksiyonda 0 max, 1000 in yansımayı temsil eder.
  delay(250);
  // Robotun temel algoritması:
  int error = position - 3000;
  int motorSpeed = Kp * error + Kd * (error - lastError);
  lastError = error;
  int rightMotorSpeed = rightBaseSpeed + motorSpeed;
  int leftMotorSpeed = leftBaseSpeed - motorSpeed;
  if (rightMotorSpeed > rightMaxSpeed ) rightMotorSpeed = rightMaxSpeed;
  if (leftMotorSpeed > leftMaxSpeed ) leftMotorSpeed = leftMaxSpeed;
  if (rightMotorSpeed < 0) rightMotorSpeed = 0;
  if (leftMotorSpeed < 0) leftMotorSpeed = 0;
  

    digitalWrite(MotorR1, HIGH);
    digitalWrite(MotorR2, LOW);
    analogWrite(MotorRE, rightMotorSpeed);

    digitalWrite(MotorL1, HIGH);
    digitalWrite(MotorL2, LOW);
    analogWrite(MotorLE, leftMotorSpeed);
  
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

void geri(){
  digitalWrite(MotorR1, LOW);
  digitalWrite(MotorR2, HIGH);
  digitalWrite(MotorRE, 255);

  digitalWrite(MotorL1, LOW);
  digitalWrite(MotorL2, HIGH);
  digitalWrite(MotorLE, 255);

}

void ileri() {

  digitalWrite(MotorR1, HIGH);
  digitalWrite(MotorR2, LOW);
  digitalWrite(MotorRE, 255);

  digitalWrite(MotorL1, HIGH);
  digitalWrite(MotorL2, LOW);
  digitalWrite(MotorLE, 255);
}

void bekle() {
    digitalWrite(MotorR1,LOW);
    digitalWrite(MotorR2, LOW);
   

    digitalWrite(MotorL1, LOW);
    digitalWrite(MotorL2, LOW);
}


void sol(){

  digitalWrite(MotorR1, HIGH);
  digitalWrite(MotorR2, LOW);
  digitalWrite(MotorRE, rightBaseSpeed);

  digitalWrite(MotorL1, LOW);
  digitalWrite(MotorL2, HIGH);
  digitalWrite(MotorLE, leftBaseSpeed);

}

void sag(){

  digitalWrite(MotorR1, LOW);
  digitalWrite(MotorR2, HIGH);
  digitalWrite(MotorRE, rightBaseSpeed);

  digitalWrite(MotorL1, HIGH);
  digitalWrite(MotorL2, LOW);
  digitalWrite(MotorLE, leftBaseSpeed);

}