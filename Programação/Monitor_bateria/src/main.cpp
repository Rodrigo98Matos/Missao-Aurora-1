#include <Arduino.h>
#include "BluetoothSerial.h"
#include "FS.h"
#include "SD.h"
#include "SPI.h"
#include <TinyGPSPlus.h>


#define tempo 60000
#define motor 25


#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif
/// prototipos de funções 
double tensao(unsigned char);
double Vin_div_T(unsigned char pin,double r1,double r2);
void smartDelay(unsigned long ms);

BluetoothSerial SerialBT;
TinyGPSPlus gps; // Declara gps como um objeto TinyGPSPlus
static const int RXPin = 16, TXPin = 17; //pinos para o GPS
static const uint32_t GPSBaud = 9600;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  SD.begin(5);
  Serial1.begin(GPSBaud,SERIAL_8N1,RXPin, TXPin);
  SerialBT.begin("Aurora-1"); //Bluetooth device name
  pinMode(motor,OUTPUT);
}

void loop() {
  int inicio=millis();
  String payload;
  double V_b = Vin_div_T(33,200,680);    //Tensão em Volts da bateria

  smartDelay(5000);//Espera 5 segundos para o GPS receber leituras validas

  payload = "["+String(gps.date.day())+"/"+String(gps.date.month())+"/"+String(gps.date.year())+"]"+"["+String(gps.time.hour())+":"+String(gps.time.minute())+"]    "+String(V_b)+"V";

  File esperimento = SD.open("/Monitor_Bateria_Motor_DC.txt", FILE_APPEND);
  esperimento.println(payload);
  esperimento.close();

  SerialBT.println(payload);
  Serial.println(payload);
  if(V_b<=3.00){
    digitalWrite(motor,1);
    while(1);
  }else{
    digitalWrite(motor,0);
  }
  while ((millis()-inicio)!=tempo);
}

double tensao(unsigned char pin){      //retorna a tensão em Volts lido na porta analógica 
  int leitura = analogRead(pin);
  return (3.300*leitura)/4095.000;
}

double Vin_div_T(unsigned char pin, double r1, double r2){
  double tens = tensao(pin);
  double res = (r2/(r1+r2));
  return (tens/res); //tensão/(R2/(R1+R2)) retorna a tensão de entrada do divisor de tensão
}

void smartDelay(unsigned long ms)
{
  unsigned long start = millis();
  do 
  {
    while (Serial1.available())
      gps.encode(Serial1.read());
  } while (millis() - start < ms);
}

