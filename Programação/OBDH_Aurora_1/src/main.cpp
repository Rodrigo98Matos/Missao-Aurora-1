#include <Arduino.h>
#include "ADC.h"
#include "TTeC.h"
#include <TinyGPSPlus.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>



void smartDelay(unsigned long ms); //prototipo da função 

static const int RXPin = 16, TXPin = 17; //pinos para o GPS
static const uint32_t GPSBaud = 9600;


TinyGPSPlus gps; // Declara gps como um objeto TinyGPSPlus
Adafruit_MPU6050 mpu;


void setup() {
  Serial.begin(9600);
  Serial1.begin(GPSBaud,SERIAL_8N1,RXPin, TXPin);

   
  smartDelay(30000); //Espera um minuto para o GPS receber leituras validas
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
}

void loop() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  double V_b1 = Vin_div_T(27,2000,6800);    //Tensão em Volts da bateria 1
  double T_b1 = ntc_10k(12);                //Temperatura em °C  da bateria 1
  double V_b2 = Vin_b2(32, V_b1,5100,3300); //Tensão em Volts da bateria 2
  double T_b2 = ntc_10k(14);                //Temperatura em °C  da bateria 2

  smartDelay(5000);
  

  String postagem = "{\"equipe\":1,\"GPS\":["+String(gps.location.lat(),6)+","+String(gps.location.lng(),6)+","+String(gps.altitude.meters())
  +"],\"Data\":["+String(gps.date.day())+","+String(gps.date.month())+","+String(gps.date.year())+"],\"Time\":["+String(gps.time.hour()-3)+","
  +String(gps.time.minute())+"],"+"\"Bateria1\" : ["+String(V_b1)+", "+String(T_b1)+"], \"Bateria2\" : ["+String(V_b2)+", "+String(T_b2)+
  "],\"Acelerometro\":["+String(a.acceleration.x)+","+String(a.acceleration.y)+","+String(a.acceleration.z)+"],\"Giroscopio\":["+String(g.gyro.x)+
  ","+String(g.gyro.y)+","+String(g.gyro.z)+"],\"Temper\":"+String(temp.temperature)+"}";

  envia_payload(postagem);

  Serial.println(postagem);

  delay(60000);
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