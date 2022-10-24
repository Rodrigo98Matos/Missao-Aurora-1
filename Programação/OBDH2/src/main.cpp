/*
 * OBDH_Aurora_1
 * main.cpp
 * Created: 12/02/2022 22:25:26
 * Author : Rodrigo Matos
 */ 
#include <Arduino.h>
#include "ADC.h"
#include "TTeC.h"
#include <TinyGPSPlus.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include "FS.h"
#include "SD.h"
#include "SPI.h"
#include "MechaQMC5883.h" //Biblioteca do Sensor QMC5883 - Modulo GY-273


#define tempo 240000
#define equipe 24
#define escala_gaus 8

void smartDelay(unsigned long ms); //prototipo da função 
float intensidade(int x, int y, int z);

static const int RXPin = 16, TXPin = 17; //pinos para o GPS
static const uint32_t GPSBaud = 9600;

TinyGPSPlus gps; // Declara gps como um objeto TinyGPSPlus
Adafruit_MPU6050 mpu;
Adafruit_BMP280 bmp;
MechaQMC5883 bussola; //Criacao do objeto para o sensor HMC5883L


void setup() {
  Wire.begin();
  Serial.begin(9600);
  SD.begin(5);
  Serial1.begin(GPSBaud,SERIAL_8N1,RXPin, TXPin);
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  bussola.init(); //Inicializando o Sensor HMC5883L
  bmp.begin(0x76);
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */
}

void loop() {
  int inicio=millis();
  int magx = 0, magy = 0, magz = 0;
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);//lê os dados do sensor inercial
  bussola.read(&magx,&magy,&magz); //Obter o valor dos eixos X, Y e Z do Sensor HMC5883L
  double V_b1 = Vin_div_T(ADC_UNIT_2,27,2000,6800);    //Tensão em Volts da bateria 1
  double T_b1 = ntc_10k(ADC_UNIT_2,12);                //Temperatura em °C  da bateria 1
  double V_b2 = Vin_b2(ADC_UNIT_1,32, V_b1,5100,3300); //Tensão em Volts da bateria 2
  double T_b2 = ntc_10k(ADC_UNIT_2,14);                //Temperatura em °C  da bateria 2
  double V_bat =  porcento(27);    //porcentagem das baterias, com o Vin maximo de 8.2V e um divisor de tensão com R1 = 5100 e R2 = 3300, o Vout maxico será de 3.3V, tornando os calculos mais simples 

  smartDelay(2000);//Espera 2 segundos para o GPS receber leituras validas
  
  String save = "{\"Ace\": {\"X\":"+String(a.acceleration.x)+",\"Y\":"+String(a.acceleration.y)+",\"Z\":"+String(a.acceleration.z)+"},\"Batteries\": {\"B1\":{\"V\":"+String(V_b1)+",\"T\":"+String(T_b1)+"},\"B2\":{\"V\":"
  +String(V_b2)+",\"T\":"+String(T_b2)+"}},\"GPS\": {\"LAT\":"+String(gps.location.lat(),6)+",\"LNG\":"+String(gps.location.lng(),6)+",\"ALT\":"+String(gps.altitude.meters())+"},\"Gir\": {\"X\":"
  +String(g.gyro.x)+",\"Y\":"+String(g.gyro.y)+",\"Z\":"+String(a.acceleration.z)+"},\"PA\": "+String(bmp.readPressure())+",\"Payload\": "+String(intensidade(magx, magy, magz))+",\"Temp\": "
  +String(bmp.readTemperature())+",\"Time\": {\"DAY\":"+String(gps.date.day())+",\"MOU\":"+String(gps.date.month())+",\"YEA\":"+String(gps.date.year())+",\"HOU\":"+String(gps.time.hour())+",\"MIN\":"+String(gps.time.minute())+",\"SEC\":"+String(gps.time.second())+"},\"equipe\": "+String(equipe)+",\"Mag\": {\"X\":"
  +String(magx/escala_gaus)+",\"Y\":"+String(magy/escala_gaus)+",\"Z\":"+String(magz/escala_gaus)+"}}";

  File file = SD.open("/Payload"+String(gps.date.month())+String(gps.date.year())+".txt", FILE_APPEND);
  file.println(save);
  file.close();

  envia_payload(save);

  Serial.println(save);
  
  while ((millis()-inicio)!=tempo);
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

float intensidade(int x, int y, int z){
  x /= 8;
  y /= 8;
  z /= 8;
  return  sqrt (pow (x, 2) + pow (y, 2) + pow (z, 2));
}