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
#define equipe 1
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


  smartDelay(5000);//Espera 5 segundos para o GPS receber leituras validas
  

  String postagem = "{\"equipe\":"+String(equipe)+",\"bat\":"+String(V_bat,0)+",\"Ace\":["+String(a.acceleration.x,0)+","+String(a.acceleration.y,0)+","+String(a.acceleration.z,0)+"],\"gir\":["+String(g.gyro.x,0)+
  ","+String(g.gyro.y,0)+","+String(g.gyro.z,0)+"],\"Temp\":"+String(bmp.readTemperature(),0)+",\"pressao\":"+String((bmp.readPressure()/101325,1))+",\"payload\":"+String(intensidade(magx, magy, magz),0)+"}";

  String save = "{\"equipe\":"+String(equipe)+",\"GPS\":["+String(gps.location.lat(),6)+","+String(gps.location.lng(),6)+","+String(gps.altitude.meters())
  +"],\"Time\":["+String(gps.time.hour())+","+String(gps.time.minute())+"],"+"\"Baterias\" : [["+String(V_b1)+", "+String(T_b1)+"],["+String(V_b2)+
  ","+String(T_b2)+"]],\"Ace\":["+String(a.acceleration.x)+","+String(a.acceleration.y)+","+String(a.acceleration.z)+"],\"Gir\":["+String(g.gyro.x)+
  ","+String(g.gyro.y)+","+String(g.gyro.z)+"],\"Temp\":"+String(bmp.readTemperature())+",\"PA\":"+String(bmp.readPressure())+",\"mag\":["
  +String(magx/escala_gaus)+","+String(magy/escala_gaus)+","+String(magz/escala_gaus)+"],\"Payload\":"+String(intensidade(magx, magy, magz))+"}";


  File file = SD.open("/Payload"+String(gps.date.day())+String(gps.date.month())+String(gps.date.year()+".txt"), FILE_APPEND);
  file.println(save);
  file.close();

  File esperimento = SD.open("/Bateria com isolamento termico.txt", FILE_APPEND);
  esperimento.println(String(gps.time.hour())+":"+String(gps.time.minute())+"\t"+String(V_bat));
  esperimento.close();


  envia_payload(postagem);

  
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