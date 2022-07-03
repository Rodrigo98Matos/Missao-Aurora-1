#include <Arduino.h>
#include "BluetoothSerial.h"
#include "FS.h"
#include "SD.h"
#include "SPI.h"
#include <TinyGPSPlus.h>
#include <driver/adc.h>
#include <esp_adc_cal.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_err.h>
#include <esp_log.h>


#define tempo 60000
#define motor 25
#define MENOR_TENSAO 3000


#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

uint32_t tensao(adc_unit_t ADC,unsigned char pin);
double Vin_div_T(adc_unit_t ADC,unsigned char pin, double r1, double r2);

/// prototipos de funções 
uint32_t tensao(adc_unit_t ADC,unsigned char pin);
double Vin_div_T(adc_unit_t ADC,unsigned char pin, double r1, double r2);
void smartDelay(unsigned long ms);
int baixo = 0;
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
  double V_b = Vin_div_T(ADC_UNIT_1,36,200,680);    //Tensão em Volts da bateria

  smartDelay(5000);//Espera 5 segundos para o GPS receber leituras validas

  payload = "["+String(gps.date.day())+"/"+String(gps.date.month())+"/"+String(gps.date.year())+"]"+"["+String(gps.time.hour())+":"+String(gps.time.minute())+"]    "+String(V_b)+"mV";

  File esperimento = SD.open("/Monitor_Bateria_Motor_DC1.txt", FILE_APPEND);
  esperimento.println(payload);
  esperimento.close();

  SerialBT.println(payload);
  Serial.println(payload);
  if(V_b<=MENOR_TENSAO){
    baixo++;
  }else{
    digitalWrite(motor,0);
  }
  if (baixo>=MENOR_TENSAO){
    digitalWrite(motor,1);
    while(1);
  }
  while ((millis()-inicio)!=tempo);
}

uint32_t tensao(adc_unit_t ADC,unsigned char pin){      // ADC_UNIT_1   ou   ADC_UNIT_2
  esp_adc_cal_characteristics_t adc_cal;//Estrutura que contem as informacoes para calibracao

  esp_adc_cal_characterize(ADC, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12, 1100, &adc_cal);//Inicializa a estrutura de calibracao

  uint32_t voltage = 0;
		for (int i = 0; i < 100; i++)
		{
			voltage += analogRead(pin);//Obtem o valor RAW do ADC
			ets_delay_us(30);
		}
		voltage /= 100;


		voltage = esp_adc_cal_raw_to_voltage(voltage, &adc_cal);//Converte e calibra o valor lido (RAW) para mV
  return voltage;
}
double Vin_div_T(adc_unit_t ADC,unsigned char pin, double r1, double r2){
  uint32_t tens = tensao(ADC,pin);
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

