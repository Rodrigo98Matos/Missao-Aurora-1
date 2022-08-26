#include <Arduino.h>
#include "FS.h"
#include "SD.h"
#include "SPI.h"
#include <driver/adc.h>
#include <esp_adc_cal.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_err.h>
#include <esp_log.h>
#include <WiFi.h>
#include <NTPClient.h>
#include <WiFiUdp.h>


#define tempo 60000
#define motor 25
#define MENOR_TENSAO 3000

//#ifndef STASSID
#define STASSID "ALBERTO E ROSELIA" //ID do WiFi
#define STAPSK  "CHAMEX5050"        //Senha do WiFi

WiFiUDP ntpUDP;

NTPClient timeClient(ntpUDP, "br.pool.ntp.org", -3*3600, 60000);


/// prototipos de funções 
uint32_t tensao(adc_unit_t ADC,unsigned char pin);
double Vin_div_T(adc_unit_t ADC,unsigned char pin, double r1, double r2);
int baixo = 0;

void setup() {
  Serial.begin(115200);
  SD.begin(5);
  pinMode(motor,OUTPUT);
  pinMode(2,OUTPUT);

}

void loop() {
  int inicio=millis();
  digitalWrite(2,1);
  String payload;
  double V_b = Vin_div_T(ADC_UNIT_1,36,200,680);    //Tensão em Volts da bateria

  //Liga WiFi
  WiFi.begin(STASSID, STAPSK);
  int cont = 0;
  while (WiFi.status() != WL_CONNECTED && cont<120) {//se não conectar depois de 1 minuto, desiste de conectar
    delay(500);cont++;
    Serial.print(".");
  }

  timeClient.begin();
  timeClient.update();

  String tempontp = String(timeClient.getFormattedTime());
 

  //Desliga WiFi
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
 
  payload = "["+tempontp+"]    "+String(V_b)+"mV";


  File esperimento = SD.open("/Monitor_Bateria_Motor_DC11.txt", FILE_APPEND);
  esperimento.println(payload);
  esperimento.close();


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
  digitalWrite(2,0);
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