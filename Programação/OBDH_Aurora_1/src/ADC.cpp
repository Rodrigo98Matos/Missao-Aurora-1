/*
 * ADC.c
 *
 * Created: 12/03/2022 14:35:26
 * Author : Rodrigo Matos
 */  
#include "ADC.h"

float tensao(adc_unit_t ADC,uint8_t pin){      // ADC_UNIT_1   ou   ADC_UNIT_2
  esp_adc_cal_characteristics_t adc_cal;//Estrutura que contem as informacoes para calibracao

  esp_adc_cal_characterize(ADC, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12, 1100, &adc_cal);//Inicializa a estrutura de calibracao

  int voltage = 0;
		for (int i = 0; i < 100; i++)
		{
			voltage += analogRead(pin);//Obtem o valor RAW do ADC
			ets_delay_us(30);
		}
		voltage /= 100;


		voltage = esp_adc_cal_raw_to_voltage(voltage, &adc_cal);//Converte e calibra o valor lido (RAW) para mV
  return (float)voltage/1000;
}
double Vin_div_T(adc_unit_t ADC,unsigned char pin, double r1, double r2){
  uint32_t tens = tensao(ADC,pin);
  double res = (r2/(r1+r2));
  return (tens/res); //tensÃ£o/(R2/(R1+R2)) retorna a tensÃ£o de entrada do divisor de tensÃ£o
}


/*double tensao(unsigned char pin){      //retorna a tensão em Volts lido na porta analógica 
  int leitura = analogRead(pin);
  return (3.3*leitura)/4095.00;
}*/
double porcento(uint8_t pin){      //retorna a tensão em Volts lido na porta analógica 
  int leitura = analogRead(pin);
  return (100*leitura)/4095.00;
}
double R1(adc_unit_t ADC,uint8_t pin){       //Em um divisor de tensão com 10000 Ohm no resistor superior, retorna o valor ohmico
  double Vadc = tensao(ADC,pin);
  return ((10000*3.3)-(Vadc*10000))/Vadc;
}
double R2(adc_unit_t ADC,uint8_t pin){       //Em um divisor de tensão com 10000 Ohm no resistor superior, retorna o valor ohmico
  double Vadc = tensao(ADC,pin);
  return Vadc * (10000/(3.3-Vadc));
}
double ntc_10k(adc_unit_t ADC,uint8_t pin){
  double  Rd0 = 10000.0,                      //Resistencia do NTC a 25°C
      T0 = 298.15,                            //25°C em Kelvin
      T1 = 273.15,                            //Temperatura de referência 1
      T2 = 373.15,                            //Temperatura de referêcia 2
      RT1 = 35563.0,                          //Reseistência do NTC em T1
      RT2 = 549.4,                            //Resistência do NTC em T2
      beta = log(RT1/RT2)/((1/T1)-(1/T2)),    //Constante do termistor
      Rinf = Rd0*exp(-beta/T0),               //Parâmetro de resistência
      Rntc,             //Resistencia do NTC
      temp; 
  Rntc = R1(ADC,pin);                
  temp = beta/log(Rntc/Rinf);       //Temperatura em Kelvin
  return temp - 273.15;         //Retorna Temperatura em Celsius
}
/*double Vin_div_T(unsigned char pin, double r1, double r2){
  double tens = tensao(pin);
  double res = (r2/(r1+r2));
  return (tens/res); //tensão/(R2/(R1+R2)) retorna a tensão de entrada do divisor de tensão
}*/
double Vin_b2(adc_unit_t ADC,uint8_t pin, double b1, double r1, double r2){
  return Vin_div_T(ADC,pin,r1,r2)-b1; //tensão/(R1/(R1+R2)) retorna a tensão de entrada do divisor de tensão menos a tensão da primeira bateria
}
