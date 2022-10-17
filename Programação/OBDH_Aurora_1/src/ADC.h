/*
 * ADC.c
 *
 * Created: 12/03/2022 14:35:26
 * Author : Rodrigo Matos
 */ 
#ifndef ADC_H_
#define ADC_H_

#include <Arduino.h>
#include <driver/adc.h>
#include <esp_adc_cal.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_err.h>
#include <esp_log.h>


float tensao(adc_unit_t ADC,uint8_t pin);
double Vin_div_T(adc_unit_t ADC,uint8_t pin, double r1, double r2);
//double tensao(unsigned char);
double R1(uint8_t);
double R2(uint8_t);
double ntc_10k(adc_unit_t ADC,uint8_t pin);
//double Vin_div_T(unsigned char pin,double r1,double r2);
double Vin_b2(adc_unit_t ADC,uint8_t pin, double b1, double r1, double r2);
double porcento(uint8_t pin);




#endif /*ADC_H_ */
