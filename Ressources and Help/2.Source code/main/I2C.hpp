#ifndef INA_I2C_H_
#define INA_I2C_H_

#include "driver/i2c.h"
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include <driver/dac.h>
#include "esp_sntp.h"

#define NTP_TAG "NTP"

// Thingspeak libraries and defines
#include <iostream>
#include "esp_tls.h"
#include "esp_http_client.h"
#include "ThingSpeak.h"
//WIFI PASSWORD AND NAME BELOW
#define SSID ""
//WIFI NAME
#define PASSWORD ""
//WIFI PASSWORD
#define THINGSPEAK_CHANNEL_ID "1735559"
#define THINGSPEAK_API_KEY "8B8HSNQADTQXQ4QA" // Write api key
#define THINGSPEAK_CHANNEL_ID2 "1757797"
#define THINGSPEAK_API_KEY2 "E4FDL7WHF8UE1MZV" // Write api key

#define OVS 1024
// OVS factor

#define SHUNT_CKT 1
#define LAP_CKT 0
#define HALL_CKT 0
/*Current method:
SHUNT_CKT-Shunt based sensing
LAP_CKT -Current transducer
HALL_CKT-Hall effect sensor
ONLY one should be active (1) at a time
*/

#define ACK 0x1
#define WR 0x0
#define RD 0x1
#define CONFIG_REG 0x00
#define CAL_REG 0x05
#define CUR_REG 0x04
#define VS_REG 0x01
/// ADS External ADC registers
#define ADS_CONFIG 0x01
#define ADS_CUR 0x00

#define i2c_master_port I2C_NUM_0
#define WAIT 0xffff
#define addrINA 0x40
#define addrADS 0x48
#define LTHRESHOLD1 10
#define UTHRESHOLD1 13
#define LTHRESHOLD2 10
#define UTHRESHOLD2 13
#define DEFAULT_VREF 1086 //in mV

#define calv1 1
#define calv2 1
#define VLSB 100000
#define DIVT 1000000
#define LAPPGA 0.256
#define HALLPGA 1.024
#define TURN 2000
// turn ration of the CT
#define SENS 0.02
// sensitivity of the hall sensor
#define LMT 3
// current of the active load
#define LMTDAC 243
// maximum output for the DAC
// 243 key value for 6 A
#define CUR_THSH 0.05
// cut off current ( for battery discharge tests)
#define WAITFEEDBACK 90000000
// wait time to confirm cutoff current

#define pc1 1.5
#define pc2 1.2

#define NQ1 1
// NQ1=charge efficiency/nominal capacity of battery1
#define NQ2 1
// NQ2=charge efficiency/nominal capacity of battery2
#define ID 0
// Self-discharge current in CUR_LSBs
// CUR_LSB=50µA
#define acoef 1
#define bcoef 1
#define ccoef 1
#define dcoef 1
// the coefficients of the linear approximation

#define dataperiod 19000
// time to wait before sending data again

#define MAX_RETRIES 5

#define CHECK(err) ESP_ERROR_CHECK(err);


struct
{

  long double ip1;
  long double ip2;
  int16_t i;

} typedef current;

typedef struct  __attribute__((packed))
{
  float v1;
  float v2;
  float vo;
}MEAS;

typedef struct   __attribute__((packed))
{
MEAS volt;
tm time;
double cur;
} DATA;

extern float SOC1;
extern float SOC2;
// Configure ADC
extern esp_adc_cal_characteristics_t *adc_chars;
extern const adc1_channel_t channel;  // GPIO34 if ADC1, GPIO14 if ADC2
extern const adc1_channel_t channel2; // GPIO35 CHANNEL7
extern const adc_bits_width_t width;
extern const adc_atten_t atten;
extern const adc_unit_t unit;
extern const long double DIVC;
extern const long double DIVCP1;
extern const long double DIVCP2;
extern const long double MAGOFFSET;
extern const long double Cn1;
extern const long double Cn2;
extern current fb;
extern uint8_t update;
extern ThingSpeak::Feed feed;
extern uint8_t SYS_ON;
extern uint16_t LAPOFFSET;
extern uint16_t SIZE;

void SET_REG(uint8_t ADDR, uint8_t REG, uint8_t *DATA, uint8_t NUM);
void SET_READ(uint8_t ADDR, uint8_t REG);
int16_t I2C_READ();
void INIT();
current EFF_CUR();
void delay(uint64_t wait);
MEAS VBAT(adc1_channel_t channel, adc1_channel_t channel2,
          esp_adc_cal_characteristics_t *adc_chars, int NO_OF_SAMPLES);
void SOC_SET(MEAS vini, uint8_t num);
void FEEDBACK();
void SET_READ_CUR();
int SEND(MEAS value, float Current, tm datetime);
int SEND_SF();
void ntpInit(const char *time_zone,bool sync);
void getTime(time_t *now, tm *timeinfo,bool inf);
#endif