#include <stdio.h>
#include <stdlib.h>
#include "I2C.hpp"
#include "wifi_utils.h"

#define ADD 10000

uint16_t SIZE=1964;
//1964 default for 1.35 days storage

//SIMPLE VOLTAGE AND CURRENT MONITORING WITH CLOUD STORAGE

DATA *RAMstorage = (DATA*)calloc(SIZE, sizeof(DATA));

current fb = {0, 0, 0};
#if SHUNT_CKT
const long double DIVC = 327.68;
#elif LAP_CKT
const long double DIVC = ((1L << 15) - 1) * 20 / (double)(LAPPGA * TURN);
#elif HALL_CKT
const long double DIVC = ((1L << 15) - 1) * (double)SENS / (double)HALLPGA;
#endif
const long double DIVCP1 = 804.9844719;
// DIVCP1=C1(In1)^pc1-1
const long double DIVCP2 = 1977.408979;
// DIVCP2=C2(In2)^pc2-1

// C1 and C2 should be in Cb

const long double MAGOFFSET = ((0.63) * ((1L << 15) - 1) / (double)HALLPGA);
const long double Cn1 = 1;
const long double Cn2 = 1;

// Cn1 and Cn2 should be in Cb
// uint8_t SYS_ON = 1;
uint16_t LAPOFFSET = 0;

// uint8_t update = 0;

ThingSpeak::Feed feed;

esp_adc_cal_characteristics_t *adc_chars;
const adc1_channel_t channel = (adc1_channel_t)ADC_CHANNEL_6;  // GPIO34 if ADC1, GPIO14 if ADC2
const adc1_channel_t channel2 = (adc1_channel_t)ADC_CHANNEL_7; // GPIO35 CHANNEL7
const adc_bits_width_t width = ADC_WIDTH_BIT_12;
const adc_atten_t atten = ADC_ATTEN_DB_0;
const adc_unit_t unit = ADC_UNIT_1;

extern "C" void app_main(void)
{
   vTaskDelay((TickType_t)(1000 / portTICK_PERIOD_MS));

  INIT();

  adc1_config_width(width);
  adc1_config_channel_atten(channel, atten);
  adc1_config_channel_atten(channel2, atten);

  // Characterize ADC
  adc_chars = (esp_adc_cal_characteristics_t *)calloc(1, sizeof(esp_adc_cal_characteristics_t));
  esp_adc_cal_characterize(unit, atten, width,
                           DEFAULT_VREF, adc_chars);



while (RAMstorage == NULL)
{  

  
  ESP_LOGE("RAM", "ALLOCATION FAILED");
  vTaskDelay((TickType_t)(1000 / portTICK_PERIOD_MS));
}

if(RAMstorage != NULL){
ESP_LOGI("RAM", "ALLOCATION SUCCESSFUL, SIZE:%d bytes",SIZE*sizeof(DATA));
}


  wifiInit(SSID, PASSWORD);
  ntpInit("UTC-1",0);
  time_t now = {0};
  tm timeinfo = {0};
  getTime(&now, &timeinfo,1);




  SET_READ_CUR();
#if LAP_CKT
  LAPOFFSET = I2C_READ();
#endif

  uint64_t  T1 = 0;
   uint16_t str=0,stri=0,snd=0,overstr=0,oversnd=0,overstri=0;
   bool status;


  while (SEND_SF() != 1)
  {

    ESP_LOGE("INTERNET:I", "INITIAL CONNECTION FAILED. RECONNECTING..");

    esp_wifi_connect();
    vTaskDelay((TickType_t)(1000 / portTICK_PERIOD_MS));
  }

  ESP_LOGI("INTERNET:I", "Internet Set");

  vTaskDelay((TickType_t)(dataperiod / portTICK_PERIOD_MS));



  while (1)
  {

    
     
    fb.i = I2C_READ();
    double I = roundf(((fb.i - ID) / DIVC) * 100) / 100;


    MEAS value = VBAT(channel, channel2, adc_chars, OVS);



#if SHUNT_CKT
   /* printf("Voltage: %fV\tVoltage2: %fV\tVoff:%fV\n",
           value.v1, value.v2,
           value.vo);*/
#elif LAP_CKT

    printf("Voltage: %fV\tVoltage2: %fV\n",
           value.v1, value.v2);
#elif HALL_CKT
    printf("Voltage: %fV\tVoltage2: %fV\n",
           value.v1, value.v2);
#endif


    getTime(&now, &timeinfo,1);
    RAMstorage[str].volt = value;
    RAMstorage[str].time = timeinfo;
    RAMstorage[str].cur = I;
    T1=(uint64_t)now;

     stri=str;
      overstri=overstr;
str++;
    if (str == SIZE)
    {
      str = 0;
      overstr++;
     

    }
   

    for (int i = 0; i < 3; i++)
    {

      status = SEND(RAMstorage[snd].volt, RAMstorage[snd].cur, RAMstorage[snd].time);
      
     

      char strftime_buf[64];
      strftime(strftime_buf, sizeof(strftime_buf), "%c", &RAMstorage[snd].time);
      printf("Voltage1:%fv ,Voltage2:%fv,Cur:%fA,Time:%s\n",RAMstorage[snd].volt.v1,RAMstorage[snd].volt.v2, RAMstorage[snd].cur,strftime_buf);
     // printf("FIRST: snd:%d,stri:%d,i:%d\n",snd,stri,i);
      if (status != 1)
      {
        esp_wifi_connect();
        break;
      }

      if (snd < stri || overstri > oversnd)
      {
        // snd shouldn't go past str (we store before sending)
        
        snd++;
        if (snd == SIZE)
        {
          snd = 0;
          oversnd++;
           //syncing time with the server
             ntpInit("UTC-1",1);

        }

      
      }
      else if( overstri>oversnd+1){
          oversnd=0;
          overstr=0;
          snd=0;

      }
      else if(snd == stri && overstri == oversnd){
        snd++;
        if (snd == SIZE)
        {
          snd = 0;
          oversnd++;
          //syncing time with the server
             ntpInit("UTC-1",1);

        }
        break;
        }
      
    
      vTaskDelay((TickType_t)(dataperiod / portTICK_PERIOD_MS));
    }
  
  while( (uint64_t)now-T1 <60){
      time(&now);

   vTaskDelay((TickType_t)(100 / portTICK_PERIOD_MS));
    };

    
  }
}

