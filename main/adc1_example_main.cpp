#include <stdio.h>
#include <stdlib.h>
#include "I2C.hpp"
#include "wifi_utils.h"






#define ADD 10000

// Time added in order to balance the integration and reduce the effect of the I2C function delays

// TO DO:
// ALGORITHM TEST USING A POWERSUPPLY
// measure the erro using the formula and compare it with the error experimentally
float SOC1 = 100, SOC2 = 100;
current fb={0,0,0};
#if SHUNT_CKT
const long double  DIVC=409.6;
#elif LPA_CKT
const long double  DIVC=((1L << 15) - 1) * 20 / (double)LPAPGA *TURN;
#elif HALL_CKT
const long double  DIVC= ((1L << 15) - 1)* (double)SENS/(double)HALLPGA;
#endif
const long double  DIVCP1= 1;
// DIVCP1=Cn1(In1)^pc1-1
const long double  DIVCP2= 1;
// DIVCP2=Cn2(In2)^pc2-1
const long double MAGOFFSET=((0.62)*((1L << 15) - 1)/(double)HALLPGA);
const long double Cn1=1;
const long double Cn2=1;
uint8_t SYS_ON=1;

uint8_t update=0;

ThingSpeak::Feed feed;

esp_adc_cal_characteristics_t *adc_chars;
const adc1_channel_t channel = (adc1_channel_t)ADC_CHANNEL_6;  // GPIO34 if ADC1, GPIO14 if ADC2
const adc1_channel_t channel2 =(adc1_channel_t) ADC_CHANNEL_7; // GPIO35 CHANNEL7
const adc_bits_width_t width = ADC_WIDTH_BIT_12;
const adc_atten_t atten = ADC_ATTEN_DB_0;
const adc_unit_t unit = ADC_UNIT_1;
void vTaskCurrentIntegration()
{

  long double Q1 = 0, Q2 = 0;
  current fa = {0,0,0}, fm = {0,0,0};
  //fb is a global variable
  int64_t Tb = 0, Ta = 0, Tm = 0;
  //T1 = 0, T2 = 0;
  double checker = 0;
 
dac_output_voltage(DAC_CHANNEL_2, TSTDAC);
  // FUNC
  // Re-check SOC integration
  // The peukert exponent varies, therefore a nominal discharge rate should be defined
  // read more about how SOC integration was implemented

  // In the final implementation, increase the ADD delay instead of the number of iterations
  // to have the same effect with the added benefit of accuracy
  while (1)
  {

    for (int i = 0; i < 500; i++)
    {

      Ta = (uint64_t)esp_timer_get_time();
      fa = EFF_CUR();
      delay(ADD);
      // We measure the time it takes to get a current reading + ADD delay
      Tm = (uint64_t)esp_timer_get_time();
      fm = EFF_CUR();
      delay(Tm - Ta);
      fb = EFF_CUR();
      Tb = (uint64_t)esp_timer_get_time();




      update=1;
      
      checker = (double)(fa.i + 4 * fm.i + fb.i - 6 * ID);
      if ((checker < 0) && (Tb - Ta) >= 0)
      {

        Q1 = (((double)(Tb - Ta) / (DIVT)) / 6) * (double)(fa.ip1 + 4 * fm.ip1 + fb.ip1) / DIVCP1;
        Q2 = (((double)(Tb - Ta) / (DIVT)) / 6) * (double)(fa.ip1 + 4 * fm.ip1 + fb.ip1) / DIVCP2;
        //Simpson's method of integration
      }
      else if ((checker < 0) && (Tb - Ta) < 0)
      {
        Q1 = (((double)(Ta - Tb) / (DIVT)) / 6) * (double)(fa.ip1 + 4 * fm.ip1 + fb.ip1) / DIVCP1;
        Q2 = (((double)(Ta - Tb) / (DIVT)) / 6) * (double)(fa.ip2 + 4 * fm.ip2 + fb.ip2) / DIVCP2;
      }
      else if ((checker >= 0) && (Tb - Ta) >= 0)
      {
        Q1 = (((double)(Tb - Ta) / (DIVT)) / 6) * NQ1 * checker / DIVC*Cn1;
        Q2 = (((double)(Tb - Ta) / (DIVT)) / 6) * NQ2 * checker / DIVC*Cn2;
      }

      else if ((checker >= 0) && (Tb - Ta) < 0)
      {
        Q1 = (((double)(Ta - Tb) / (DIVT)) / 6) * NQ1 * checker / DIVC*Cn1;
        Q2 = (((double)(Ta - Tb) / (DIVT)) / 6) * NQ2 * checker / DIVC*Cn2;
      }

      //SOC1 += Q1 * 100;
      //SOC2 += Q2 * 100;
      SOC1 +=Q1;
      SOC2 +=Q2;
    }
    

    ////////////////////////////////////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////////////
SOC1= (SOC1>100)? 100 : SOC1;
SOC2= (SOC2>100)? 100 : SOC2;
SOC1= (SOC1<0)? 0 : SOC1;
SOC2= (SOC2<0)? 0 : SOC2;
double I= (double)(fb.i - ID) /DIVC;
    printf("intg1:%Lf,intg2: %f,func:%lf, time:%lf,\n", Q2, SOC2, I,
           (double)(Tb - Ta) / DIVT);
            MEAS value = VBAT(channel, channel2, adc_chars, 256);
    /*  if(value.v1<=LTHRESHOLD1 || value.v1>=UTHRESHOLD1 ){
      SOC_SET(value,0);
      }
      if(value.v2<=LTHRESHOLD2 || value.v2>=UTHRESHOLD2 ){
      SOC_SET(value,1);
      }*/
    // Two calibration thresholds
#if SHUNT_CKT
    printf("Voltage: %lfV\tVoltage2: %lfV\tVoff:%lfV\n",
           value.v1, value.v2,
           value.vo);
#elif HALL_CKT || LPA_CKT
    printf("Voltage: %lfV\tVoltage2: %lfV\n",
           value.v1, value.v2);
#endif
if(-I > CUR_THSH ){
    SEND(value,SOC1);
    }
    else{

SYS_ON=0;
 printf("I: %lfA\t OFF\n",I);
    }

  }

}

extern "C" void app_main(void)
{

INIT();

  adc1_config_width(width);
  adc1_config_channel_atten(channel, atten);
  adc1_config_channel_atten(channel2, atten);

  // Characterize ADC
  adc_chars = (esp_adc_cal_characteristics_t*)calloc( 1,sizeof(esp_adc_cal_characteristics_t));
  esp_adc_cal_characterize(unit, atten, width,
                           DEFAULT_VREF, adc_chars);

  // SOC_SET(VBAT(channel, channel2, adc_chars,256),2);
  dac_output_enable(DAC_CHANNEL_2); // GPIO26


  wifiInit(SSID, PASSWORD);
  
    SET_READ_CUR();
  xTaskCreatePinnedToCore((TaskFunction_t)vTaskCurrentIntegration,
                          "INTEGRATION",
                          100000,
                          NULL,
                          1,
                          NULL,
                          1);

  /////////////////////////////////////////////////////////////

  ////////////////////////////////////////
  // Continuously sample ADC1
  while (1)
  {

if(SYS_ON){
    FEEDBACK();
}
else{
dac_output_voltage(DAC_CHANNEL_2, 0);
}
    // printf(" Core:%d\n", xPortGetCoreID());
    vTaskDelay((TickType_t)(50 / portTICK_PERIOD_MS));
  }
}

