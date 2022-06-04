#include "I2C.hpp"








void SET_REG(uint8_t ADDR, uint8_t REG, uint8_t *DATA, uint8_t NUM)
{

  i2c_cmd_handle_t cmd = i2c_cmd_link_create();

  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, ((ADDR << 1) | WR), ACK);
  i2c_master_write_byte(cmd, REG, ACK);
  i2c_master_write(cmd, DATA, NUM, ACK);
  i2c_master_stop(cmd);
  esp_err_t err = i2c_master_cmd_begin(i2c_master_port, cmd, WAIT);
  CHECK(err);
  i2c_cmd_link_delete(cmd);
  CHECK(err);
}

void SET_READ(uint8_t ADDR, uint8_t REG)
{
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, ((ADDR << 1) | WR), ACK);
  i2c_master_write_byte(cmd, REG, ACK);
  i2c_master_stop(cmd);
  esp_err_t err = i2c_master_cmd_begin(i2c_master_port, cmd, WAIT);
  CHECK(err);
  i2c_cmd_link_delete(cmd);
  CHECK(err);
}
void SET_READ_CUR()
{
#if SHUNT_CKT
  SET_READ(addrINA, CUR_REG);
#elif LAP_CKT || HALL_CKT
  SET_READ(addrADS, ADS_CUR);
#endif
}



void INIT()
{

  // measure offset

  i2c_config_t conf; 

      conf.mode = I2C_MODE_MASTER;
      conf.sda_io_num = GPIO_NUM_25;
      conf.sda_pullup_en = GPIO_PULLUP_DISABLE;
      conf.scl_io_num = GPIO_NUM_27;
      conf.scl_pullup_en = GPIO_PULLUP_DISABLE;
      conf.master.clk_speed = 390000;


  esp_err_t err = i2c_param_config(i2c_master_port, &conf);
 
  CHECK(err);
  err = i2c_driver_install(i2c_master_port, conf.mode, 0, 0, 0);
  CHECK(err);

#if SHUNT_CKT
  uint8_t DATA1[2] = {0x21, 0x9d};
  //40mV PGA
  uint8_t DATA2[2] = {0x57, 0x61};
  //for 10A  (12 bits)

  SET_REG(addrINA, CONFIG_REG, DATA1, 2);
  SET_REG(addrINA, CAL_REG, DATA2, 2);
#elif LAP_CKT
 uint8_t DATA1[2] = {0x8a, 0xe3};
  //AINp= AIN0
  //AINn=AIN1
  SET_REG(addrADS, ADS_CONFIG, DATA1, 2);
#elif HALL_CKT
  uint8_t DATA1[2] = {0x86, 0xe3};
  //AINp=AIN0
  //AINn=AIN1
  SET_REG(addrADS, ADS_CONFIG, DATA1, 2);
#endif  
}






int16_t I2C_READ()
{
#if SHUNT_CKT
  uint8_t ADDR = addrINA;
#elif LAP_CKT || HALL_CKT
  uint8_t ADDR = addrADS;
#endif

  uint8_t resultA[2] = {0};
  uint16_t result = 0;
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  esp_err_t err = i2c_master_start(cmd);
  i2c_master_write_byte(cmd, ((ADDR << 1) | RD), ACK);
  // i2c_master_read(cmd,resultA,2,I2C_MASTER_ACK);
  i2c_master_read_byte(cmd, &resultA[0], I2C_MASTER_ACK);
  i2c_master_read_byte(cmd, &resultA[1], I2C_MASTER_LAST_NACK);
  i2c_master_stop(cmd);
  err = i2c_master_cmd_begin(i2c_master_port, cmd, WAIT);
  CHECK(err);
  i2c_cmd_link_delete(cmd);
  CHECK(err);

#if SHUNT_CKT 
result = (((uint16_t)resultA[0] << 8) | ((uint16_t)resultA[1]));
#elif LAP_CKT
  result = (((uint16_t)resultA[0] << 8) | ((uint16_t)resultA[1])) - LAPOFFSET;
#elif HALL_CKT
  result = (((uint16_t)resultA[0] << 8) | ((uint16_t)resultA[1])) -  MAGOFFSET;
#endif
  return result;
}




void delay(uint64_t wait)
{
  uint64_t microseconds = (uint64_t)esp_timer_get_time();
  while ((uint64_t)esp_timer_get_time() >= 0xffffffffffffffff - wait)
    ;
  while ((uint64_t)esp_timer_get_time() - microseconds < wait)
    ;
}

current EFF_CUR()
{

  int16_t var = I2C_READ();

  current HL = {0,0,0};
  if (((var - ID) < 0))
  {
    HL.i = var;
    HL.ip1 = -powl((double)(-(var - ID)) / DIVC, pc1);
    HL.ip2 = -powl((double)(-(var - ID)) / DIVC, pc2);
  }
  else
  {
    HL.i = var;
    HL.ip1 = powl((double)((var - ID)) / DIVC, pc1);
    HL.ip2 = powl((double)((var - ID)) / DIVC, pc2);
  }

  return HL;
}



MEAS VBAT(adc1_channel_t channel, adc1_channel_t channel2,
          esp_adc_cal_characteristics_t *adc_chars, int NO_OF_SAMPLES)
{
 long double MULT1=39.65384615;
long double MULT2=22.34759358;
//MULT is the inverse of the attenuation of the voltage dividers
  uint32_t adc_reading = 0;
  uint32_t adc_reading2 = 0;
  double voff = 0.0;
  // Multisampling
  for (int i = 0; i < NO_OF_SAMPLES; i++)
  {
    adc_reading += adc1_get_raw((adc1_channel_t)channel);
    adc_reading2 += adc1_get_raw((adc1_channel_t)channel2);
  }
  adc_reading /= NO_OF_SAMPLES;
  adc_reading2 /= NO_OF_SAMPLES;
  // Convert adc_reading to voltage in mV
  double voltage = (double)esp_adc_cal_raw_to_voltage(adc_reading, adc_chars) / 1000;   // GPIO34
  double voltage2 = (double)esp_adc_cal_raw_to_voltage(adc_reading2, adc_chars) / 1000; // GPIO35
  double cf = calv1 * MULT1;                                              // GPIO34 (24 V) 260(130x2) ohms
  double cf2 = calv2 * MULT2;                                             // GPIO35 (12 V) 470 ohms
#if SHUNT_CKT
  SET_READ(addrINA, VS_REG);
  voff = (double)I2C_READ() / VLSB;

    SET_READ_CUR();
#endif
  MEAS batteries = {(voltage2)*cf2 - voff, (voltage)*cf - (voltage2)*cf2, voff};
  return batteries;
}

void SOC_SET(MEAS vini, uint8_t num)
{
  if (num == 0)
  {
    SOC1 = (vini.v1 - bcoef) / acoef;
  }
  else if (num == 1)
  {
    SOC2 = (vini.v1 - dcoef) / ccoef;
  }
  else if (num == 2)
  {
    SOC1 = (vini.v1 - bcoef) / acoef;
    SOC2 = (vini.v1 - dcoef) / ccoef;
  }
  // linear approximation
}

void FEEDBACK()
{

  double Current=0.0;
  static uint8_t VGS = 0;
  //2.89V => 4V amplified
   //T3 = (uint64_t)esp_timer_get_time();

  for (int i = 0; i < 500; i++)
  {
    
     Current = -1*((double)(fb.i - ID) / DIVC);
     
     
   
  if(update ==1){
    if (Current > LMT && VGS != 0)
    {
      VGS--;
      
    }
    else if (Current < LMT && VGS != LMTDAC)
    {
      VGS++;
    }

    dac_output_voltage(DAC_CHANNEL_2, VGS);
    update=0;
   
    }

  }

  
    /*T4 = (uint64_t)esp_timer_get_time();
  printf("RESPONSE: %lf\t CURRENT :%lf A\n",(double)(T4 - T3) / DIVT,Current);*/
}


void SEND(MEAS value,float Charge1,float Current){





    // Create Feed
    
    feed.created_at = "";   // setting it to "" uses current datetime, setting to datetime_iso8061 stores the data with this specific datetime
    feed.fields.push_back({1, (float)(value.v1)}); //< field1 = 11.11f
    feed.fields.push_back({3, (Charge1)}); //< field3 = 33.33f
    feed.fields.push_back({5, (float)(Current)}); //< field3 = 33.33f


    // Post Feed  
   ThingSpeak client(THINGSPEAK_CHANNEL_ID, THINGSPEAK_API_KEY);
    client.post(feed);



}