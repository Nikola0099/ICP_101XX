#include <stdint.h>
#include <stm32f1xx_hal_conf.h>
#include <stm32f1xx_it.h>
#include <string.h>
#include "math.h"

I2C_HandleTypeDef hi2c1;
uint8_t SensorAddressWrite = 0x76 << 1 | 0x00;
uint8_t SensorAddressRead = 0x76 << 1 | 0x01;

void readValue(float *pressure, float *temperature)
{
    uint8_t sensorAddressWrite = 0xC6;                        //Iz dokumentacije zavrsni bit 0 za write
    uint8_t sensorAddressRead = 0xC7;                         //Iz dokumentacije zavrsni bit 1 za read tj, 0xC7
    uint8_t sensorMode[2] = {0x48, 0xA3};
    float sensor_constants[4];
    short out[4];
    unsigned char data_write[10];
    unsigned char data_read[10] = {0};
    int i;
    float p_Pa_calib[3];
    float LUT_lower;
    float LUT_upper;
    float quadr_factor;
    float offst_factor;
    uint32_t p_dout;
    uint16_t t_dout;


    data_write[0] = 0xC5;                                                                     //Recimo senzoru da zelimo citati otp pomocu adrese 0xC59500669C
    data_write[1] = 0x95;
    data_write[2] = 0x00;
    data_write[3] = 0x66;
    data_write[4] = 0x9C;
    HAL_I2C_Master_Transmit(&hi2c1, sensorAddressWrite, data_write, 5, HAL_MAX_DELAY);

    //Read OTP values
    for(i = 0; i<4 ; i++)
    {
    data_write[0] = 0xC7;
    data_write[1] = 0xF7;
    HAL_I2C_Master_Transmit(&hi2c1, sensorAddressWrite, data_write, 2, HAL_MAX_DELAY);      //Citamo OTP vrednosti sekvencijalno, po 2 vrednosti + CRC 
    HAL_I2C_Master_Receive(&hi2c1, sensorAddressRead  , data_read, 3, HAL_MAX_DELAY);
    out[i] = data_read[0] << 8 | data_read[1];
    }

    //Init base
    for(i = 0; i < 4 ; i++){
    sensor_constants[i] = (float)out[i];                                                    //Konvertujemo u float i dodeljujemo
    }
    p_Pa_calib[0] = 45000.0;                                                                  //Konstante iz dokumentacije
    p_Pa_calib[1] = 80000.0;
    p_Pa_calib[2] = 105000.0;
    LUT_lower = 3.5 * (1<<20);
    LUT_upper = 11.5 * (1<<20);
    quadr_factor = 1 / 16777216.0;
    offst_factor = 2048.0;

        HAL_Delay(200);

        //Citanje P iz registra
    HAL_I2C_Master_Transmit(&hi2c1, sensorAddressWrite, sensorMode, 2, HAL_MAX_DELAY);        //Recimo mu da zelimo citanje, saljemo da zelimo prvo pressure pomocu SensorMode
    HAL_Delay(1000);
    HAL_I2C_Master_Receive(&hi2c1, sensorAddressRead  , data_read, 9, HAL_MAX_DELAY);         //Citamo 9 vrednosti od senzora, poredjane u obliku 2 vrednosti + CRC, 1., 2., 4. vrednost vezane za pritisak, 6. i 7. za temp 

    p_dout = data_read[0] << 16 | data_read[1]<<8 | data_read[3];                             //Raw P
    t_dout = data_read[6]<<8 | data_read[7];                                                  //Raw temp
    
    float t;         
    float s1,s2,s3;
    float in[3];
    float A,B,C;        //konstante za dobijanje pravih vrednosti temp i pritiska
    t = (float)(t_dout - 32768);            //dobijeno iz dokumentacije
    s1 = LUT_lower + (float)(sensor_constants[0] * t * t) * quadr_factor;
    s2 = offst_factor * sensor_constants[3] + (float)(sensor_constants[1] * t * t) * quadr_factor;
    s3 = LUT_upper + (float)(sensor_constants[2] * t * t) * quadr_factor;
    in[0] = s1;
    in[1] = s2;
    in[2] = s3;
    C = (in[0] * in[1] * (p_Pa_calib[0] - p_Pa_calib[1]) +in[1] * in[2] * (p_Pa_calib[1] - p_Pa_calib[2]) +
        in[2] * in[0] * (p_Pa_calib[2] - p_Pa_calib[0])) /
        (in[2] * (p_Pa_calib[0] - p_Pa_calib[1]) +
        in[0] * (p_Pa_calib[1] - p_Pa_calib[2]) +
        in[1] * (p_Pa_calib[2] - p_Pa_calib[0]));
    A = (p_Pa_calib[0] * in[0] - p_Pa_calib[1] * in[1] - (p_Pa_calib[1] - p_Pa_calib[0]) * C) / (in[0] - in[1]);
    B = (p_Pa_calib[0] - A) * (in[0] + C);
    *pressure = A + B / (C + p_dout);                                                          //Real P
    *temperature = -45.f + 175.f/65536.f * t_dout;                                             //Real Temp
}