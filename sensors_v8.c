/*
 * Copyright (c) 2018, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/* THIS PROJECT IS THE CODE FOR MAJOR PROJECT 2019 TITLED ON BOARD KITE SENSOR ARRAY*/

#define _GPRMC_  1
#define _GPGGA_  2
#define _OTHER_  3

#include <stdint.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <stddef.h>
#include <unistd.h>
#include <pthread.h>
#include <string.h>
/* Driver Header files */
#include <ti/drivers/GPIO.h>
#include <ti/drivers/I2C.h>
#include <ti/drivers/UART.h>

/* Example/Board Header files */
#include "Board.h"
#include "MPU6050.h"

/*UART and I2C Handle Definitions */
  I2C_Handle      i2c;
  I2C_Params      i2cParams;

  UART_Handle uart;
  UART_Params uartParams;

/* I2C Read Write Function to create an I2C Transaction for Read or Write*/
int I2C_ReadWrite(I2C_Handle i2c, uint8_t RegAddr, uint8_t *txBuffer , uint8_t *rxBuffer, uint8_t WriteCount, uint8_t ReadCount)
  {
      I2C_Transaction i2cTransaction;

      txBuffer[0] = RegAddr;
      i2cTransaction.slaveAddress = MPU6050_ADDRESS;
      i2cTransaction.writeBuf = txBuffer;
      i2cTransaction.writeCount = WriteCount;
      i2cTransaction.readBuf = rxBuffer;
      i2cTransaction.readCount = ReadCount;

      if (I2C_transfer(i2c, &i2cTransaction)) {
                return 1;
              }
              else {
                  return 0;
              }
  }

/*Power Management Registers, Configuration Register and Sample Rate Divider Register Setup Values*/
int MPU6050_Setup(uint8_t *txBuffer, uint8_t *rxBuffer, uint8_t PWR1_SET , uint8_t PWR2_SET, uint8_t CONF, uint8_t SampleRate )
{
    //Check Power On Values
        int I2C_Flag = I2C_ReadWrite(i2c, WHO_AM_I_MPU6050, txBuffer, rxBuffer, 1,1);
        I2C_Flag = I2C_ReadWrite(i2c, PWR_MGMT_1, txBuffer, rxBuffer,1,1);

        //Write the Initial PWR Settings
        txBuffer[1] = PWR1_SET; //PWR_MGMT1 Register value
        I2C_Flag = I2C_ReadWrite(i2c, PWR_MGMT_1, txBuffer, NULL,2,0);
        txBuffer[1] = PWR2_SET; //PWR_MGMT2 Register value
        I2C_Flag = I2C_ReadWrite(i2c, PWR_MGMT_2, txBuffer, NULL,2,0);

        //Write Initial Configuration Register Settings
        txBuffer[1] = CONF; //CONFIG Register
        I2C_Flag = I2C_ReadWrite(i2c, CONFIG, txBuffer, NULL,2,0);
        txBuffer[1] = SampleRate; //SMPLRT_DIV Register
        I2C_Flag = I2C_ReadWrite(i2c, SMPLRT_DIV, txBuffer, NULL,2,0);

        return I2C_Flag;
}

/*MPU6050 Interrupt Setup Register*/
int MPU6050_INT_Setup(uint8_t *txBuffer, uint8_t *rxBuffer, uint8_t INT_CFG , uint8_t INT_EN)
{
    //Write Interrupt Configuration
    txBuffer[1] = INT_CFG;
    int I2C_Flag = I2C_ReadWrite(i2c, INT_PIN_CFG, txBuffer, NULL,2,0);
    //Write Interrupt Enable
    txBuffer[1] = INT_EN;
    I2C_Flag = I2C_ReadWrite(i2c, INT_ENABLE, txBuffer, NULL,2,0);

    return I2C_Flag;
}

/*Read Interrupt Status register and return status value */
uint8_t Read_INT()
{
    uint8_t txBuffer[1], rxBuffer[1];
    //Read
    int I2C_Flag = I2C_ReadWrite(i2c, INT_STATUS, txBuffer, rxBuffer, 1,1);
    uint8_t status = rxBuffer[0];

    return status;
}

/*Gyroscope Configuration Settings*/
int Gyro_Config(uint8_t *txBuffer, uint8_t *rxBuffer, uint8_t GYRO_SET)
{
    //Write
    txBuffer[1] = GYRO_SET;
    int I2C_Flag = I2C_ReadWrite(i2c, GYRO_CONFIG, txBuffer, NULL,2,0);
    //Read
    I2C_Flag = I2C_ReadWrite(i2c, GYRO_CONFIG, txBuffer, rxBuffer, 1,1);

    return I2C_Flag;
}

/*Accelerometer Configuration Settings*/
int Accel_Config(uint8_t *txBuffer, uint8_t *rxBuffer, uint8_t ACCEL_SET)
{
    //Write
    txBuffer[1] = ACCEL_SET;
    int I2C_Flag = I2C_ReadWrite(i2c, ACCEL_CONFIG, txBuffer, NULL, 2,0);
    //Read
    I2C_Flag = I2C_ReadWrite(i2c, ACCEL_CONFIG, txBuffer, rxBuffer, 1,1);

    return I2C_Flag;
}

/*Calibrate Accelerometer and Gyroscope with Initial values*/
void Calibrate(uint8_t *txBuffer, float *dest2, float *dest1)
{
    uint8_t data[12]; // data array to hold accelerometer and gyro x, y, z, data
    uint16_t ii, packet_count, fifo_count;
    int32_t gyro_bias[3] = {0, 0, 0}, accel_bias[3] = {0, 0, 0};

    // reset device, reset all registers, clear gyro and accelerometer bias registers
    txBuffer[1] = 0x08;
    int Flag = I2C_ReadWrite(i2c, PWR_MGMT_1, txBuffer, NULL, 2,0); // Write a one to bit 7 reset bit; toggle reset device
    usleep(100000);

    // get stable time source
    // Set clock source to be PLL with x-axis gyroscope reference, bits 2:0 = 001
    txBuffer[1] = 0x01;
    I2C_ReadWrite(i2c, PWR_MGMT_1, txBuffer, NULL, 2,0);
    txBuffer[1] = 0x00;
    I2C_ReadWrite(i2c, PWR_MGMT_2, txBuffer, NULL, 2,0);
    usleep(200000);

    // Configure device for bias calculation
    txBuffer[1] = 0x00;
    I2C_ReadWrite(i2c, INT_ENABLE, txBuffer, NULL, 2,0); // Disable all interrupts
    txBuffer[1] = 0x00;
    I2C_ReadWrite(i2c, FIFO_EN, txBuffer, NULL, 2,0); // Disable FIFO
    txBuffer[1] = 0x00;
    I2C_ReadWrite(i2c, PWR_MGMT_1, txBuffer, NULL, 2,0);  // Turn on internal clock source
    txBuffer[1] = 0x0C;
    I2C_ReadWrite(i2c, USER_CTRL, txBuffer, NULL, 2,0);   // Reset FIFO and DMP
    usleep(15000);

    // Configure MPU6050 gyro and accelerometer for bias calculation
    txBuffer[1] = 0x01;
    I2C_ReadWrite(i2c, CONFIG, txBuffer, NULL, 2,0); // Set low-pass filter to 188 Hz
    txBuffer[1] = 0x00;
    I2C_ReadWrite(i2c, SMPLRT_DIV, txBuffer, NULL, 2,0); //Set Sample Rate Divider to 1kHz
    txBuffer[1] = 0x00;
    I2C_ReadWrite(i2c, GYRO_CONFIG, txBuffer, NULL, 2,0); // Set gyro full-scale to 250 degrees per second, maximum sensitivity
    txBuffer[1] = 0x00;
    I2C_ReadWrite(i2c, ACCEL_CONFIG, txBuffer, NULL, 2,0); // Set accelerometer full-scale to 2 g, maximum sensitivity

    uint16_t  gyrosensitivity  = 131;   // = 131 LSB/degrees/sec
    uint16_t  accelsensitivity = 16384;  // = 16384 LSB/g

    // Configure FIFO to capture accelerometer and gyro data for bias calculation
    txBuffer[1] = 0x40;
    I2C_ReadWrite(i2c, USER_CTRL, txBuffer, NULL, 2,0);  // Enable FIFO
    txBuffer[1] = 0x78;
    I2C_ReadWrite(i2c, FIFO_EN, txBuffer, NULL, 2,0);   // Enable gyro and accelerometer sensors for FIFO  (max size 1024 bytes in MPU-6050)
    usleep(80000); // accumulate 80 samples in 80 milliseconds = 960 bytes

    // At end of sample accumulation, turn off FIFO sensor read
    txBuffer[1] = 0x00;
    I2C_ReadWrite(i2c, FIFO_EN, txBuffer, NULL, 2,0);   // Disable gyro and accelerometer sensors for FIFO
    txBuffer[1] = 0x78;
    I2C_ReadWrite(i2c, FIFO_COUNTH, txBuffer, data, 1,2); // read FIFO sample count
    fifo_count = ((uint16_t)data[0] << 8) | data[1];
    packet_count = fifo_count / 12; // How many sets of full gyro and accelerometer data for averaging

    for (ii = 0; ii < packet_count; ii++) {
      int16_t accel_temp[3] = {0, 0, 0}, gyro_temp[3] = {0, 0, 0};
      I2C_ReadWrite(i2c, ACCEL_XOUT_H, txBuffer, data, 1,12); // read data for averaging
      accel_temp[0] = (int16_t) (((int16_t)data[0] << 8) | data[1]  ) ;  // Form signed 16-bit integer for each sample in FIFO
      accel_temp[1] = (int16_t) (((int16_t)data[2] << 8) | data[3]  ) ;
      accel_temp[2] = (int16_t) (((int16_t)data[4] << 8) | data[5]  ) ;
      gyro_temp[0]  = (int16_t) (((int16_t)data[6] << 8) | data[7]  ) ;
      gyro_temp[1]  = (int16_t) (((int16_t)data[8] << 8) | data[9]  ) ;
      gyro_temp[2]  = (int16_t) (((int16_t)data[10] << 8) | data[11]) ;

      accel_bias[0] += (int32_t) accel_temp[0]; // Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
      accel_bias[1] += (int32_t) accel_temp[1];
      accel_bias[2] += (int32_t) accel_temp[2];
      gyro_bias[0]  += (int32_t) gyro_temp[0];
      gyro_bias[1]  += (int32_t) gyro_temp[1];
      gyro_bias[2]  += (int32_t) gyro_temp[2];

    }
    accel_bias[0] /= (int32_t) packet_count; // Normalize sums to get average count biases
    accel_bias[1] /= (int32_t) packet_count;
    accel_bias[2] /= (int32_t) packet_count;
    gyro_bias[0]  /= (int32_t) packet_count;
    gyro_bias[1]  /= (int32_t) packet_count;
    gyro_bias[2]  /= (int32_t) packet_count;

    if (accel_bias[2] > 0L) {
      accel_bias[2] -= (int32_t) accelsensitivity; // Remove gravity from the z-axis accelerometer bias calculation
    }
    else {
      accel_bias[2] += (int32_t) accelsensitivity;
    }

    // Construct the gyro biases for push to the hardware gyro bias registers, which are reset to zero upon device startup
    data[0] = (-gyro_bias[0] / 4  >> 8) & 0xFF; // Divide by 4 to get 32.9 LSB per deg/s to conform to expected bias input format
    data[1] = (-gyro_bias[0] / 4)       & 0xFF; // Biases are additive, so change sign on calculated average gyro biases
    data[2] = (-gyro_bias[1] / 4  >> 8) & 0xFF;
    data[3] = (-gyro_bias[1] / 4)       & 0xFF;
    data[4] = (-gyro_bias[2] / 4  >> 8) & 0xFF;
    data[5] = (-gyro_bias[2] / 4)       & 0xFF;

    dest1[0] = (float) gyro_bias[0] / (float) gyrosensitivity; // construct gyro bias in deg/s for later manual subtraction
    dest1[1] = (float) gyro_bias[1] / (float) gyrosensitivity;
    dest1[2] = (float) gyro_bias[2] / (float) gyrosensitivity;

    // Construct the accelerometer biases for push to the hardware accelerometer bias registers. These registers contain
    // factory trim values which must be added to the calculated accelerometer biases; on boot up these registers will hold
    // non-zero values. In addition, bit 0 of the lower byte must be preserved since it is used for temperature
    // compensation calculations. Accelerometer bias registers expect bias input as 2048 LSB per g, so that
    // the accelerometer biases calculated above must be divided by 8.

    int32_t accel_bias_reg[3] = {0, 0, 0}; // A place to hold the factory accelerometer trim biases
    I2C_ReadWrite(i2c, XA_OFFSET_H, txBuffer, data, 1,2); // Read factory accelerometer trim values
    accel_bias_reg[0] = (int16_t) ((int16_t)data[0] << 8) | data[1];
    I2C_ReadWrite(i2c, YA_OFFSET_H, txBuffer, data, 1,2);
    accel_bias_reg[1] = (int16_t) ((int16_t)data[0] << 8) | data[1];
    I2C_ReadWrite(i2c, ZA_OFFSET_H, txBuffer, data, 1,2);
    accel_bias_reg[2] = (int16_t) ((int16_t)data[0] << 8) | data[1];

    uint32_t mask = 1uL; // Define mask for temperature compensation bit 0 of lower byte of accelerometer bias registers
    uint8_t mask_bit[3] = {0, 0, 0}; // Define array to hold mask bit for each accelerometer bias axis

    for (ii = 0; ii < 3; ii++) {
      if (accel_bias_reg[ii] & mask) mask_bit[ii] = 0x01; // If temperature compensation bit is set, record that fact in mask_bit
    }

    // Construct total accelerometer bias, including calculated average accelerometer bias from above
    accel_bias_reg[0] -= (accel_bias[0] / 8); // Subtract calculated averaged accelerometer bias scaled to 2048 LSB/g (16 g full scale)
    accel_bias_reg[1] -= (accel_bias[1] / 8);
    accel_bias_reg[2] -= (accel_bias[2] / 8);

    data[0] = (accel_bias_reg[0] >> 8) & 0xFF;
    data[1] = (accel_bias_reg[0])      & 0xFF;
    data[1] = data[1] | mask_bit[0]; // preserve temperature compensation bit when writing back to accelerometer bias registers
    data[2] = (accel_bias_reg[1] >> 8) & 0xFF;
    data[3] = (accel_bias_reg[1])      & 0xFF;
    data[3] = data[3] | mask_bit[1]; // preserve temperature compensation bit when writing back to accelerometer bias registers
    data[4] = (accel_bias_reg[2] >> 8) & 0xFF;
    data[5] = (accel_bias_reg[2])      & 0xFF;
    data[5] = data[5] | mask_bit[2]; // preserve temperature compensation bit when writing back to accelerometer bias registers

    // Output scaled accelerometer biases for manual subtraction in the main program
    dest2[0] = (float)accel_bias[0] / (float)accelsensitivity;
    dest2[1] = (float)accel_bias[1] / (float)accelsensitivity;
    dest2[2] = (float)accel_bias[2] / (float)accelsensitivity;

}

/*Read Accelerometer Register Values*/
void Read_MPUAccel(uint8_t *txBuffer, uint8_t A_CONFIG, float *calib_accel_XYZ, float *AccelF)
{
    int i;
        uint8_t Data1[6];
        int Accel[3];
        float divider_a;
        uint8_t a = (A_CONFIG & 0x18);

        //Set Sensitivity Value For Accelerometer
        switch(a)
        {
        case 0 : divider_a = SSF_AFS_2G; break;
        case 8 : divider_a = SSF_AFS_4G; break;
        case 16 : divider_a = SSF_AFS_8G; break;
        case 24 : divider_a = SSF_AFS_16G; break;
        }

        int I2C_Flag = I2C_ReadWrite(i2c, ACCEL_XOUT_H, txBuffer, Data1, 1, 6);

        //Convert 8 bit unsigned integer values to 16 bit signed values and subtract the Calibrated readings

        Accel[0] = ((int16_t)(Data1[0] << 8 | Data1[1]));
        Accel[1] = ((int16_t)(Data1[2] << 8 | Data1[3]));
        Accel[2] = ((int16_t)(Data1[4] << 8 | Data1[5]));

        for(i=0;i<3;i++){
            AccelF[i] = (float)(Accel[i])/divider_a - (float)calib_accel_XYZ[i];
        }
}

/*Read Gyroscope Register Values*/
void Read_MPUGyro(uint8_t *txBuffer, uint8_t G_CONFIG, float *calib_gyro_XYZ, float *GyroF)
{
    int i;
       uint8_t Data2[6];
       int Gyro[3];
       float divider_g;
       uint8_t b = (G_CONFIG & 0x18);

       //Set Sensitivity value for Gyroscope
       switch(b)
        {
        case 0 : divider_g = SSF_GFS_250DPS; break;
        case 8 : divider_g = SSF_GFS_500DPS; break;
        case 16 : divider_g = SSF_GFS_1000DPS; break;
        case 24 : divider_g = SSF_GFS_2000DPS; break;
        }

       int I2C_Flag = I2C_ReadWrite(i2c, GYRO_XOUT_H, txBuffer, Data2, 1, 6);

       //Convert 8 bit unsigned integer values to 16 bit signed values and subtract the Calibrated readings
       Gyro[0] = ((int16_t)(Data2[0] << 8 | Data2[1]));
       Gyro[1] = ((int16_t)(Data2[2] << 8 | Data2[3]));
       Gyro[2] = ((int16_t)(Data2[4] << 8 | Data2[5]));

       for(i=0;i<3;i++){
           GyroF[i] = (float)(Gyro[i])/divider_g - (float)calib_gyro_XYZ[i];
       }
}

/*Read Temperature Sensor Register Value*/
void Read_MPUTemp(uint8_t *txBuffer, float *tempF)
{
       uint8_t Data3[2];

       int I2C_Flag = I2C_ReadWrite(i2c, TEMP_OUT_H, txBuffer, Data3, 1, 2);

       //Convert 8 bit unsigned integer values to 16 bit signed values and subtract the Calibrated readings
       tempF[0] = (int16_t)(Data3[0] << 8 | Data3[1]);

       tempF[0] = ((float)tempF[0])/340.0 + 36.53;
}

/*Read all Sensor Registers*/
void Read_MPUData(uint8_t *txBuffer, uint8_t G_CONFIG, uint8_t A_CONFIG, float *calib_accel_XYZ, float *calib_gyro_XYZ, float *AccelF, float *GyroF, float *tempF)
{
    int i;
    uint8_t Data1[6];
    uint8_t Data2[6];
    uint8_t Data3[2];
    int Accel[3];
    int Gyro[3];
    float divider_a;
    float divider_g;
    uint8_t a = (A_CONFIG & 0x18);
    uint8_t b = (G_CONFIG & 0x18);

    //Set Sensitivity Value For Accelerometer
    switch(a)
    {
    case 0 : divider_a = SSF_AFS_2G; break;
    case 8 : divider_a = SSF_AFS_4G; break;
    case 16 : divider_a = SSF_AFS_8G; break;
    case 24 : divider_a = SSF_AFS_16G; break;
    }

    //Set Sensitivity value for Gyroscope
    switch(b)
     {
     case 0 : divider_g = SSF_GFS_250DPS; break;
     case 8 : divider_g = SSF_GFS_500DPS; break;
     case 16 : divider_g = SSF_GFS_1000DPS; break;
     case 24 : divider_g = SSF_GFS_2000DPS; break;
     }

    int I2C_Flag = I2C_ReadWrite(i2c, ACCEL_XOUT_H, txBuffer, Data1, 1, 6);
    I2C_Flag = I2C_ReadWrite(i2c, GYRO_XOUT_H, txBuffer, Data2, 1, 6);
    I2C_Flag = I2C_ReadWrite(i2c, TEMP_OUT_H, txBuffer, Data3, 1, 2);

    //Convert 8 bit unsigned integer values to 16 bit signed values and subtract the Calibrated readings

    Accel[0] = ((int16_t)(Data1[0] << 8 | Data1[1]));
    Accel[1] = ((int16_t)(Data1[2] << 8 | Data1[3]));
    Accel[2] = ((int16_t)(Data1[4] << 8 | Data1[5]));
    Gyro[0] = ((int16_t)(Data2[0] << 8 | Data2[1]));
    Gyro[1] = ((int16_t)(Data2[2] << 8 | Data2[3]));
    Gyro[2] = ((int16_t)(Data2[4] << 8 | Data2[5]));
    tempF[0] = (int16_t)(Data3[0] << 8 | Data3[1]);

    tempF[0] = (float)(tempF[0])/340.0 + 36.53;

    for(i=0;i<3;i++){
        AccelF[i] = (float)(Accel[i])/divider_a - (float)calib_accel_XYZ[i];
        GyroF[i] = (float)(Gyro[i])/divider_g - (float)calib_gyro_XYZ[i];
    }
}

//---------------------------------------------------------SHANTY EDIT BEGINS--------------------------------------
//temporary global variables due to laziness
uint8_t GPRMC_ok = 0, GPGGA_ok = 0;
uint8_t char_number = 0, SentenceType = 0, Term;
char sentence[6], rawTime[11], rawDate[7], rawSpeed[6], rawCourse[6], rawSatellites[3], rawLatitude[13], rawLongitude[13], rawAltitude[7], buffer[12];

void stringcpy(char *str1, char *str2, uint8_t dir)
{
  uint8_t chr = 0;
  do
  {
    str2[chr + dir] = str1[chr];
  } while(str1[chr++] != '\0');
}


float parse_rawDegree(char *term_)
{
  float term_value = atof(term_)/100;
  int16_t term_dec = term_value;
  term_value -= term_dec;
  term_value  = term_value * 5/3 + term_dec;
  return term_value;
}

float Latitude()
{
  return parse_rawDegree(rawLatitude);
}

float Longitude()
{
  return parse_rawDegree(rawLongitude);
}

float Altitude()
{
  return atof(rawAltitude);
}

/*
void update(float* lat, float* lon, float* alt)
{
    *lat = Latitude();
    *lon = Longitude();
    *alt = Altitude();
}
*/
int gps_read()
{
    char temp, msg[20] = "CARRIAGE RETURN";
    //int dollar= 0;
    UART_read(uart,&temp, sizeof(temp));
    char c = temp;
  //  if(c == '\r')
   //     UART_write(uart, &msg, sizeof(msg));
  //  UART_write(uart, &temp, sizeof(temp));

      switch(c)
      {
      case '\r':  // sentence end
                //UART_read(uart,&temp, sizeof(temp));
          /*
          if(SentenceType == _GPRMC_)
                  GPRMC_ok = 1;
                if(SentenceType == _GPGGA_)
                  GPGGA_ok = 1;
                if(GPRMC_ok && GPGGA_ok) {
                  GPRMC_ok = GPGGA_ok = 0;
                  return 1;
                }
                break; */
          if(SentenceType == _GPGGA_)
          {
             //GPGGA_ok = 1;
             return 1;
          }

      case '$': // sentence start
                Term = char_number = 0;
                break;

              case ',':  // term end (new term start)
                buffer[char_number] = '\0';
                if(Term == 0) {
                  stringcpy(buffer, sentence, 0);
                  if(strcmp(sentence, "GPRMC") == 0)
                    SentenceType = _GPRMC_;
                  else if(strcmp(sentence, "GPGGA") == 0)
                         SentenceType = _GPGGA_;
                       else
                         SentenceType = _OTHER_;
                }

                // Latitude
                if((Term == 2) && (SentenceType == _GPGGA_)) {
                  stringcpy(buffer, rawLatitude, 1);
                }
                // Latitude N/S
                if((Term == 3) && (SentenceType == _GPGGA_)) {
                  if(buffer[0] == 'N')
                    rawLatitude[0] = '0';
                  else
                    rawLatitude[0] = '-';
                }

                // Longitude
                if((Term == 4) && (SentenceType == _GPGGA_)) {
                  stringcpy(buffer, rawLongitude, 1);
                }
                // Longitude E/W
                if((Term == 5) && (SentenceType == _GPGGA_)) {
                  if(buffer[0] == 'E')
                    rawLongitude[0] = '0';
                  else
                    rawLongitude[0] = '-';
                }
                // Altitude
                if((Term == 9) && (SentenceType == _GPGGA_)) {
                  stringcpy(buffer, rawAltitude, 0);
                }
                Term++;
                char_number = 0;
                break;

              default:
                buffer[char_number++] = c;
                break;
            }

            return 0;
}




void Read_GPSData(UART_Handle uart,  float *lat, float *lon, float *alt)
{

    while(1)
    {
       if(gps_read())
       {
           GPIO_write(Board_GPIO_LED0, Board_GPIO_LED_OFF);
           *lat = Latitude();
           *lon = Longitude();
           *alt = Altitude();
           return;
       }
       else
       {
           GPIO_toggle(Board_GPIO_LED0);
       }

    }
}
//--------------------------------------------------------SHANTY EDIT ENDS ------------------------------------------
 /*  ======== mainThread ========
 */
void *mainThread(void *arg0)
{
    //I2C Read Write Buffers
    uint8_t         txBuffer[4];
    uint8_t         rxBuffer[4];
    float        calib_accel_XYZ[3];
    float        calib_gyro_XYZ[3];
    float           Accel_Data[3];
    float           Gyro_Data[3];
    float           temperature[1];
    float           lat[1], lon[1], alt[1];
    char msg1[] = "UART Initialised\r\n";
    char msg2[] = "I2C Initialisation Fault\r\n";
    char msg3[] = "I2C Initialised\r\n";


    /* Call driver init functions */
    GPIO_init();
    I2C_init();
    UART_init();

    /* Configure the LED and if applicable */
    GPIO_setConfig(Board_GPIO_LED0, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW);
    GPIO_setConfig(Board_GPIO_LED1, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW);

    //Setup UART with Predetermined Settings
    UART_Params_init(&uartParams);
    uartParams.writeDataMode = UART_DATA_TEXT;
    uartParams.readDataMode = UART_DATA_TEXT;
    uartParams.readReturnMode = UART_RETURN_FULL;
    uartParams.readEcho = UART_ECHO_OFF;
    uartParams.baudRate = 9600;

       //Open UART
       uart = UART_open(Board_UART0, &uartParams);
            if (uart == NULL) {
             /* UART_open() failed Turn on GREEN LED */
                GPIO_write(Board_GPIO_LED1, Board_GPIO_LED_ON);
              }
              else {
                  //Print Message to Indicate UART is Initialized
                  GPIO_write(Board_GPIO_LED1, Board_GPIO_LED_OFF);
                  UART_write(uart, msg1, sizeof(msg1));
              }

    //Setup I2C with Predetermined Settings
    // Create I2C for usage
    I2C_Params_init(&i2cParams);
    i2cParams.bitRate = I2C_100kHz;

    //Check if I2C is Null Print. Print Ready if Ready
    i2c = I2C_open(Board_I2C0, &i2cParams);
     if (i2c == NULL) {
         /* I2C Bus_open() failed Turn on RED LED */
         GPIO_write(Board_GPIO_LED0, Board_GPIO_LED_ON);
         UART_write(uart, msg2, sizeof(msg2));
      }
       else{
           //Print Message to indicate I2C Initialised
           GPIO_write(Board_GPIO_LED0, Board_GPIO_LED_OFF);
           UART_write(uart, msg3, sizeof(msg3));
       }


     uint8_t PWR1 = 0x00;
     uint8_t PWR2 = 0x00;
     uint8_t CONF = 0x03;
     uint8_t SampleRate = 0x04;
     //Initial Power Management and Set up Configurations Setting of MPU6050
     int Flag = MPU6050_Setup(txBuffer, rxBuffer, PWR1 , PWR2, CONF, SampleRate);

    //Calibrate Accelerometer and Gyroscope
    Calibrate(txBuffer, calib_accel_XYZ,calib_gyro_XYZ);

    //Interrupt Setup on MPU6050
    uint8_t INT_CFG = 0x00;
    uint8_t INT_EN = 0x01;
    Flag = MPU6050_INT_Setup(txBuffer, rxBuffer, INT_CFG, INT_EN);

    //Write the Gyroscope Configuration Settings
    uint8_t G_CONFIG = 0x00;
    Flag = Gyro_Config(txBuffer, rxBuffer, G_CONFIG);

    //Write Accelerometer Configuration Settings
    uint8_t A_CONFIG = 0x00;
    Flag = Accel_Config(txBuffer, rxBuffer, A_CONFIG);

    //Read Accelerometer, Temperature, Gyroscope, Latitude, Longitude and Altitude Data forever Looping

    while(1)
    {
        uint8_t status = Read_INT();
        if( status && 0x01 )
        {
            //Turn OFF Green LED
            GPIO_write(Board_GPIO_LED1, Board_GPIO_LED_OFF);
            ///To Read All Sensor Data at Once
            //Read_MPUData(txBuffer, G_CONFIG, A_CONFIG, calib_accel_XYZ, calib_gyro_XYZ, Accel_Data, Gyro_Data, temperature);
            Read_MPUAccel(txBuffer,A_CONFIG,calib_accel_XYZ,Accel_Data);
            Read_MPUGyro(txBuffer,G_CONFIG,calib_gyro_XYZ,Gyro_Data);
            Read_MPUTemp(txBuffer,temperature);
            Read_GPSData(uart, lat, lon, alt);
        }

        else GPIO_toggle(Board_GPIO_LED1); //Toggle Green LED to Indicate Sensor Data not ready

    usleep(10000); //Sleep for 10msec
   }

    I2C_close(i2c);
    UART_close(uart);

    return (NULL);
}
