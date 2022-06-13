/*
 * MAX30100.c
 *
 *  Created on: May 31, 2022
 *      Author: Mateusz-M
 */

#include "MAX30100.h"

#define MAX30100_ADDRESS                (0xAE)

#define MAX30100_TIMEOUT                (1u)

#define MAX30100_PART_ID                (0x11)

#define MAX30100_SAMPLES_BUF_LENGTH     (4u)


static I2C_HandleTypeDef* i2c_handle;

/*===========================================================================*
 *
 * EXPORTED FUNCTION DEFINITION SECTION
 *
 *===========================================================================*/

/*===========================================================================*
 * Function: Max30100_Write
 *===========================================================================*/
HAL_StatusTypeDef Max30100_Write(MAX30100_Register_T writeRegister, uint8_t data)
{
    HAL_StatusTypeDef result;

    result = HAL_I2C_Mem_Write(i2c_handle, MAX30100_ADDRESS, writeRegister, 1u, &data, sizeof(data), MAX30100_TIMEOUT);

    return result;
}

/*===========================================================================*
 * Function: Max30100_Read
 *===========================================================================*/
HAL_StatusTypeDef Max30100_Read(MAX30100_Register_T readRegister, uint8_t* buffer)
{
    HAL_StatusTypeDef result;

    result = HAL_I2C_Mem_Read(i2c_handle, MAX30100_ADDRESS, readRegister, 1u, buffer, sizeof(uint8_t), MAX30100_TIMEOUT);

    return result;
}

/*===========================================================================*
 * Function: Max30100_Reset
 *===========================================================================*/
HAL_StatusTypeDef Max30100_Reset(void)
{
    uint8_t writeBuf;
    uint8_t readBuf;
    HAL_StatusTypeDef result;

    writeBuf = MAX30100_SET_BIT(MAX30100_RESET_OFFSET);

    result = Max30100_Write(MAX30100_REGISTER_MODE_CONFIGURATION, writeBuf);
    if (result != HAL_OK)
    {
        goto max30100_reset_exit;
    }

    do
    {
        result = Max30100_Read(MAX30100_REGISTER_MODE_CONFIGURATION, &readBuf);
        if (result != HAL_OK)
        {
            goto max30100_reset_exit;
        }
    } while ((readBuf & MAX30100_SET_BIT(MAX30100_RESET_OFFSET)) != 0u);

    /* TODO: Add timeout */
max30100_reset_exit:

    return result;
}

/*===========================================================================*
 * Function: Max30100_WaitForSensorStartup
 *===========================================================================*/
HAL_StatusTypeDef Max30100_WaitForSensorStartup(void)
{
    uint8_t readBuf;
    HAL_StatusTypeDef result;

    do
    {
        result = Max30100_Read(MAX30100_REGISTER_INTERRUPT_STATUS, &readBuf);
        if (result != HAL_OK)
        {
            goto max30100_wait_for_sensor_exit;
        }
    } while ((readBuf & MAX30100_SET_BIT(MAX30100_PWR_RDY_OFFSET)) != 0u);

    /* TODO: Add timeout */
max30100_wait_for_sensor_exit:

    return result;
}

/*===========================================================================*
 * Function: Max30100_Init
 *===========================================================================*/
HAL_StatusTypeDef Max30100_Init(I2C_HandleTypeDef* i2c)
{
    uint8_t readBuf;
    uint8_t writeBuf;
    HAL_StatusTypeDef result;

    i2c_handle = i2c;

    result = Max30100_WaitForSensorStartup();
    if (result != HAL_OK)
    {
        goto max30100_init_exit;
    }

    result = Max30100_Reset();
    if (result != HAL_OK)
    {
        goto max30100_init_exit;
    }

    result = Max30100_Read(MAX30100_REGISTER_PART_ID, &readBuf);
    if (result != HAL_OK)
    {
        goto max30100_init_exit;
    }

    if (readBuf != MAX30100_PART_ID)
    {
        result = HAL_ERROR;
        goto max30100_init_exit;
    }

    writeBuf = MAX30100_MODE_SPO2;
    result = Max30100_Write(MAX30100_REGISTER_MODE_CONFIGURATION, writeBuf);
    if (result != HAL_OK)
    {
        goto max30100_init_exit;
    }

    writeBuf = 0x00;
    result = Max30100_Write(MAX30100_REGISTER_FIFO_WRITE_POINTER, writeBuf);
    if (result != HAL_OK)
    {
        goto max30100_init_exit;
    }

    result = Max30100_Write(MAX30100_REGISTER_OVER_FLOW_CURRENT, writeBuf);
    if (result != HAL_OK)
    {
        goto max30100_init_exit;
    }

    result = Max30100_Write(MAX30100_REGISTER_FIFO_READ_POINTER, writeBuf);
    if (result != HAL_OK)
    {
        goto max30100_init_exit;
    }

    writeBuf = ((MAX30100_LED_CURRENT_40_2 << MAX30100_RED_PA_OFFSET) |
                (MAX30100_LED_CURRENT_40_2 << MAX30100_IR_PA_OFFSET));
    result = Max30100_Write(MAX30100_REGISTER_LED_CONFIGURATION, writeBuf);
    if (result != HAL_OK)
    {
        goto max30100_init_exit;
    }

    writeBuf = (1u << MAX30100_SPO2_HI_RES_EN_OFFSET) | (MAX30100_SPO2_SR_100_SPS << MAX30100_SPO2_SR_OFFSET) |
               (MAX30100_LED_PW_16_BIT);
    result = Max30100_Write(MAX30100_REGISTER_SPO2_CONFIGURATION, writeBuf);
    if (result != HAL_OK)
    {
        goto max30100_init_exit;
    }

    writeBuf = MAX30100_SET_BIT(MAX30100_ENB_SPO2_RDY_OFFSET);
    result = Max30100_Write(MAX30100_REGISTER_INTERRUPT_ENABLE, writeBuf);
    if (result != HAL_OK)
    {
        goto max30100_init_exit;
    }

    Max30100_Read(MAX30100_REGISTER_SPO2_CONFIGURATION, &readBuf);

max30100_init_exit:

    return result;
}

/*===========================================================================*
 * Function: Max30100_ReadSample
 *===========================================================================*/
HAL_StatusTypeDef Max30100_ReadSample(uint16_t* redSample, uint16_t* irSample)
{
    uint8_t readBuf[MAX30100_SAMPLES_BUF_LENGTH];
    HAL_StatusTypeDef result;

    result = HAL_I2C_Mem_Read(i2c_handle, MAX30100_ADDRESS, MAX30100_REGISTER_FIFO_DATA_REGISTER, 1u,
                              readBuf, MAX30100_SAMPLES_BUF_LENGTH, MAX30100_TIMEOUT);

    if (result != HAL_OK)
    {
        goto max30100_read_sample_exit;
    }

    *irSample = (readBuf[0] << 8u) | readBuf[1];
    *redSample = (readBuf[2] << 8u) | readBuf[3];

max30100_read_sample_exit:

    return result;
}
