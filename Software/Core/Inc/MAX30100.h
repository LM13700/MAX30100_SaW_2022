/*
 * MAX30100.h
 *
 *  Created on: May 31, 2022
 *      Author: Mateusz-stare
 */

#ifndef INC_MAX30100_H_
#define INC_MAX30100_H_

/*===========================================================================*
 *
 * INCLUDE SECTION
 *
 *===========================================================================*/

#include "stm32l4xx_hal.h"
#include "i2c.h"
#include <stdbool.h>

/*===========================================================================*
 *
 * EXPORTED DEFINES AND MACRO SECTION
 *
 *===========================================================================*/

#define MAX30100_SET_BIT(_BIT_OFFSET_)  ((uint8_t)(0x01 << (uint8_t)(_BIT_OFFSET_)))

#define MAX30100_PWR_RDY_OFFSET         (0x00)
#define MAX30100_SPO2_RDY_OFFSET        (0x04)

#define MAX30100_ENB_SPO2_RDY_OFFSET    (0x04)

#define MAX30100_RESET_OFFSET           (0x06)

#define MAX30100_LED_CURRENT_7_6        (0x02)
#define MAX30100_LED_CURRENT_11_0       (0x03)
#define MAX30100_LED_CURRENT_14_2       (0x04)
#define MAX30100_LED_CURRENT_17_4       (0x05)
#define MAX30100_LED_CURRENT_40_2       (0x0C)
#define MAX30100_RED_PA_OFFSET          (0x04)
#define MAX30100_IR_PA_OFFSET           (0x00)
#define MAX30100_SPO2_SR_OFFSET         (0x02)
#define MAX30100_SPO2_HI_RES_EN_OFFSET  (0x06)
#define MAX30100_LED_PW_13_BIT          (0x00)
#define MAX30100_LED_PW_16_BIT          (0x03)
#define MAX30100_SPO2_SR_100_SPS        (0x01)
#define MAX30100_SPO2_SR_400_SPS        (0x04)
#define MAX30100_SPO2_SR_600_SPS        (0x05)

#define MAX30100_MODE_HR                (0x02)
#define MAX30100_MODE_SPO2              (0x03)

#define MAX30100_LIGHT_THRESHOLD        (25000u)

/*===========================================================================*
 *
 * EXPORTED TYPES AND ENUMERATION SECTION
 *
 *===========================================================================*/

typedef enum MAX30100_Register_Tag
{
    /* Status */
    MAX30100_REGISTER_INTERRUPT_STATUS = 0x00,
    MAX30100_REGISTER_INTERRUPT_ENABLE = 0x01,
    /* FIFO */
    MAX30100_REGISTER_FIFO_WRITE_POINTER = 0x02,
    MAX30100_REGISTER_OVER_FLOW_CURRENT = 0x03,
    MAX30100_REGISTER_FIFO_READ_POINTER = 0x04,
    MAX30100_REGISTER_FIFO_DATA_REGISTER = 0x05,
    /* Configuration */
    MAX30100_REGISTER_MODE_CONFIGURATION = 0x06,
    MAX30100_REGISTER_SPO2_CONFIGURATION = 0x07,
    MAX30100_REGISTER_LED_CONFIGURATION = 0x09,
    /* Temperature */
    MAX30100_REGISTER_TEMP_INTEGER = 0x16,
    MAX30100_REGISTER_TEMP_FRACTION = 0x17,
    /* Part ID */
    MAX30100_REGISTER_REVISION_ID = 0xFE,
    MAX30100_REGISTER_PART_ID = 0xFF,

    MAX30100_REGISTER_COUNT
} MAX30100_Register_T;

/*===========================================================================*
 *
 * EXPORTED FUNCTION DECLARATION SECTION
 *
 *===========================================================================*/

/**
  * @brief Initialize MAX30100 sensor.
  * @param i2c - I2C handle structure representing connection with sensor
  * @note  None
  * @retval HAL Status
  */
HAL_StatusTypeDef Max30100_Init(I2C_HandleTypeDef* i2c);

/**
  * @brief Reset MAX30100 sensor.
  * @param None
  * @note  This is a blocking function.
  * @retval HAL Status
  */
HAL_StatusTypeDef Max30100_Reset(void);

/**
  * @brief Wait for MAX30100 sensor startup.
  * @param None
  * @note  This is a blocking function.
  * @retval HAL Status
  */
HAL_StatusTypeDef Max30100_WaitForSensorStartup(void);

/**
  * @brief Writes to MAX30100 sensor register.
  * @param writeRegister - specifies register of the MAX30100
  * @param data - data to be written to the register
  * @note  None
  * @retval HAL Status
  */
HAL_StatusTypeDef Max30100_Write(MAX30100_Register_T writeRegister, uint8_t data);

/**
  * @brief Reads from MAX30100 sensor register.
  * @param writeRegister - specifies register of the MAX30100
  * @param buffer - address of buffer where result will be stored
  * @note  None
  * @retval HAL Status
  */
HAL_StatusTypeDef Max30100_Read(MAX30100_Register_T readRegister, uint8_t* buffer);

/**
  * @brief Reads MAX30100 sample values.
  * @param redSample - address of buffer where red sample value will be stored
  * @param irSample - address of buffer where IR sample value will be stored
  * @note  None
  * @retval HAL Status
  */
HAL_StatusTypeDef Max30100_ReadSample(uint16_t* redSample, uint16_t* irSample);


#endif /* INC_MAX30100_H_ */

