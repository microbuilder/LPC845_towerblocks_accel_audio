/*
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
 * Copyright 2018 NXP
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef _FSL_FXOS8700_H_
#define _FSL_FXOS8700_H_

#include "fsl_common.h"
#if defined(FSL_FEATURE_SOC_LPI2C_COUNT) && (FSL_FEATURE_SOC_LPI2C_COUNT > 0)
#include "fsl_lpi2c.h"
#else
#include "fsl_i2c.h"
#endif

/*!
 * @brief Register definitions for the FXOS8700.
 */
enum _fxos8700_constants
{
    kFXOS8700_STATUS          = 0x00, /**< 0x00 */
    kFXOS8700_OUT_X_MSB       = 0x01, /**< 0x01 */
    kFXOS8700_OUT_X_LSB       = 0x02, /**< 0x02 */
    kFXOS8700_OUT_Y_MSB       = 0x03, /**< 0x03 */
    kFXOS8700_OUT_Y_LSB       = 0x04, /**< 0x04 */
    kFXOS8700_OUT_Z_MSB       = 0x05, /**< 0x05 */
    kFXOS8700_OUT_Z_LSB       = 0x06, /**< 0x06 */
    kFXOS8700_F_SETUP         = 0x09, /**< 0x06 */
    kFXOS8700_WHO_AM_I        = 0x0D, /**< 0x0D (default value = 0b11000111, read only) */
    kFXOS8700_XYZ_DATA_CFG    = 0x0E, /**< 0x0E */
    kFXOS8700_CTRL_REG1       = 0x2A, /**< 0x2A (default value = 0b00000000, read/write) */
    kFXOS8700_CTRL_REG2       = 0x2B, /**< 0x2B (default value = 0b00000000, read/write) */
    kFXOS8700_CTRL_REG3       = 0x2C, /**< 0x2C (default value = 0b00000000, read/write) */
    kFXOS8700_CTRL_REG4       = 0x2D, /**< 0x2D (default value = 0b00000000, read/write) */
    kFXOS8700_CTRL_REG5       = 0x2E, /**< 0x2E (default value = 0b00000000, read/write) */
    kFXOS8700_MSTATUS         = 0x32, /**< 0x32 */
    kFXOS8700_MOUT_X_MSB      = 0x33, /**< 0x33 */
    kFXOS8700_MOUT_X_LSB      = 0x34, /**< 0x34 */
    kFXOS8700_MOUT_Y_MSB      = 0x35, /**< 0x35 */
    kFXOS8700_MOUT_Y_LSB      = 0x36, /**< 0x36 */
    kFXOS8700_MOUT_Z_MSB      = 0x37, /**< 0x37 */
    kFXOS8700_MOUT_Z_LSB      = 0x38, /**< 0x38 */
    kFXOS8700_MCTRL_REG1      = 0x5B, /**< 0x5B (default value = 0b00000000, read/write) */
    kFXOS8700_MCTRL_REG2      = 0x5C, /**< 0x5C (default value = 0b00000000, read/write) */
    kFXOS8700_MCTRL_REG3      = 0x5D, /**< 0x5D (default value = 0b00000000, read/write) */
};

typedef struct _fxos_data
{
    uint8_t accelXMSB;
    uint8_t accelXLSB;
    uint8_t accelYMSB;
    uint8_t accelYLSB;
    uint8_t accelZMSB;
    uint8_t accelZLSB;
    uint8_t magXMSB;
    uint8_t magXLSB;
    uint8_t magYMSB;
    uint8_t magYLSB;
    uint8_t magZMSB;
    uint8_t magZLSB;
} fxos_data_t;

/*! @brief fxos8700 configure definition. This structure should be global.*/
typedef struct _fxos_handle
{
#if defined(FSL_FEATURE_SOC_LPI2C_COUNT) && (FSL_FEATURE_SOC_LPI2C_COUNT)
    LPI2C_Type *base;
    lpi2c_master_transfer_t xfer;
    lpi2c_master_handle_t *i2cHandle;
#else
    I2C_Type *base;                 /*!< I2C base. */
    i2c_master_handle_t *i2cHandle; /*!< I2C master transfer context */
    i2c_master_transfer_t xfer;     /*!< I2C master xfer */
#endif
} fxos_handle_t;

/*******************************************************************************
 * API
 ******************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif

/*!
 * @brief Initialize the FXOS8700 driver instance.
 *
 * @param accel_device Device driver state structure that will be filled in by this function.
 *      It is the responsibility of the caller to provide storage for this structure, and
 *      to free that storage when the driver is no longer needed.
 *
 * @return kStatus_Success if success or kStatus_Fail if error.
 */
status_t FXOS_Init(fxos_handle_t *handle);

/*!
 * @brief Read the current acceleration and magnetometer values.
 *
 * @param handle Pointer to a valid accel_device instance structure.
 * @param accel Pointer to a structure that will be filled in with the current acceleration
 *      and magnetometer values for all three axes.
 *
 * @return kStatus_Success if success or kStatus_Fail if error.
 */
status_t FXOS_ReadSensorData(fxos_handle_t *handle, fxos_data_t *data);

/*!
 * @brief Read the value of the specified register.
 *
 * @param handle Pointer to a valid accel_device instance structure.
 * @param reg variable store address of register
 * @param val pointer store return value.
 *
 * @return kStatus_Success if success or kStatus_Fail if error.
 */
status_t FXOS_ReadReg(fxos_handle_t *handle, uint8_t reg, uint8_t *val);

/*!
 * @brief Write the value to the specified register.
 *
 * @param handle Pointer to a valid accel_device instance structure.
 * @param reg variable store address of register
 * @param val pointer store value which is writen to accelerometer.
 *
 * @return kStatus_Success if success or kStatus_Fail if error.
 */
status_t FXOS_WriteReg(fxos_handle_t *handle, uint8_t reg, uint8_t val);

#if defined(__cplusplus)
}
#endif

#endif /* _FSL_FXOS8700_H_ */
