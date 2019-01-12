/*
 * Copyright (c) 2013-2015, Freescale Semiconductor, Inc.
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

#include "fsl_fxos.h"

/******************************************************************************
 * Code
 ******************************************************************************/
#if defined(FSL_FEATURE_SOC_LPI2C_COUNT) && (FSL_FEATURE_SOC_LPI2C_COUNT)
volatile static bool g_completionFlag = false;
volatile static bool g_nakFlag = false;

void FXOS_master_callback(LPI2C_Type *base, lpi2c_master_handle_t *handle, status_t status, void *userData)
{
    /* Signal transfer success when received success status. */
    if (status == kStatus_Success)
    {
        g_completionFlag = true;
    }
    /* Signal transfer success when received success status. */
    if (status == kStatus_LPI2C_Nak)
    {
        g_nakFlag = true;
    }
}
#endif

status_t FXOS_Init(fxos_handle_t *handle)
{
    uint8_t val = 0;

    /* Reset sensor */
    if (FXOS_ReadReg(handle, kFXOS8700_CTRL_REG2, &val) != kStatus_Success)
    {
        return kStatus_Fail;
    }
    val &= (uint8_t)(~(1 >> 7));
    if (FXOS_WriteReg(handle, kFXOS8700_CTRL_REG2, val) != kStatus_Success)
    {
        return kStatus_Fail;
    }

    /* Put the fxos8700 into standby mode */
    if (FXOS_ReadReg(handle, kFXOS8700_CTRL_REG1, &val) != kStatus_Success)
    {
        return kStatus_Fail;
    }
    val &= (uint8_t)(~(0x01));
    if (FXOS_WriteReg(handle, kFXOS8700_CTRL_REG1, val) != kStatus_Success)
    {
        return kStatus_Fail;
    }

    /* Set the range, -4g to 4g */
    if (FXOS_ReadReg(handle, kFXOS8700_XYZ_DATA_CFG, &val) != kStatus_Success)
    {
        return kStatus_Fail;
    }
    val &= (uint8_t)(~0x03);
    val |= 0x01;
    if (FXOS_WriteReg(handle, kFXOS8700_XYZ_DATA_CFG, val) != kStatus_Success)
    {
        return kStatus_Fail;
    }

    /* Set the F_MODE, disable FIFO */
    if (FXOS_ReadReg(handle, kFXOS8700_F_SETUP, &val) != kStatus_Success)
    {
        return kStatus_Fail;
    }
    val &= 0x3F;
    if (FXOS_WriteReg(handle, kFXOS8700_F_SETUP, val) != kStatus_Success)
    {
        return kStatus_Fail;
    }

    /* Put the fxos8700 into active mode */
    if (FXOS_ReadReg(handle, kFXOS8700_CTRL_REG1, &val) != kStatus_Success)
    {
        return kStatus_Fail;
    }
    val |= 0x01;
    val &= (uint8_t)(~0x02); /* set F_READ to 0 */
    if (FXOS_WriteReg(handle, kFXOS8700_CTRL_REG1, val) != kStatus_Success)
    {
        return kStatus_Fail;
    }

    return kStatus_Success;
}

status_t FXOS_ReadSensorData(fxos_handle_t *handle, fxos_data_t *data)
{
    uint8_t val = 0U;
    uint8_t ucStatus = 0;

    do
    {
        if (FXOS_ReadReg(handle, kFXOS8700_STATUS, &ucStatus) != kStatus_Success)
        {
            return kStatus_Fail;
        }
    } while (!(ucStatus & 0x08));

    /* Accel X axis */
    if (FXOS_ReadReg(handle, kFXOS8700_OUT_X_MSB, &val) != kStatus_Success)
    {
        return kStatus_Fail;
    }
    data->accelXMSB = val;
    if (FXOS_ReadReg(handle, kFXOS8700_OUT_X_LSB, &val) != kStatus_Success)
    {
        return kStatus_Fail;
    }
    data->accelXLSB = val;

    /* Accel Y axis */
    if (FXOS_ReadReg(handle, kFXOS8700_OUT_Y_MSB, &val) != kStatus_Success)
    {
        return kStatus_Fail;
    }
    data->accelYMSB = val;
    if (FXOS_ReadReg(handle, kFXOS8700_OUT_Y_LSB, &val) != kStatus_Success)
    {
        return kStatus_Fail;
    }
    data->accelYLSB = val;

    /* Accel Z axis */
    if (FXOS_ReadReg(handle, kFXOS8700_OUT_Z_MSB, &val) != kStatus_Success)
    {
        return kStatus_Fail;
    }
    data->accelZMSB = val;
    if (FXOS_ReadReg(handle, kFXOS8700_OUT_Z_LSB, &val) != kStatus_Success)
    {
        return kStatus_Fail;
    }
    data->accelZLSB = val;

    return kStatus_Success;
}

status_t FXOS_ReadReg(fxos_handle_t *handle, uint8_t reg, uint8_t *val)
{
    status_t status = kStatus_Success;

    /* Configure I2C xfer */
    handle->xfer.subaddress = (uint32_t)reg;
    handle->xfer.subaddressSize = 1U;
    handle->xfer.data = val;
    handle->xfer.dataSize = 1U;
#if defined(FSL_FEATURE_SOC_LPI2C_COUNT) && (FSL_FEATURE_SOC_LPI2C_COUNT)
    handle->xfer.direction = kLPI2C_Read;
    handle->xfer.flags = kLPI2C_TransferDefaultFlag;
#else
    handle->xfer.direction = kI2C_Read;
    handle->xfer.flags = kI2C_TransferDefaultFlag;
#endif

#if defined(FSL_FEATURE_SOC_LPI2C_COUNT) && (FSL_FEATURE_SOC_LPI2C_COUNT)
    if (LPI2C_MasterTransferNonBlocking(handle->base, handle->i2cHandle, &handle->xfer) != kStatus_Success)
    {
        return kStatus_Fail;
    }
    /*  wait for transfer completed. */
    while ((!g_nakFlag) && (!g_completionFlag))
    {
    }

    g_nakFlag = false;

    if (g_completionFlag == true)
    {
        g_completionFlag = false;
    }
    else
    {
        status = kStatus_Fail;
    }
#else
    status = I2C_MasterTransferBlocking(handle->base, &handle->xfer);
#endif

    return status;
}

status_t FXOS_WriteReg(fxos_handle_t *handle, uint8_t reg, uint8_t val)
{
    status_t status = kStatus_Success;
    uint8_t buff[1];

    buff[0] = val;
    /* Set I2C xfer structure */
    handle->xfer.subaddress = (uint32_t)reg;
    handle->xfer.subaddressSize = 1U;
    handle->xfer.data = buff;
    handle->xfer.dataSize = 1U;
#if defined(FSL_FEATURE_SOC_LPI2C_COUNT) && (FSL_FEATURE_SOC_LPI2C_COUNT)
    handle->xfer.direction = kLPI2C_Write;
    handle->xfer.flags = kLPI2C_TransferDefaultFlag;
#else
    handle->xfer.direction = kI2C_Write;
    handle->xfer.flags = kI2C_TransferDefaultFlag;
#endif

#if defined(FSL_FEATURE_SOC_LPI2C_COUNT) && (FSL_FEATURE_SOC_LPI2C_COUNT)
    if (LPI2C_MasterTransferNonBlocking(handle->base, handle->i2cHandle, &handle->xfer) != kStatus_Success)
    {
        return kStatus_Fail;
    }
    /*  wait for transfer completed. */
    while ((!g_nakFlag) && (!g_completionFlag))
    {
    }

    g_nakFlag = false;

    if (g_completionFlag == true)
    {
        g_completionFlag = false;
    }
    else
    {
        status = kStatus_Fail;
    }
#else
    status = I2C_MasterTransferBlocking(handle->base, &handle->xfer);
#endif

    return status;
}
