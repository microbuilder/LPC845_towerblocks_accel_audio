/*
 * Copyright (c) 2016, Freescale Semiconductor, Inc.
 * Copyright 2016-2018 NXP
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

/*	## Overview
 * 	-----------
 * 	The accel_audio application shows how to interface off-shelf components using SDK drivers
 * 	for I2C and the DAC. In this example the LPC845 Breakout board is interfaced with a
 * 	FXOS8700 accelerometer board. The accelerometer acts as an I2C slave device to the LPC845
 * 	master device. Colors on the on-board RGB LED should change when accelerometer position is
 * 	changed, and an audio alert will sound as motion is detected via the accelerometer.
 *
 *	## Toolchain Supported
 *	---------------------
 *	- MCUXpresso10.2.1
 *
 *	## Hardware Requirements
 *	------------------------
 *	- Micro USB cable
 *	- LPC845 board
 *	- FX0S8700 break-out board
 *	- External 2.2Kohm pull-up resistors (SCL SDA lines)
 *	- Personal Computer
 *
 *	## Board Settings
 *	------------------------
 *	Connect pins of I2C master and slave as below:
 *	MASTER_BOARD        CONNECTS TO         SLAVE_BOARD
 *	Pin Name   Board Location     Pin Name   Board Location
 *	SCL        CN1-23              SCL        J2-3
 *	SDA        CN1-22              SDA        J2-4
 *	VCC		   CN1-40              VDD/VDDIO  J1-3/4
 *	GND        CN1-20              GND        J1-5
 *	DAC0       CN1-2               A+ on the audio amp
 *
 */

/*******************************************************************************
 * Standard C Included Files
 ******************************************************************************/
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "board.h"
#include "fsl_fxos.h"
#include "fsl_i2c.h"
#include "fsl_debug_console.h"
#include "fsl_gpio.h"
#include "fsl_dma.h"
#include "fsl_dac.h"
#include "fsl_power.h"
#include "pin_mux.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
/* Accelerometer delta (degrees to register as a valid motion event) */
#define ACCEL_DELTA_DEGREES			(1.0)

/* LED port and pins */
#define BOARD_LED0_PORT 			(BOARD_INITPINS_GREEN_PORT)
#define BOARD_LED0_PIN 				(BOARD_INITPINS_GREEN_PIN)
#define BOARD_LED1_PORT 			(BOARD_INITPINS_BLUE_PORT)
#define BOARD_LED1_PIN 				(BOARD_INITPINS_BLUE_PIN)
#define BOARD_LED2_PORT 			(BOARD_INITPINS_RED_PORT)
#define BOARD_LED2_PIN 				(BOARD_INITPINS_RED_PIN)

/* I2C */
#define EXAMPLE_I2C_MASTER_BASE 	(I2C0_BASE)
#define I2C_MASTER_CLOCK_FREQUENCY 	(12000000)
#define EXAMPLE_I2C_MASTER 			((I2C_Type *)EXAMPLE_I2C_MASTER_BASE)
#define I2C_MASTER_SLAVE_ADDR_7BIT 	(0x7EU)
#define I2C_BAUDRATE 				(100000) 	/* 100 KHz */
#define I2C_DATA_LENGTH 			(33)  		/* MAX is 256 */

/* DAC (Audio output, 10-bit data)
 *
 * For audio output the sample rate should generally be one of:
 *
 *     FREQ:        30MHz:	Desc:
 *     --------		------	-----------------------------
 *   o 8000 Hz		3750	Telephones, etc.
 *   o 11025 Hz		2721	1/4 CD audio rate
 *   o 22050 Hz		1361	1/2 CD audio rate
 *   o 44100 Hz		 680	Audio CD, MPEG-1
 *   o 48000 Hz		 625	Professional video and audio
 *
 *   NOTE: Human speech is generally in the 100 Hz–4 kHz range,
 *   meaning a low 8kHz sample rate is usually sufficient if only
 *   human speech is present in the signal.
 *
 *   The LPC84x DMA peripheral supports single transfers up to
 *   1,024 words, so waveforms should be kept within this
 *   threshold for convenience sake.
 */
#define DEMO_DAC_BASE 				(DAC0)
#define DEMO_DMA_DAC_CHANNEL 		(22U)		/* DAC0_DMAREQ */
#define DEMO_DAC_DATA_REG_ADDR 		(0x40014000)
#define DEMO_DAC_COUNTER_VALUE 		(3750U)


/*******************************************************************************
 * Prototypes
 ******************************************************************************/
static void DMA_Configuration(void);

/*******************************************************************************
 * Variables
 ******************************************************************************/
dma_handle_t gDmaHandleStruct;

/* 8 kHz 'chirp' in 10-bit unsigned values. */
const uint32_t g_waveform[] = {
512, 684, 836, 948,1012,1012, 948, 832, 676, 500, 324, 172, 64,   12,  28, 104,
236, 400, 584, 756, 892, 980,1000, 952, 844, 688, 504, 324, 172,  64,  24,  56,
156, 308, 492, 676, 836, 944, 988, 956, 852, 696, 512, 328, 172,  68,  40,  88,
208, 376, 568, 748, 888, 964, 960, 880, 736, 552, 360, 192,  84,  56, 108, 232,
408, 600, 780, 908, 960, 924, 812, 644, 448, 264, 128,  68,  96, 204, 376, 572,
756, 892, 944, 912, 792, 616, 416, 240, 116,  80, 136, 272, 460, 656, 824, 920,
924, 836, 676, 476, 288, 148,  92, 132, 260, 448, 648, 816, 912, 908, 812, 640,
440, 256, 136, 108, 180, 332, 532, 724, 864, 912, 856, 712, 516, 320, 172, 116,
164, 304, 496, 696, 844, 900, 848, 708, 512, 316, 172, 128, 188, 340, 540, 732,
860, 884, 804, 636, 432, 252, 148, 152, 264, 448, 652, 808, 876, 832, 688, 492,
300, 172, 152, 244, 420, 624, 792, 868, 828, 684, 488, 296, 176, 168, 272, 456,
656, 808, 856, 792, 628, 428, 252, 168, 204, 344, 544, 728, 836, 828, 708, 516,
324, 200, 188, 292, 476, 672, 808, 832, 736, 556, 360, 220, 192, 280, 460, 656,
796, 824, 732, 552, 356, 224, 204, 304, 488, 680, 800, 808, 692, 504, 320, 212,
228, 360, 556, 728, 812, 764, 612, 420, 264, 212, 292, 460, 652, 780, 792, 680,
492, 316, 224, 260, 408, 600, 752, 792, 708, 532, 348, 240, 256, 388, 576, 732,
784, 708, 540, 356, 248, 264, 396, 584, 736, 776, 688, 516, 340, 248, 288, 436,
620, 748, 756, 644, 464, 308, 256, 332, 500, 672, 760, 720, 572, 392, 276, 284,
408, 588, 724, 748, 644, 472, 320, 272, 352, 520, 680, 748, 684, 528, 360, 280,
324, 472, 640, 736, 700, 560, 392, 292, 316, 452, 620, 724, 704, 572, 408, 300,
320, 448, 612, 716, 696, 564, 404, 304, 332, 464, 624, 712, 680, 540, 384, 308,
356, 500, 648, 708, 648, 500, 360, 316, 396, 548, 672, 696, 600, 444, 336, 340,
452, 604, 692, 660, 532, 388, 328, 388, 528, 656, 684, 596, 452, 348, 356, 468,
608, 680, 636, 504, 380, 344, 424, 564, 664, 656, 544, 412, 348, 400, 528, 644,
660, 572, 436, 356, 388, 508, 628, 660, 584, 452, 368, 388, 496, 616, 656, 584,
460, 376, 392, 496, 612, 648, 580, 460, 380, 400, 508, 612, 640, 564, 448, 384,
416, 524, 620, 628, 544, 432, 388, 440, 548, 624, 608, 516, 416, 396, 468, 572,
624, 580, 480, 408, 416, 504, 596, 612, 544, 448, 404, 452, 544, 608, 588, 500,
424, 420, 496, 580, 604, 544, 460, 416, 456, 544, 596, 576, 496, 432, 436, 508,
580, 588, 528, 452, 432, 480, 556, 588, 548, 476, 436, 464, 532, 580, 560, 496,
444, 456, 516, 568, 568, 512, 456, 452, 504, 560, 568, 520, 468, 456, 496, 548,
};

i2c_master_handle_t g_MasterHandle;
i2c_master_config_t masterConfig;

fxos_handle_t fxosHandle = {0};
fxos_data_t fxosSensorData = {0};

/* Delta tracking for accel data samples. */
struct vector_data {
	int16_t x;
	int16_t y;
	int16_t z;
};
struct vector_data g_accel_current = {0};
struct vector_data g_angle_last = {0};
struct vector_data g_angle_current = {0};

/* Counter for valid motion events */
static volatile uint32_t g_accel_events = 0;
static volatile bool g_dma_busy = false;

/* Define the init structure for the output LED pin */
gpio_pin_config_t led_config =
{
	kGPIO_DigitalOutput, 0,
};

status_t status = kStatus_Fail;
volatile bool g_MasterCompletionFlag = false;

/* FXOS8700 device address */
const uint8_t g_fxos_address = 0x1FU;
uint8_t g_accelGValue = 0;
uint8_t g_dataScale = 0;

/*******************************************************************************
 * Code
 ******************************************************************************/
/* Software ISR for DMA transfer done. */
void DEMO_DMA_Callback(dma_handle_t *handle, void *param, bool transferDone, uint32_t tcds)
{
	/* Let the main loop know that the DMA has finished pushing the last sample out to the DAC. */
	g_dma_busy = false;
}

/* I2C Accelerometer Interface Functions. */
void i2c_master_callback(I2C_Type *base, i2c_master_handle_t *handle, status_t status, void *userData)
{
    /* Signal transfer success when received success status. */
    if (status == kStatus_Success)
    {
        g_MasterCompletionFlag = true;
    }
}

void init_i2c_master()
{
	fxosHandle.base = BOARD_FXOS_I2C_BASEADDR;
	masterConfig.baudRate_Bps = I2C_BAUDRATE;
	fxosHandle.i2cHandle = &g_MasterHandle;
	fxosHandle.xfer.slaveAddress = g_fxos_address;
	fxosHandle.xfer.direction = kI2C_Write;

	/* Initialize and set default configuration: clock, freq for I2C master device */
	I2C_MasterGetDefaultConfig(&masterConfig);
	I2C_MasterInit(BOARD_FXOS_I2C_BASEADDR, &masterConfig, I2C_MASTER_CLOCK_FREQUENCY);
	I2C_MasterTransferCreateHandle(BOARD_FXOS_I2C_BASEADDR, &g_MasterHandle, i2c_master_callback, NULL);
}

/*******************************************************************************
 * FXOS8700 Initialization function
 ******************************************************************************/
int init_fxos(void)
{
	/* Initialize the FXOS8700:
	 *
	 * - Accelerometer
	 *   o Dynamic range : -4g to +4g
	 *   o FIFO : disabled
	 *   o F_READ : Normal Read mode
	 *
	 * - Magnetometer
	 *   o Dynamic range : TBD
	 */
	if (FXOS_Init(&fxosHandle) != kStatus_Success)
	{
		PRINTF("\n Failed FXOS_Init");
		return kStatus_Fail;
	}

	/* Get Accelerometer dynamic range */
	if (FXOS_ReadReg(&fxosHandle, kFXOS8700_XYZ_DATA_CFG, &g_accelGValue) != kStatus_Success)
	{
		PRINTF("\n Failed FXOS_ReadReg");
		return kStatus_Fail;
	}

	return kStatus_Success;
}

/*******************************************************************************
 * FXOS8700 value reading function
 ******************************************************************************/
int fxos_read_value(void)
{
	if (FXOS_ReadSensorData(&fxosHandle, &fxosSensorData) != kStatus_Success)
	{
		PRINTF("\n\r Read data failed");
		return kStatus_Fail;
	}

	return kStatus_Success;
}

/*******************************************************************************
 * DMA config function
 ******************************************************************************/
static void DMA_Configuration(void)
{
    dma_transfer_config_t dmaTransferConfigStruct;

    /* Configure DMA. */
    DMA_Init(DMA0);
    DMA_EnableChannel(DMA0, DEMO_DMA_DAC_CHANNEL);
    DMA_CreateHandle(&gDmaHandleStruct, DMA0, DEMO_DMA_DAC_CHANNEL);
    DMA_SetCallback(&gDmaHandleStruct, DEMO_DMA_Callback, NULL);

    /* Prepare and submit the transfer. */
    DMA_PrepareTransfer(&dmaTransferConfigStruct,       /* To keep the configuration. */
                        (void *)g_waveform,             /* DMA transfer source address. */
                        (void *)DEMO_DAC_DATA_REG_ADDR, /* DMA transfer destination address. */
                        sizeof(uint32_t),               /* DMA transfer destination address width(bytes). */
                        sizeof(g_waveform),             /* DMA transfer bytes to be transferred. */
                        kDMA_MemoryToPeripheral,        /* DMA transfer type. */
                        NULL                            /* nextDesc Chain custom descriptor to transfer. */
                        );
    DMA_SubmitTransfer(&gDmaHandleStruct, &dmaTransferConfigStruct);
    DMA_StartTransfer(&gDmaHandleStruct);
}

/*******************************************************************************
 * Main function
 ******************************************************************************/
int main(void)
{
    dac_config_t dacConfigStruct;

    /* Initialize Board pins, clock and debug console unit*/
    BOARD_InitPins();
    BOARD_BootClockRUN();
    BOARD_InitDebugConsole();

    /* Power up DAC0 and enable the DMA IRQ.*/
    POWER_DisablePD(kPDRUNCFG_PD_DAC0);
    NVIC_EnableIRQ(DMA0_IRQn);
    DMA_Configuration();

    /* Init DAC0, enable DMA transfers, set DAC0 frequency. */
    DAC_GetDefaultConfig(&dacConfigStruct);
    DAC_Init(DEMO_DAC_BASE, &dacConfigStruct);
    DAC_EnableDMA(DEMO_DAC_BASE, true);
    DAC_SetCounterValue(DEMO_DAC_BASE, DEMO_DAC_COUNTER_VALUE);
    DAC_EnableDoubleBuffering(DEMO_DAC_BASE, true);

    /* I2C master and FXOS accel initialization */
    init_i2c_master();
    status = init_fxos();

    /* Check accelerometer FS bits[1:0] for dynamic range value :
     *   o +/-2g = 0x00
     *   o +/-4g = 0x01	(default)
     *   o +/-8g = 0x02
     */
    switch (g_accelGValue & 0x03)
	{
		case 0x00: g_dataScale = 2;
				   break;
		case 0x01: g_dataScale = 4;
				   break;
		case 0x02: g_dataScale = 8;
				   break;
	}

    /* Init output LED GPIO Ports. */
    GPIO_PortInit(GPIO, BOARD_LED0_PORT);
    GPIO_PortInit(GPIO, BOARD_LED1_PORT);
    GPIO_PortInit(GPIO, BOARD_LED2_PORT);

    /* Init output LED GPIO Pins. */
    GPIO_PinInit(GPIO, BOARD_LED0_PORT, BOARD_LED0_PIN, &led_config);
    GPIO_PinInit(GPIO, BOARD_LED1_PORT, BOARD_LED1_PIN, &led_config);
    GPIO_PinInit(GPIO, BOARD_LED2_PORT, BOARD_LED2_PIN, &led_config);

    /* Constantly poll for motion on the accelerometer, and trigger playback of
     * an audio sample if the motion threshold is crossed. */
    while(1)
	{
		/* Read Accelerometer Sensor Data */
		status = fxos_read_value();

		/* Save sensor data as a 16 bit result.
		 * The 16 bit result is in left-justified format.
		 * Shift 4 bits to the right (12-bit resolution) to get the actual value.
		 */
		g_accel_current.x =
			(int16_t)(((uint16_t)(fxosSensorData.accelXMSB << 8)) | ((uint16_t)(fxosSensorData.accelXLSB))) / 16U;
		g_accel_current.y =
			(int16_t)(((uint16_t)(fxosSensorData.accelYMSB << 8)) | ((uint16_t)(fxosSensorData.accelYLSB))) / 16U;
		g_accel_current.z =
			(int16_t)(((uint16_t)(fxosSensorData.accelZMSB << 8)) | ((uint16_t)(fxosSensorData.accelZLSB))) / 16U;

		/* Convert raw accelerometer sensor data to angle (normalize to 0-90 degrees). No negative angles. */
		g_angle_current.x = (int16_t)floor((double)g_accel_current.x * (double)g_dataScale * 180.0 / 4096.0);
		g_angle_current.y = (int16_t)floor((double)g_accel_current.y * (double)g_dataScale * 180.0 / 4096.0);

		/* Add a motion event 'tick' if we passed the ACCEL_DELTA_DEGREES threshold on either axis. */
		if (floor((double)g_angle_current.x - (double)g_angle_last.x) > ACCEL_DELTA_DEGREES) {
			g_accel_events++;
		}
		if (floor((double)g_angle_current.y - (double)g_angle_last.y) > ACCEL_DELTA_DEGREES) {
			g_accel_events++;
		}

		/* Reset the delta tracking variables. */
		if (g_accel_events) {
			g_angle_last.x = g_angle_current.x;
			g_angle_last.y = g_angle_current.y;
		}

		/* Trigger an appropriate number of 'blips' on the speaker */
		while(g_accel_events) {
			/* Turn the green LED ON */
			GPIO_PinWrite(GPIO, BOARD_LED0_PORT, BOARD_LED0_PIN, 0);
			GPIO_PinWrite(GPIO, BOARD_LED1_PORT, BOARD_LED1_PIN, 1);
			GPIO_PinWrite(GPIO, BOARD_LED2_PORT, BOARD_LED2_PIN, 1);
			/* Start a new DMA transfer. */
			g_dma_busy = true;
	 	    DMA_StartTransfer(&gDmaHandleStruct);
		    /* Decrement the DMA event counter. */
			g_accel_events--;
			/* Wait until DMA is done (audio playback complete) and toggle LEDs */
			while(g_dma_busy) { }
			GPIO_PinWrite(GPIO, BOARD_LED0_PORT, BOARD_LED0_PIN, 1);
			GPIO_PinWrite(GPIO, BOARD_LED1_PORT, BOARD_LED1_PIN, 0);
			GPIO_PinWrite(GPIO, BOARD_LED2_PORT, BOARD_LED2_PIN, 0);
		}
	 }
}
