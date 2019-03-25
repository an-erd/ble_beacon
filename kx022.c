/**
 * Copyright (c) 2015 - 2018, Nordic Semiconductor ASA
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
 #include "kx022.h"

// configuration registers
uint8_t NRF_TWI_MNGR_BUFFER_LOC_IND KX022_cntl1_reg_addr  	= KX022_1020_REG_CNTL1;
uint8_t NRF_TWI_MNGR_BUFFER_LOC_IND KX022_cntl2_reg_addr  	= KX022_1020_REG_CNTL2;
uint8_t NRF_TWI_MNGR_BUFFER_LOC_IND KX022_cntl3_reg_addr  	= KX022_1020_REG_CNTL3;
uint8_t NRF_TWI_MNGR_BUFFER_LOC_IND KX022_odcntl_reg_addr 	= KX022_1020_REG_ODCNTL;
uint8_t NRF_TWI_MNGR_BUFFER_LOC_IND KX022_inc1_reg_addr   	= KX022_1020_REG_INC1;
uint8_t NRF_TWI_MNGR_BUFFER_LOC_IND KX022_inc2_reg_addr   	= KX022_1020_REG_INC2;
uint8_t NRF_TWI_MNGR_BUFFER_LOC_IND KX022_inc3_reg_addr   	= KX022_1020_REG_INC3;
uint8_t NRF_TWI_MNGR_BUFFER_LOC_IND KX022_inc4_reg_addr   	= KX022_1020_REG_INC4;
uint8_t NRF_TWI_MNGR_BUFFER_LOC_IND KX022_wufc_reg_addr   	= KX022_1020_REG_WUFC;


// data register (consecutive bytes will be read)
uint8_t NRF_TWI_MNGR_BUFFER_LOC_IND KX022_xout_reg_addr   	= KX022_1020_REG_XOUTL;	// read 6 bytes

// interrupt register
uint8_t NRF_TWI_MNGR_BUFFER_LOC_IND KX022_int_rel_reg_addr 	= KX022_1020_REG_INT_REL;	// read 5 byte
uint8_t NRF_TWI_MNGR_BUFFER_LOC_IND KX022_ins1_reg_addr   	= KX022_1020_REG_INS1;		// read 4 bytes (INS1..3, STATUS)

// first try full set of controls -> need power optimization/sleep not working
//static uint8_t config_1[2] = {KX022_1020_REG_CNTL1, 			0x00 }; 		// KX022_1020_STANDBY
//static uint8_t config_2[2] = {KX022_1020_REG_CNTL1, 			0x47 }; 		// KX022_1020_STANDBY | KX022_1020_HIGH_RESOLUTION | KX022_1020_DATA_READY_OFF | KX022_1020_RANGE_2G | KX022_1020_TAP_DETECT_ON | KX022_1020_WAKE_UP_ON | KX022_1020_TILT_POSITION_OFF
//static uint8_t config_3[2] = {KX022_1020_REG_ODCNTL, 			0x02 }; 		// KX022_1020_IIR_BYPASS_OFF | KX022_1020_LOW_PASS_FILTER_ODR_9 | KX022_1020_OUTPUT_RATE_50_HZ
//static uint8_t config_4[2] = {KX022_1020_REG_CNTL3, 			0x9E };			// Tilt ODR 12.5 Hz, directional tap ODR 400 Hz, Wake Up ODR 50 Hz
//static uint8_t config_5[2] = {KX022_1020_REG_WUFC, 				0x05 };			// WUFC(counts) =0.1 x 50 = 5counts
//static uint8_t config_6[2] = {KX022_1020_REG_ATH, 				0x08 };			// WAKEUP_THREHOLD (counts) =0.5 x 16 =8 counts
//static uint8_t config_7[2] = {KX022_1020_REG_TILT_TIMER, 	0x01 };			// 80ms timer(TiltOutput Data Rate is 12.5Hz)
//static uint8_t config_8[2] = {KX022_1020_REG_CNTL1, 	   	0xC7 }; 		// KX022_1020_OPERATE | KX022_1020_HIGH_RESOLUTION | KX022_1020_DATA_READY_OFF | KX022_1020_RANGE_2G | KX022_1020_TAP_DETECT_ON | KX022_1020_WAKE_UP_ON | KX022_1020_TILT_POSITION_OFF

//nrf_twi_mngr_transfer_t const kx022_init_transfers[KX022_INIT_TRANSFER_COUNT] =
//{
//    NRF_TWI_MNGR_WRITE(KX022_ADDR, config_1, 2, 0),
//		NRF_TWI_MNGR_WRITE(KX022_ADDR, config_2, 2, 0),
//		NRF_TWI_MNGR_WRITE(KX022_ADDR, config_3, 2, 0),
//		NRF_TWI_MNGR_WRITE(KX022_ADDR, config_4, 2, 0),
//		NRF_TWI_MNGR_WRITE(KX022_ADDR, config_5, 2, 0),
//		NRF_TWI_MNGR_WRITE(KX022_ADDR, config_6, 2, 0),
//		NRF_TWI_MNGR_WRITE(KX022_ADDR, config_7, 2, 0),
//		NRF_TWI_MNGR_WRITE(KX022_ADDR, config_8, 2, 0)
//};

// WIP optimization of power consumption
static uint8_t config_0[2] = {KX022_1020_REG_CNTL1, 				0x00 };	// KX022_1020_STANDBY 
static uint8_t config_1[2] = {KX022_1020_REG_CNTL1, 				0x40 };	// KX022_1020_STANDBY | KX022_1020_HIGH_RESOLUTION
static uint8_t config_2[2] = {KX022_1020_REG_ODCNTL, 			 	0x02 };	// KX022_1020_OUTPUT_RATE_50_HZ
static uint8_t config_3[2] = {KX022_1020_REG_CNTL1, 				0xC0 };	// KX022_1020_OPERATE | KX022_1020_HIGH_RESOLUTION
static uint8_t config_4[2] = {KX022_1020_REG_CNTL1, 				0xC0 };	// KX022_1020_OPERATE | KX022_1020_HIGH_RESOLUTION

//static uint8_t config_2[2] = {KX022_1020_REG_CNTL1, 			0x47 }; 		// KX022_1020_STANDBY | KX022_1020_HIGH_RESOLUTION | KX022_1020_DATA_READY_OFF | KX022_1020_RANGE_2G | KX022_1020_TAP_DETECT_ON | KX022_1020_WAKE_UP_ON | KX022_1020_TILT_POSITION_OFF
//static uint8_t config_4[2] = {KX022_1020_REG_CNTL3, 			0x9E };			// Tilt ODR 12.5 Hz, directional tap ODR 400 Hz, Wake Up ODR 50 Hz
//static uint8_t config_5[2] = {KX022_1020_REG_WUFC, 				0x05 };			// WUFC(counts) =0.1 x 50 = 5counts
//static uint8_t config_6[2] = {KX022_1020_REG_ATH, 				0x08 };			// WAKEUP_THREHOLD (counts) =0.5 x 16 =8 counts
//static uint8_t config_7[2] = {KX022_1020_REG_TILT_TIMER, 	0x01 };			// 80ms timer(TiltOutput Data Rate is 12.5Hz)
//static uint8_t config_8[2] = {KX022_1020_REG_CNTL1, 	   	0xC7 }; 		// KX022_1020_OPERATE | KX022_1020_HIGH_RESOLUTION | KX022_1020_DATA_READY_OFF | KX022_1020_RANGE_2G | KX022_1020_TAP_DETECT_ON | KX022_1020_WAKE_UP_ON | KX022_1020_TILT_POSITION_OFF

nrf_twi_mngr_transfer_t const kx022_init_transfers[KX022_INIT_TRANSFER_COUNT] =
{
    NRF_TWI_MNGR_WRITE(KX022_ADDR, config_1, 2, 0)
//		NRF_TWI_MNGR_WRITE(KX022_ADDR, config_2, 2, 0),
//		NRF_TWI_MNGR_WRITE(KX022_ADDR, config_3, 2, 0),
//		NRF_TWI_MNGR_WRITE(KX022_ADDR, config_4, 2, 0)
//		NRF_TWI_MNGR_WRITE(KX022_ADDR, config_5, 2, 0),
//		NRF_TWI_MNGR_WRITE(KX022_ADDR, config_6, 2, 0),
//		NRF_TWI_MNGR_WRITE(KX022_ADDR, config_7, 2, 0),
//		NRF_TWI_MNGR_WRITE(KX022_ADDR, config_8, 2, 0)
};
