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
 
#ifndef KX022_H__
#define KX022_H__

#ifdef __cplusplus
extern "C" {
#endif

//KX022-1020 register map
#define KX022_1020_REG_XHPL                           0x00
#define KX022_1020_REG_XHPH                           0x01
#define KX022_1020_REG_YHPL                           0x02
#define KX022_1020_REG_YHPH                           0x03
#define KX022_1020_REG_ZHPL                           0x04
#define KX022_1020_REG_ZHPH                           0x05
#define KX022_1020_REG_XOUTL                          0x06
#define KX022_1020_REG_XOUTH                          0x07
#define KX022_1020_REG_YOUTL                          0x08
#define KX022_1020_REG_YOUTH                          0x09
#define KX022_1020_REG_ZOUTL                          0x0A
#define KX022_1020_REG_ZOUTH                          0x0B
#define KX022_1020_REG_COTR                           0x0C
#define KX022_1020_REG_WHO_AM_I                       0x0F
#define KX022_1020_REG_TSCP                           0x10
#define KX022_1020_REG_TSPP                           0x11
#define KX022_1020_REG_INS1                           0x12
#define KX022_1020_REG_INS2                           0x13
#define KX022_1020_REG_INS3                           0x14
#define KX022_1020_REG_STAT                           0x15
#define KX022_1020_REG_INT_REL                        0x17
#define KX022_1020_REG_CNTL1                          0x18
#define KX022_1020_REG_CNTL2                          0x19
#define KX022_1020_REG_CNTL3                          0x1A
#define KX022_1020_REG_ODCNTL                         0x1B
#define KX022_1020_REG_INC1                           0x1C
#define KX022_1020_REG_INC2                           0x1D
#define KX022_1020_REG_INC3                           0x1E
#define KX022_1020_REG_INC4                           0x1F
#define KX022_1020_REG_INC5                           0x20
#define KX022_1020_REG_INC6                           0x21
#define KX022_1020_REG_TILT_TIMER                     0x22
#define KX022_1020_REG_WUFC                           0x23
#define KX022_1020_REG_TDTRC                          0x24
#define KX022_1020_REG_TDTC                           0x25
#define KX022_1020_REG_TTH                            0x26
#define KX022_1020_REG_TTL                            0x27
#define KX022_1020_REG_FTD                            0x28
#define KX022_1020_REG_STD                            0x29
#define KX022_1020_REG_TLT                            0x2A
#define KX022_1020_REG_TWS                            0x2B
#define KX022_1020_REG_ATH                            0x30
#define KX022_1020_REG_TILT_ANGLE_LL                  0x32
#define KX022_1020_REG_TILT_ANGLE_HL                  0x33
#define KX022_1020_REG_HYST_SET                       0x34
#define KX022_1020_REG_LP_CNTL                        0x35
#define KX022_1020_REG_BUF_CNTL1                      0x3A
#define KX022_1020_REG_BUF_CNTL2                      0x3B
#define KX022_1020_REG_BUF_STATUS_1                   0x3C
#define KX022_1020_REG_BUF_STATUS_2                   0x3D
#define KX022_1020_REG_BUF_CLEAR                      0x3E
#define KX022_1020_REG_BUF_READ                       0x3F
#define KX022_1020_REG_SELF_TEST                      0x60

//KX022-1020 default values
#define KX022_1020_DEVICE_ADDRESS_L                   0x1E
#define KX022_1020_DEVICE_ADDRESS_H                   0x1F
#define KX022_1020_WHO_AM_I                           0x14
#define INT_0                                         0x00
#define INT_1                                         0x01
#define INT_NONE                                      0xFF

//KX022-1020 settings
//KX022_1020_REG_CNTL1                                                MSB   LSB   DESCRIPTION
#define KX022_1020_STANDBY                            0b00000000  //  7     7     allows to change settings
#define KX022_1020_OPERATE                            0b10000000  //  7     7     start measuring; needs 1.2/KX022_1020_OUTPUT_RATE_n delay to load settings
#define KX022_1020_LOW_POWER                          0b00000000  //  6     6     low current draw, lower resolution
#define KX022_1020_HIGH_RESOLUTION                    0b01000000  //  6     6     high resolution, higher current draw
#define KX022_1020_DATA_READY_OFF                     0b00000000  //  5     5     disable new data interrupt
#define KX022_1020_DATA_READY_ON                      0b00100000  //  5     5     enable new data interrupt
#define KX022_1020_RANGE_2G                           0b00000000  //  4     3     set the output range to +-2g
#define KX022_1020_RANGE_4G                           0b00001000  //  4     3     set the output range to +-4g
#define KX022_1020_RANGE_8G                           0b00011000  //  4     3     set the output range to +-8g
#define KX022_1020_TAP_DETECT_OFF                     0b00000000  //  2     2     disable Directional Tap (TM) function
#define KX022_1020_TAP_DETECT_ON                      0b00000100  //  2     2     enable Directional Tap (TM) function
#define KX022_1020_WAKE_UP_OFF                        0b00000000  //  1     1     disable wake up on motion detect
#define KX022_1020_WAKE_UP_ON                         0b00000010  //  1     1     enable wake up on motion detect
#define KX022_1020_TILT_POSITION_OFF                  0b00000000  //  0     0     disable detection of orientation changes
#define KX022_1020_TILT_POSITION_ON                   0b00000001  //  0     0     enable detection of orientation changes
//KX022_1020_REG_ODCNTL
#define KX022_1020_IIR_BYPASS_OFF                     0b00000000  //  7     7     filtering applied
#define KX022_1020_IIR_BYPASS_ON                      0b10000000  //  7     7     filtering bypassed
#define KX022_1020_LOW_PASS_FILTER_ODR_9              0b00000000  //  6     6     low pass filter corner frequency: KX022_1020_OUTPUT_RATE_n/9
#define KX022_1020_LOW_PASS_FILTER_ODR_2              0b01000000  //  6     6                                       KX022_1020_OUTPUT_RATE_n/2
#define KX022_1020_OUTPUT_RATE_12_5_HZ                0b00000000  //  3     0     data output rate: 12.5 Hz
#define KX022_1020_OUTPUT_RATE_25_HZ                  0b00000001  //  3     0                       25 Hz
#define KX022_1020_OUTPUT_RATE_50_HZ                  0b00000010  //  3     0                       50 Hz
#define KX022_1020_OUTPUT_RATE_100_HZ                 0b00000011  //  3     0                       100 Hz
#define KX022_1020_OUTPUT_RATE_200_HZ                 0b00000100  //  3     0                       200 Hz
#define KX022_1020_OUTPUT_RATE_400_HZ                 0b00000101  //  3     0                       400 Hz    not available in KX022_LOW_POWER
#define KX022_1020_OUTPUT_RATE_800_HZ                 0b00000110  //  3     0                       800 Hz    not available in KX022_LOW_POWER
#define KX022_1020_OUTPUT_RATE_1600_HZ                0b00000111  //  3     0                       1600 Hz   not available in KX022_LOW_POWER
#define KX022_1020_OUTPUT_RATE_0_781_HZ               0b00001000  //  3     0                       0.781 Hz
#define KX022_1020_OUTPUT_RATE_1_563_HZ               0b00001001  //  3     0                       1.563 Hz
#define KX022_1020_OUTPUT_RATE_3_125_HZ               0b00001010  //  3     0                       3.125 Hz
#define KX022_1020_OUTPUT_RATE_6_25_HZ                0b00001011  //  3     0                       6.25 Hz
//KX022_1020_REG_INC1
#define KX022_1020_INT1_DISABLE                       0b00000000  //  5     5     INT output disabled
#define KX022_1020_INT1_ENABLE                        0b00100000  //  5     5     INT output enabled
#define KX022_1020_INT1_ACTIVE_LOW                    0b00000000  //  4     4     INT active low
#define KX022_1020_INT1_ACTIVE_HIGH                   0b00010000  //  4     4     INT active high
#define KX022_1020_INT1_LATCH_ON                      0b00000000  //  3     3     INT will latch until cleared by reading KX022_1020_REG_INT_REL
#define KX022_1020_INT1_LATCH_OFF                     0b00001000  //  3     3     INT will output a single 50 us pulse
//KX022_1020_REG_INC2
#define KX022_1020_INT1_X_NEG_OFF                     0b00000000  //  5     5     negative X value will not trigger interrupt
#define KX022_1020_INT1_X_NEG_ON                      0b00100000  //  5     5     negative X value will trigger interrupt
#define KX022_1020_INT1_X_POS_OFF                     0b00000000  //  4     4     positive X value will not trigger interrupt
#define KX022_1020_INT1_X_POS_ON                      0b00010000  //  4     4     positive X value will trigger interrupt
#define KX022_1020_INT1_Y_NEG_OFF                     0b00000000  //  3     3     negative Y value will not trigger interrupt
#define KX022_1020_INT1_Y_NEG_ON                      0b00001000  //  3     3     negative Y value will trigger interrupt
#define KX022_1020_INT1_Y_POS_OFF                     0b00000000  //  2     2     positive Y value will not trigger interrupt
#define KX022_1020_INT1_Y_POS_ON                      0b00000100  //  2     2     positive Y value will trigger interrupt
#define KX022_1020_INT1_Z_NEG_OFF                     0b00000000  //  1     1     negative Z value will not trigger interrupt
#define KX022_1020_INT1_Z_NEG_ON                      0b00000010  //  1     1     negative Z value will trigger interrupt
#define KX022_1020_INT1_Z_POS_OFF                     0b00000000  //  0     0     positive Z value will not trigger interrupt
#define KX022_1020_INT1_Z_POS_ON                      0b00000001  //  0     0     positive Z value will trigger interrupt
//KX022_1020_REG_INC4
#define KX022_1020_INT1_BUFFER_FULL_OFF               0b00000000  //  6     6     interrupt will not be triggered on full buffer
#define KX022_1020_INT1_BUFFER_FULL_ON                0b01000000  //  6     6     interrupt will be triggered on full buffer
#define KX022_1020_INT1_WATERMARK_OFF                 0b00000000  //  5     5     interrupt will not be triggered on watermark (filled FIFO)
#define KX022_1020_INT1_WATERMARK_ON                  0b00100000  //  5     5     interrupt will be triggered on watermark (filled FIFO)
#define KX022_1020_INT1_DATA_READY_OFF                0b00000000  //  4     4     interrupt will not be triggered when the measured data are ready
#define KX022_1020_INT1_DATA_READY_ON                 0b00010000  //  4     4     interrupt will be triggered when the measured data are ready
#define KX022_1020_INT1_TAP_OFF                       0b00000000  //  2     2     interrupt will not be triggered on tap/double tap detection
#define KX022_1020_INT1_TAP_ON                        0b00000100  //  2     2     interrupt will be triggered on tap/double tap detection
#define KX022_1020_INT1_MOTION_DETECT_OFF             0b00000000  //  1     1     interrupt will not be triggered on motion detection
#define KX022_1020_INT1_MOTION_DETECT_ON              0b00000010  //  1     1     interrupt will be triggered on motion detection
#define KX022_1020_INT1_TILT_OFF                      0b00000000  //  0     0     interrupt will not be triggered on tilt detection
#define KX022_1020_INT1_TILT_ON                       0b00000001  //  0     0     interrupt will be triggered on tilt detection

#define KX022_ADDR          ( 0x1E )

#define KX022_NUMBER_OF_REGISTERS 11

#define KX022_GET_ACC(acc_lsb, acc_msb) \
    (((int16_t)acc_msb << 8) | acc_lsb)

#ifdef __cplusplus
}
#endif

#endif // KX022_H__
