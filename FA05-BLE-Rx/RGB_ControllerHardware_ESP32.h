/*
 * SmartMatrix Library - Hardware-Specific Header File (for SmartMatrix Shield V4)
 *
 * Copyright (c) 2015 Louis Beaudoin (Pixelmatix)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

 // Note: only one MatrixHardware_*.h file should be included per project

#ifndef MATRIX_HARDWARE_H
#define MATRIX_HARDWARE_H
// formula used is 80000000L/(cfg->clkspeed_hz + 1), must result in >=2.  Acceptable values 26.67MHz, 20MHz, 16MHz, 13.34MHz...
#define ESP32_I2S_CLOCK_SPEED (20000000UL)

#define ESP32_FORUM_PINOUT              0
#define ESP32_FORUM_PINOUT_WITH_LATCH   1
#define SMARTLED_SHIELD_V0_PINOUT       2
#define ESP32_JC_RIBBON_PINOUT          3
#define HUB75_ADAPTER_PINOUT            4
#define HUB75_ADAPTER_LATCH_BREADBOARD_PINOUT            5
#define HUB75_ADAPTER_V0_THT_PINOUT     6
#define HUB75_ADAPTER_V0_SMT_PINOUT     7
#define ESP32_JC_RIBBON_PINOUT_WEMOS    8

#ifndef GPIOPINOUT
#define GPIOPINOUT CUSTOM
#endif
//#define GPIOPINOUT ESP32_FORUM_PINOUT_WITH_LATCH // note this mode is untested as of 2018-05-17 - not being used anymore now that SmartMatrix Shield is available
//#define GPIOPINOUT SMARTLED_SHIELD_V0_PINOUT

//Upper half RGB
#define BIT_R1  (1<<0)   
#define BIT_G1  (1<<1)   
#define BIT_B1  (1<<2)   
//Lower half RGB
#define BIT_R2  (1<<3)   
#define BIT_G2  (1<<4)   
#define BIT_B2  (1<<5)   

// Control Signals
#define BIT_LAT (1<<6) 
#define BIT_OE  (1<<7)  

#define BIT_A (1<<8)    
#define BIT_B (1<<9)    
#define BIT_C (1<<10)   
#define BIT_D (1<<11)   
#define BIT_E (1<<12)   

#endif
// This pinout takes a ribbon cable and flattens it, pin order is 1, 9, 2, 10 ...
// it connects to https://www.tindie.com/products/jasoncoon/16-output-nodemcu-esp32-wifi-ble-led-controller/
// *** WARNING, I cut the trace on Jason's board that went to pin 3, and patched a wire
// to pin 27 so that I can use RX/TX serial debugging ****
// That shield's pinout is this for the output of the level shifters:
// 23, 22, 27 (was 3), 21, 19, 18, 5, 17,    16, 4, 0, 2, 15, 14, 12, 13

    // ADDX is output directly using GPIO
    #define CLKS_DURING_LATCH   0 
    #define MATRIX_I2S_MODE I2S_PARALLEL_BITS_16
    #define MATRIX_DATA_STORAGE_TYPE uint16_t

    /*
    HUB 75 pinout
    01 02 B0
    03 04 Gnd
    05 06 G1
    07 08 E

    09 10 B
    11 12 D
    13 14 STB/Latch
    15 16 Gnd

                        ESP32 pin / comment
    1	R1	25	Red Data (columns 1-16)
    2	G1	26   	Green Data (columns 1-16)

    3	B1	27	(was 3) Blue Data (columns 1-16)
    4	GND	21/GND	Ground

    5	R2	14	Red Data (columns 17-32)
    6	G2	12 	Green Data (columns 17-32)

    7	B2	13    	Blue Data (columns 17-32)
    8	E	18	Demux Input E for 64x64 panels

    9	A	22	Demux Input A0
    10	B	19	Demux Input A1
    
    11	C	5/	Demux Input A2
    12	D	17 	Demux Input E1, E3 (32x32 panels only)

    13	CLK	16 	LED Drivers' Clock
    14	STB	4 	LED Drivers' Latch
    
    15	OE	15	LED Drivers' Output Enable
    16	GND	13/GND	Ground
    */ 
    #define R1_PIN  GPIO_NUM_25
    #define G1_PIN  GPIO_NUM_26
    #define B1_PIN  GPIO_NUM_27
    #define R2_PIN  GPIO_NUM_14
    #define G2_PIN  GPIO_NUM_12
    #define B2_PIN  GPIO_NUM_13

    #define A_PIN   GPIO_NUM_22
    #define B_PIN   GPIO_NUM_19
    #define C_PIN   GPIO_NUM_5
    #define D_PIN   GPIO_NUM_17
    #define E_PIN   GPIO_NUM_18

    #define CLK_PIN GPIO_NUM_16
    #define LAT_PIN GPIO_NUM_4
    #define OE_PIN  GPIO_NUM_15


    #define DEBUG_1_GPIO    GPIO_NUM_23