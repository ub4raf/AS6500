/* --COPYRIGHT--,BSD
 * Copyright (c) 2015, Texas Instruments Incorporated
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
 * --/COPYRIGHT--*/
/*******************************************************************************
 *
 * hal_LCD.c
 *
 * Hardware abstraction layer for the FH-1138P Segmented LCD
 * on MSP-EXP430FR6989
 *
 * February 2015
 * E. Chen
 *
 ******************************************************************************/

#include <driverlib.h>
#include "hal_LCD.h"
#include "string.h"
#include "delay.h"


// ***** Global Variables******************************************************
// Memory locations for LCD characters
const uint8_t SegmentLoc[ LCD_NUM_CHAR ][ 4 ] =
{
        { 18, 19, 20, 21 },                                                     // Position 1 = Digit A1
        { 10, 11, 12, 13 },                                                     // Position 2 = Digit A2
        {  6,  7,  8,  9 },                                                     // Position 3 = Digit A3
        { 36, 37, 38, 39 },                                                     // Position 4 = Digit A4
        { 28, 29, 30, 31 },                                                     // Position 5 = Digit A5
        { 14, 15, 16, 17 }                                                      // Position 6 = Digit A6
};

const uint8_t position_digit[6]=    {9,5,3,18,14,7};

const char ascii_font[224][2]={
   {0x00,0x00}, // ' '
   {0x04,0x40}, //"
   {0x33,0x10}, //#
   {0xB7,0x50}, //$
   {0x24, 0x28},  /* "%" */
   {0x00,0x00}, //TODO&
   {0x00,0x40}, //'
   {0x01,0x20}, //(
   {0x00,0x11}, //)
   {0x03, 0xFA},  /* "*" */
   {0x03, 0x05},  /* "+" */
   {0x00, 0x00}, //TODO ,
   {0x03, 0x00},  /* "-" */
   {0x00, 0x00}, //TODO .
   {0x00, 0x28},  /* "/" */
   {0xFC, 0x28},  /* "0" LCD segments a+b+c+d+e+f+k+q */
   {0x60, 0x20},  /* "1" */
   {0xDB, 0x00},  /* "2" */
   {0xF3, 0x00},  /* "3" */
   {0x67, 0x00},  /* "4" */
   {0xB7, 0x00},  /* "5" */
   {0xBF, 0x00},  /* "6" */
   {0xE4, 0x00},  /* "7" */
   {0xFF, 0x00},  /* "8" */
   {0xF7, 0x00},   /* "9" */
   {0x00,0x00}, // TODO:
   {0x00,0x00}, // TODO;
   {0x01,0x20}, //<
   {0x8F,0x00},  /* "=" *///TODO
   {0x00,0x11}, //>
   {0xC1,0x10}, //?
   {0x00,0x00},  // TODO@
   //  _"#$%&'()*+,-./0..9:;<=>?@
      {0xEF, 0x00},  /* "A" LCD segments a+b+c+e+f+g+m */
      {0xF1, 0x50},  /* "B" */
      {0x9C, 0x00},  /* "C" */
      {0xF0, 0x50},  /* "D" */
      {0x9F, 0x00},  /* "E" */
      {0x8F, 0x00},  /* "F" */
      {0xBD, 0x00},  /* "G" */
      {0x6F, 0x00},  /* "H" */
      {0x90, 0x50},  /* "I" */
      {0x78, 0x00},  /* "J" */
      {0x0E, 0x22},  /* "K" */
      {0x1C, 0x00},  /* "L" */
      {0x6C, 0xA0},  /* "M" */
      {0x6C, 0x82},  /* "N" */
      {0xFC, 0x00},  /* "O" */
      {0xCF, 0x00},  /* "P" */
      {0xFC, 0x02},  /* "Q" */
      {0xCF, 0x02},  /* "R" */
      {0xB7, 0x00},  /* "S" */
      {0x80, 0x50},  /* "T" */
      {0x7C, 0x00},  /* "U" */
      {0x0C, 0x28},  /* "V" */
      {0x6C, 0x0A},  /* "W" */
      {0x00, 0xAA},  /* "X" */
      {0x00, 0xB0},  /* "Y" */
      {0x90, 0x28},   /* "Z" */
   {0x00, 0x00},  /* TODO [ */
   {0x00, 0x82},  /* \ */
   {0x00, 0x00},  /* TODO ] */
   {0x04, 0x80},  /* ^ */
   {0x10, 0x00},  /* _ */
   {0x00, 0x40},  /* ' */
      {0xEF, 0x00},  /* "A" LCD segments a+b+c+e+f+g+m */
      {0xF1, 0x50},  /* "B" */
      {0x9C, 0x00},  /* "C" */
      {0xF0, 0x50},  /* "D" */
      {0x9F, 0x00},  /* "E" */
      {0x8F, 0x00},  /* "F" */
      {0xBD, 0x00},  /* "G" */
      {0x6F, 0x00},  /* "H" */
      {0x90, 0x50},  /* "I" */
      {0x78, 0x00},  /* "J" */
      {0x0E, 0x22},  /* "K" */
      {0x1C, 0x00},  /* "L" */
      {0x6C, 0xA0},  /* "M" */
      {0x6C, 0x82},  /* "N" */
      {0xFC, 0x00},  /* "O" */
      {0xCF, 0x00},  /* "P" */
      {0xFC, 0x02},  /* "Q" */
      {0xCF, 0x02},  /* "R" */
      {0xB7, 0x00},  /* "S" */
      {0x80, 0x50},  /* "T" */
      {0x7C, 0x00},  /* "U" */
      {0x0C, 0x28},  /* "V" */
      {0x6C, 0x0A},  /* "W" */
      {0x00, 0xAA},  /* "X" */
      {0x00, 0xB0},  /* "Y" */
      {0x90, 0x28},   /* "Z" */
   {0x02, 0x22},  /* { */
   {0x00, 0x50},  /* | */
   {0x01, 0x88},  /* } */
   {0x00, 0x00},  /* TODO ~ */
   {0x00, 0x00}  /* DEL */
};

// LCD memory map for numeric digits
const char digit[10][2] =
{
    {0xFC, 0x28},  /* "0" LCD segments a+b+c+d+e+f+k+q */
    {0x60, 0x20},  /* "1" */
    {0xDB, 0x00},  /* "2" */
    {0xF3, 0x00},  /* "3" */
    {0x67, 0x00},  /* "4" */
    {0xB7, 0x00},  /* "5" */
    {0xBF, 0x00},  /* "6" */
    {0xE4, 0x00},  /* "7" */
    {0xFF, 0x00},  /* "8" */
    {0xF7, 0x00}   /* "9" */
};

// LCD memory map for uppercase letters
const char alphabetBig[26][2] =
{
    {0xEF, 0x00},  /* "A" LCD segments a+b+c+e+f+g+m */
    {0xF1, 0x50},  /* "B" */
    {0x9C, 0x00},  /* "C" */
    {0xF0, 0x50},  /* "D" */
    {0x9F, 0x00},  /* "E" */
    {0x8F, 0x00},  /* "F" */
    {0xBD, 0x00},  /* "G" */
    {0x6F, 0x00},  /* "H" */
    {0x90, 0x50},  /* "I" */
    {0x78, 0x00},  /* "J" */
    {0x0E, 0x22},  /* "K" */
    {0x1C, 0x00},  /* "L" */
    {0x6C, 0xA0},  /* "M" */
    {0x6C, 0x82},  /* "N" */
    {0xFC, 0x00},  /* "O" */
    {0xCF, 0x00},  /* "P" */
    {0xFC, 0x02},  /* "Q" */
    {0xCF, 0x02},  /* "R" */
    {0xB7, 0x00},  /* "S" */
    {0x80, 0x50},  /* "T" */
    {0x7C, 0x00},  /* "U" */
    {0x0C, 0x28},  /* "V" */
    {0x6C, 0x0A},  /* "W" */
    {0x00, 0xAA},  /* "X" */
    {0x00, 0xB0},  /* "Y" */
    {0x90, 0x28}   /* "Z" */
};

// LCD memory map for uppercase letters
const char alphabetCyr[32][2] =
{
    {0xEF, 0x00},  /* "A" LCD segments a+b+c+e+f+g+m */
    {0xB1, 0x50},  /* "B" */
    {0xF1, 0x50},  /* "V" */
    {0x8C, 0x00},  /* "G" */
    {0x9F, 0x00},  /* "D" */
    {0x8F, 0x00},  /* "E" */
    {0xBD, 0x00},  /* "J" */
    {0x6F, 0x00},  /* "Z" */
    {0x90, 0x50},  /* "I" */
    {0x0E, 0x22},  /* "K" */
    {0x1C, 0x00},  /* "L" */
    {0x6C, 0xA0},  /* "M" */
    {0x6C, 0x82},  /* "N" */
    {0xFC, 0x00},  /* "O" */
    {0xCF, 0x00},  /* "P" */
    {0xFC, 0x02},  /* "R" */
    {0xB7, 0x00},  /* "S" */
    {0x80, 0x50},  /* "T" */
    {0x7C, 0x00},  /* "U" */
    {0x0C, 0x28},  /* "F" */
    {0x6C, 0x0A},  /* "Z" */
    {0x00, 0xAA},  /* "H" */
    {0x00, 0xB0},  /* "4" */
    {0x90, 0x28},  /* "Sh" */
    {0x0C, 0x28},  /* "Sh'" */
    {0x6C, 0x0A},  /* "b" */
    {0x00, 0xAA},  /* "b|" */
    {0x00, 0xAA},  /* "*b" */
    {0x00, 0xB0},  /* "e" */
    {0x90, 0x28},  /* "Yu" */
    {0x0C, 0x28}  /* "Ya" */
};

// LCD memory map for uppercase letters
const char alphabetSym[32][2] =
{
    {0x00,0x00}, // ' '
    {0x04,0x40}, //"
    {0x33,0x10}, //#
    {0xB7,0x50}, //$
    {0x24, 0x28},  /* "%" */
    {0x00,0x00}, //TODO&
    {0x00,0x40}, //'
    {0x01,0x20}, //(
    {0x00,0x11}, //)
    {0x03, 0xFA},  /* "*" */
    {0x03, 0x05},  /* "+" */
    {0x00, 0x00}, //TODO ,
    {0x03, 0x00},  /* "-" */
    {0x00, 0x00}, //TODO .
    {0x00, 0x28},  /* "/" */
    {0xFC, 0x28},  /* "0" LCD segments a+b+c+d+e+f+k+q */
    {0x60, 0x20},  /* "1" */
    {0xDB, 0x00},  /* "2" */
    {0xF3, 0x00},  /* "3" */
    {0x67, 0x00},  /* "4" */
    {0xB7, 0x00},  /* "5" */
    {0xBF, 0x00},  /* "6" */
    {0xE4, 0x00},  /* "7" */
    {0xFF, 0x00},  /* "8" */
    {0xF7, 0x00},   /* "9" */
    {0x00,0x00}, // TODO:
    {0x00,0x00}, // TODO;
    {0x01,0x20}, //<
    {0x8F,0x00},  /* "=" *///TODO
    {0x00,0x11}, //>
    {0xC1,0x10}, //?
    {0x00,0x00}  // TODO@

    //  _"#$%&'()*+,-./0..9:;<=>?@
};
// This structure is defined by the LCD_C DriverLib module, and is passed to the LCD_C_init() function

void Init_LCD(uint8_t blink_mode)
{

LCD_C_initParam initParams = {
    LCD_C_CLOCKSOURCE_ACLK,                                                     // Use ACLK as the LCD's clock source
    LCD_C_CLOCKDIVIDER_1,
    LCD_C_CLOCKPRESCALAR_16,
    MUX_RATE,                                                                // LaunchPad LCD specifies 4MUX mode
    LCD_C_LOW_POWER_WAVEFORMS,
    LCD_C_SEGMENTS_ENABLED
};
    LCD_C_off(LCD_C_BASE);

    // LCD Operation - VLCD generated internally, V2-V4 generated internally, v5 to ground

    /*  'FR6989 LaunchPad LCD1 uses Segments S4, S6-S21, S27-S31 and S35-S39 */
    LCD_C_setPinAsLCDFunctionEx(LCD_C_BASE, LCD_C_SEGMENT_LINE_4,
                                LCD_C_SEGMENT_LINE_4);
    LCD_C_setPinAsLCDFunctionEx(LCD_C_BASE, LCD_C_SEGMENT_LINE_6,
                                LCD_C_SEGMENT_LINE_21);
    LCD_C_setPinAsLCDFunctionEx(LCD_C_BASE, LCD_C_SEGMENT_LINE_27,
                                LCD_C_SEGMENT_LINE_31);
    LCD_C_setPinAsLCDFunctionEx(LCD_C_BASE, LCD_C_SEGMENT_LINE_35,
                                LCD_C_SEGMENT_LINE_39);
    LCD_C_init(LCD_C_BASE, &initParams);
    LCD_C_setVLCDSource(LCD_C_BASE, LCD_C_VLCD_GENERATED_INTERNALLY,
                        LCD_C_V2V3V4_GENERATED_INTERNALLY_NOT_SWITCHED_TO_PINS,
                        LCD_C_V5_VSS);

    // Set VLCD voltage to 3.20v
    LCD_C_setVLCDVoltage(LCD_C_BASE,
                         LCD_C_CHARGEPUMP_VOLTAGE_3_02V_OR_2_52VREF);

    // Enable charge pump and select internal reference for it
    LCD_C_enableChargePump(LCD_C_BASE);
    LCD_C_selectChargePumpReference(LCD_C_BASE,
                                    LCD_C_INTERNAL_REFERENCE_VOLTAGE);

    LCD_C_configChargePump(LCD_C_BASE, LCD_C_SYNCHRONIZATION_ENABLED, 0);

    // Clear LCD memory
    LCD_C_clearMemory(LCD_C_BASE);
    LCD_C_clearBlinkingMemory( LCD_C_BASE );

    // Select to display main LCD memory
    //LCD_C_selectDisplayMemory( LCD_C_BASE, LCD_C_DISPLAYSOURCE_MEMORY );

    // Turn blinking features off
    LCD_C_setBlinkingControl( LCD_C_BASE,LCD_C_BLINK_FREQ_CLOCK_DIVIDER_1,
                              //LCD_C_BLINK_FREQ_CLOCK_PRESCALAR_32768, //2fps
                              LCD_C_BLINK_FREQ_CLOCK_PRESCALAR_16384,   //4fps
                              blink_mode);

    //Turn LCD on
    LCD_C_on(LCD_C_BASE);
}

/*
 * Scrolls input string across LCD screen from left to right
 */
void displayScrollText(char *msg, uint8_t Memory)
{
    int length = strlen(msg);
    //int oldmode = mode;
    int i;
    int s = 5;
    char buffer[6] = "      ";
    for (i=0; i<length+7; i++)
    {
        //if (mode != oldmode)
        //    break;
        int t;
        for (t=0; t<6; t++)
            buffer[t] = ' ';
        int j;
        for (j=0; j<length; j++)
        {
            if (((s+j) >= 0) && ((s+j) < 6))
                buffer[s+j] = msg[j];
        }
        s--;
/*
        LCD_showChar(buffer[0], pos1, Memory );
        LCD_showChar(buffer[1], pos2, Memory );
        LCD_showChar(buffer[2], pos3, Memory );
        LCD_showChar(buffer[3], pos4, Memory );
        LCD_showChar(buffer[4], pos5, Memory );
        LCD_showChar(buffer[5], pos6, Memory );
*/
        showString(buffer, Memory);
        delay_ms(100);
        //__delay_cycles(200000);
    }
}

void showString(char *pStr, int Memory )
{
    if(pStr[0])showChar(pStr[0],0, Memory);
    if(pStr[1])showChar(pStr[1],1, Memory);
    if(pStr[2])showChar(pStr[2],2, Memory);
    if(pStr[3])showChar(pStr[3],3, Memory);
    if(pStr[4])showChar(pStr[4],4, Memory);
    if(pStr[5])showChar(pStr[5],5, Memory);
}
//*****************************************************************************
// TEST_LCD_showSymbol()
//
// Provides a single function which tests every mode of the LCD_showSymbol()
// function. You should be able to verify it's correct operation using with
// this function - along with use of breakpoints, single-stepping and a little
// manipulation of the LCD memory in the CCS Registers window.
//*****************************************************************************
void TEST_LCD_showSymbol( void )
{
    int i;                                                                      // Iterates thru Operations
    int j;                                                                      // Iterates thru Symbols
    int m;                                                                      // Iterates thru Memories
    volatile int r = -1;                                                        // Let's you view the return value (volatile needed to compiler won't optimize it away)

    for ( m = 0; m < 2; m++ )
    {
        if ( m == 1 )
        {
            LCD_C_selectDisplayMemory( LCD_C_BASE, LCD_C_DISPLAYSOURCE_BLINKINGMEMORY );
        }
        else
        {
            LCD_C_selectDisplayMemory( LCD_C_BASE, LCD_C_DISPLAYSOURCE_MEMORY );
        }

        for ( i = 0; i <= 3; i++ )
        {
            for ( j = 0; j <= LCD_A4COL; j++ )
            {
                r = LCD_showSymbol( i, j, m );
            }
        }
        //__delay_cycles(1000000);
        delay_ms(1500);
    }

    // Clear LCD memory and restore to display to main LCD memory
    LCD_C_selectDisplayMemory( LCD_C_BASE, LCD_C_DISPLAYSOURCE_MEMORY );
    LCD_C_clearMemory( LCD_C_BASE );
    LCD_C_clearBlinkingMemory( LCD_C_BASE );
}

void    blinkChar(uint8_t position)
    {

        LCDBMEM[position_digit[position]]   &= 0x00;
        LCDBMEM[position_digit[position]+1] &= 0x05;
        if( LCDMEM[position_digit[position]]||(LCDMEM[position_digit[position]+1]&0xFA) )
            {
                LCDBMEM[position_digit[position]+1] |=   LCDMEM[position_digit[position]+1]&0xFA;
                LCDBMEM[position_digit[position]]   |=   LCDMEM[position_digit[position]];
            }
        else
            {
                LCDBMEM[position_digit[position]]|=  digit[0][0];
                LCDMEM [position_digit[position]]|=  digit[0][0];
                LCDBMEM[position_digit[position]+1]|=digit[0][1];
                LCDMEM [position_digit[position]+1]|=digit[0][1];
            }
    }


/*
 * Displays input character at given LCD digit/position
 * Only spaces, numeric digits, and uppercase letters are accepted characters
 */
void showChar(char c, int position, int Memory )
{
    {
        // chars erase
        LCDMEM[position_digit[position]]    &= 0x00;
        LCDMEM[position_digit[position]+1]  &= 0x05;
        LCDBMEM[position_digit[position]]   &= 0x00;
        LCDBMEM[position_digit[position]+1] &= 0x05;
    }

    if(c > ' ')
    {
        if( (c <= '~') )
            {
            // Display alphabet
                    LCDMEM[position_digit[position]]   |= ascii_font[c-' '-1][0];
                    LCDMEM[position_digit[position]+1] |= ascii_font[c-' '-1][1];
            }
        else
            {
                LCDMEM[position_digit[position]]   |= alphabetCyr[(c-0xC0)%32][0];
                LCDMEM[position_digit[position]+1] |= alphabetCyr[(c-0xC0)%32][1];
            }
    }

    if(Memory==LCD_MEMORY_BLINKING)
    {
        if(c > ' ')
        {
                if( (c <= '~') )
                    {
                    // Display alphabet
                            LCDBMEM[position_digit[position]]   |= ascii_font[c-' '-1][0];
                            LCDBMEM[position_digit[position]+1] |= ascii_font[c-' '-1][1];
                    }
                else
                    {
                        LCDBMEM[position_digit[position]]   |= alphabetCyr[(c-0xC0)%32][0];
                        LCDBMEM[position_digit[position]+1] |= alphabetCyr[(c-0xC0)%32][1];
                    }
        }
    }

}

/*
 * Clears memories to all 6 digits on the LCD
 */
void clearLCD()
{
    LCDMEM[pos1] = LCDBMEM[pos1] = 0;
    LCDMEM[pos1+1] = LCDBMEM[pos1+1] = 0;
    LCDMEM[pos2] = LCDBMEM[pos2] = 0;
    LCDMEM[pos2+1] = LCDBMEM[pos2+1] = 0;
    LCDMEM[pos3] = LCDBMEM[pos3] = 0;
    LCDMEM[pos3+1] = LCDBMEM[pos3+1] = 0;
    LCDMEM[pos4] = LCDBMEM[pos4] = 0;
    LCDMEM[pos4+1] = LCDBMEM[pos4+1] = 0;
    LCDMEM[pos5] = LCDBMEM[pos5] = 0;
    LCDMEM[pos5+1] = LCDBMEM[pos5+1] = 0;
    LCDMEM[pos6] = LCDBMEM[pos6] = 0;
    LCDMEM[pos6+1] = LCDBMEM[pos6+1] = 0;

    LCDM14 = LCDBM14 = 0x00;
    LCDM18 = LCDBM18 = 0x00;
    LCDM3 = LCDBM3 = 0x00;
}

uint32_t    int_pow10   (uint8_t    power)
{
    switch  (power)
    {
    case    0:
        return 1;
    case    1:
        return  10;
    case    2:
        return  100;
    case    3:
        return  1000;
    case    4:
        return  10000;
    case    5:
        return  100000;
    case    6:
        return  1000000;
    default:
        {
            uint8_t res=1;
            while(power--)res*=10;
          return  res;
        }
    }
}

#include    "stdio.h"
void LCD_displayFloat(double val, int Memory )
{
    char output[8];
    double temp = val;
    uint8_t i;
    uint8_t coma=0;
    showString("      ", LCD_MEMORY_BLINKING );
    showString("      ", LCD_MEMORY_MAIN );
    if (val < 0)
        temp = (-val);

    if((temp<999999)&&(temp>0.001))                             //отображение чисел без научной нотации
        {
            if(temp>49999)
                LCD_displayNumber((signed long)temp, Memory,0);   //отображение целым чисел от 50 тысяч
            else
                {
                    if(temp>4999)
                        {
                            showString("    00", Memory );
                            showString("    00", LCD_MEMORY_MAIN );
                            LCD_displayNumber((signed long)(temp*10), Memory,0);
                            coma=5;
                        }
                    else
                    if(temp>499.9)
                        {
                            showString("   000", Memory );
                            showString("   000", LCD_MEMORY_MAIN );
                            LCD_displayNumber((signed long)(temp*100), Memory,0);
                            coma=4;
                        }
                    else
                    if(temp>49.99)
                        {
                            showString("  0000", Memory );
                            showString("  0000", LCD_MEMORY_MAIN );
                            LCD_displayNumber((signed long)(temp*1000), Memory,0);
                            coma=3;
                        }
                    else
                    if(temp>4.999)
                        {
                            showString(" 00000", Memory );
                            showString(" 00000", LCD_MEMORY_MAIN );
                            LCD_displayNumber((signed long)(temp*10000), Memory,0);
                            coma=2;
                        }
                    else
                    //if(temp>0.4999x,x)
                        {
                            //showString("000000", Memory );
                            //showString("000000", LCD_MEMORY_MAIN );
                            LCD_displayNumber((signed long)(temp*100000), Memory,1);
                            coma=1;
                        }
                }
        }
    else                        //отображение в научной нотации
        {
            //if((temp>int_pow10(9)||(temp<1))   //двухзначный порядок и минусовой:
            sprintf(output,"%.1e",temp);
            UART_puts(EUSCI_A1_BASE,(char*)output);
            UART_puts(EUSCI_A1_BASE,"==");
            output[1]=output[2];  //num
            output[2]=output[3];  //e
            output[3]=output[4];  //+-
            output[4]=output[5];  //y
            output[5]=output[6];  //x
            output[6]='\0';      // \n
            coma=5;
            showString(output,Memory);
            UART_puts(EUSCI_A1_BASE,(char*)output);
            UART_puts(EUSCI_A1_BASE,"\n");
        }

    for(i=LCD_A1DP;i<LCD_A5DP;i++)
        {
            LCD_showSymbol( LCD_CLEAR, i, LCD_MEMORY_BLINKING );
            LCD_showSymbol( LCD_CLEAR, i, LCD_MEMORY_MAIN );
        }
    if(coma)
        LCD_showSymbol( LCD_UPDATE, LCD_A1DP+coma-1, Memory);

    if (val < 0)
        LCD_showSymbol( LCD_UPDATE, LCD_NEG, Memory );
    else
        LCD_showSymbol( LCD_CLEAR, LCD_NEG, Memory );
}
//*****************************************************************************
// LCD_displayNumber()
//
// Displays a right-aligned number on the display. This function takes one
// input value and does not return a value.
//
// Arg 1: "val" is the integer to be displayed. It must be less than 6 digits
//        in length or this function writes "ERROR". The number is first
//        converted into character equivalants, then displayed.
// Arg 2: "Memory" Determines which LCD memory to act upon. '0'
//        represents the Main LCD memory registers; while '1' represents
//        the Blinking LCD memory registers. This allows a single
//        function to perform all the possible operations for a segment.
// Arg 3: Show zeros always or use usual number format
//
// This function does not provide any 'punctuation'. That is, decimals, signs
// or colons. You could add these using the LCD_showSymbol() function.
//*****************************************************************************
void LCD_displayNumber(signed long val, int Memory, bool zeros)
{
    unsigned long div[6];
    unsigned long mod[6];
    unsigned long err;

    if(!zeros)  {
        showString("      ", LCD_MEMORY_BLINKING );
        showString("      ", LCD_MEMORY_MAIN );
                }
    long temp = val;
    if (val < 0)
        temp = (-val);

    err = temp / 1000000;
    if ( err > 0 )
        showString(" ERROR", Memory);
    else
    {
        div[0] = temp / 100000  ;
        mod[0] = temp % 100000  ;
        div[1] = mod[0] / 10000;
        mod[1] = mod[0] % 10000;
        div[2] = mod[1] / 1000 ;
        mod[2] = mod[1] % 1000 ;
        div[3] = mod[2] / 100  ;
        mod[3] = mod[2] % 100  ;
        div[4] = mod[3] / 10   ;
        div[5] = mod[3] % 10   ;

        unsigned int i = 0;
        int LeadingZeros=0;

        for ( i = 0; i < ( LCD_NUM_CHAR - 1); i++ )
        {
            if ( (( div[i] == 0 ) && ( !LeadingZeros)) && ( !zeros ) )
            {
                showChar( ' ', i, Memory );
            }
            else
            {
                showChar( div[i] + '0', i, Memory );
                LeadingZeros++;
            }
        }

        i = LCD_NUM_CHAR - 1;
        showChar( div[i] + '0', i, Memory );
    }

    if (val < 0)
        {
            LCD_showSymbol( LCD_UPDATE, LCD_NEG, Memory );
            //LCD_showSymbol( LCD_CLEAR, LCD_NEG, Memory );
        }
    else
        LCD_showSymbol( LCD_CLEAR, LCD_NEG, Memory );
}

void clear_Symbols(void)
{
    // in registers clear only 2 bits, don't touch characters
    //  20,16,11,9,7,5
    LCDMEM[19]&=0xFA;
    LCDMEM[15]&=0xFA;
    LCDMEM[10]&=0xFA;
    LCDMEM[8]&=0xFA;
    LCDMEM[6]&=0xFA;
    LCDMEM[4]&=0xFA;
    LCDBMEM[19]&=0xFA;
    LCDBMEM[15]&=0xFA;
    LCDBMEM[10]&=0xFA;
    LCDBMEM[8]&=0xFA;
    LCDBMEM[6]&=0xFA;
    LCDBMEM[4]&=0xFA;
    //  18,14,
    LCDMEM[17] &=0x1F;
    LCDMEM[13] &=0x0F;
    LCDBMEM[17]&=0x1F;
    LCDBMEM[13]&=0x0F;
    //  3
    LCDMEM[2] &=0xF0;
    LCDBMEM[2]&=0xF0;
}

void clear_text_Symbols (void)
{
    // clear dots, cols, negative and degree symbols
    LCDMEM[19]&=0xFA;
    LCDMEM[15]&=0xFA;
    LCDMEM[10]&=0xFA;
    LCDMEM[6] &=0xFA;
    LCDMEM[4] &=0xFE;

    LCDBMEM[19]&=0xFA;
    LCDBMEM[15]&=0xFA;
    LCDBMEM[10]&=0xFA;
    LCDBMEM[6] &=0xFA;
    LCDBMEM[4] &=0xFE;
}

//*****************************************************************************
// LCD_showSymbol()
//
// The function shows (or modifies) the LCD memory bits to control the various
// special icons (i.e. non-character segments) of the display.
//
// This function has three arguments and returns either '0' or '1'.
//
//      Return - reflects the value of the icon's segment enable bit
//               at the end of the function
//      Arg 1  - 'Operation' specifies on of 4 different functions
//               that can be performed on an icon
//      Arg 2  - 'Symbol' indicates the special icon segment operated upon
//      Arg 3  - 'Memory' specifies which LCD memory (main or blinking)
//               the function should modify or test
//
// Operation:    The function can perform 4 different functions on each icon:
//
//      LCD_UPDATE - sets the segment bit to enable the icon
//      LCD_CLEAR  - clears the icon's segment bit
//      LCD_TOGGLE - toggles the icon's segment bit
//      LCD_GET    - returns the value of the icon's segment bit
//
//      Additionally, all operations return the icon's segment bit value.
//      For UPDATE, this should always be '1'; and for CLEAR, this should
//      be '0'; the others will vary based on the current value (GET) or
//      new value (TOGGLE).
//
//  Symbol:      The 'icon' value is used to select which LCD memory register
//               and bit to operate upon.
//
//  Memory:      Determines which LCD memory to act upon. '0'
//               represents the Main LCD memory registers; while '1' represents
//               the Blinking LCD memory registers. This allows a single
//               function to perform all the possible operations for a segment.
//
// The associated header file enumerates these operations as well as the various
// icons found on this LCD.
//*****************************************************************************
int LCD_showSymbol( int Operation, int Symbol, int Memory )
{
    int idx = 0;                                                                // Specifies the index of the LCD memory register (i.e. x in LCDMx)
    int bit = 0;                                                                // Specifies the charachters bit within the LCD memory register
    //int mem = 0;                                                                // Offset for LCD's "main" or "blinking" memory registers (either 0 or 32)
    int ret = -1;                                                               // Holds the function's return value

    // Which Memory has the user specified?  'Main' or 'Blinking'
    // Since 'Blinking' only applies to MUX4 (and lower) configurations, we ignore the Memory field if it does not apply.
    // This function cheats - instead of using LCDMx (for main) or LCDBMx (for blinking), we cheated and made use of the fact that the
    // two sets of LCD memory registers are aliased. Therefore, setting LCDBM3 also sets LCDM35.
    // Note, we made 'initParams' a global variable to make this test easier; though, we could have also tested the LCD register itself.
    //if (( MUX_RATE <= LCD_C_4_MUX ) && ( Memory == 1 ))
    //    mem = 35-3;                                                             // Writing to LCDBM35 also sets LCDM3 (and so on with the other values)

    // Select the LCDM memory register index (idx) and bit depending upon the provided Symbol value
    switch ( Symbol )
    {
    case LCD_TMR:
        idx = LCD_TIMER_IDX;
        bit = LCD_TIMER_COM;
        break;
    case LCD_HRT:
        idx = LCD_HEART_IDX;
        bit = LCD_HEART_COM;
        break;
    case LCD_REC:
        idx = LCD_REC_IDX;
        bit = LCD_REC_COM;
        break;
    case LCD_EXCLAMATION:
        idx = LCD_EXCLAMATION_IDX;
        bit = LCD_EXCLAMATION_COM;
        break;
    case LCD_BATT:
        idx = LCD_BATT_IDX;
        bit = LCD_BATT_COM;
        break;
    case LCD_BRACKETS:
        idx = LCD_BRACKETS_IDX;
        bit = LCD_BRACKETS_COM;
        break;
    case LCD_B6:
        idx = LCD_B6_IDX;
        bit = LCD_B6_COM;
        break;
    case LCD_B5:
        idx = LCD_B5_IDX;
        bit = LCD_B5_COM;
        break;
    case LCD_B4:
        idx = LCD_B4_IDX;
        bit = LCD_B4_COM;
        break;
    case LCD_B3:
        idx = LCD_B3_IDX;
        bit = LCD_B3_COM;
        break;
    case LCD_B2:
        idx = LCD_B2_IDX;
        bit = LCD_B2_COM;
        break;
    case LCD_B1:
        idx = LCD_B1_IDX;
        bit = LCD_B1_COM;
        break;
    case LCD_ANT:
        idx = LCD_ANT_IDX;
        bit = LCD_ANT_COM;
        break;
    case LCD_TX:
        idx = LCD_TX_IDX;
        bit = LCD_TX_COM;
        break;
    case LCD_RX:
        idx = LCD_RX_IDX;
        bit = LCD_RX_COM;
        break;
    case LCD_NEG:
        idx = LCD_NEG_IDX;
        bit = LCD_NEG_COM;
        break;
    case LCD_DEG:
        idx = LCD_DEG_IDX;
        bit = LCD_DEG_COM;
        break;
    case LCD_A1DP:
        idx = LCD_A1DP_IDX;
        bit = LCD_A1DP_COM;
        break;
    case LCD_A2DP:
        idx = LCD_A2DP_IDX;
        bit = LCD_A2DP_COM;
        break;
    case LCD_A3DP:
        idx = LCD_A3DP_IDX;
        bit = LCD_A3DP_COM;
        break;
    case LCD_A4DP:
        idx = LCD_A4DP_IDX;
        bit = LCD_A4DP_COM;
        break;
    case LCD_A5DP:
        idx = LCD_A5DP_IDX;
        bit = LCD_A5DP_COM;
        break;
    case LCD_A2COL:
        idx = LCD_A2COL_IDX;
        bit = LCD_A2COL_COM;
        break;
    case LCD_A4COL:
        idx = LCD_A4COL_IDX;
        bit = LCD_A4COL_COM;
        break;
    default:
        break;
    }


        {
            // This switch acts upon the correct icon segment based upon the Operation specified by the user
            switch ( Operation )
            {
            case LCD_UPDATE:
                LCDMEM[ idx - 1 ] |= bit;
                break;
            case LCD_CLEAR:
                LCDMEM[ idx - 1 ] &= ~bit;
                break;
            case LCD_TOGGLE:
                LCDMEM[ idx - 1 ] ^= bit;
                break;
            }
            // The LCD_GET operation is always performed; this is what is returned by the function
            if ( ( LCDMEM[ idx - 1 ] & bit ) >> 0 )
                ret = 1;
            else
                ret = 0;
        }
if(Memory==LCD_MEMORY_BLINKING)
        {
            // This switch acts upon the correct icon segment based upon the Operation specified by the user
            switch ( Operation )
            {
            case LCD_UPDATE:
                LCDBMEM[ idx - 1 ] |= bit;
                break;
            case LCD_CLEAR:
                LCDBMEM[ idx - 1 ] &= ~bit;
                break;
            case LCD_TOGGLE:
                LCDBMEM[ idx - 1 ] ^= bit;
                break;
            }
            // The LCD_GET operation is always performed; this is what is returned by the function
            if ( ( LCDBMEM[ idx - 1 ] & bit ) >> 0 )
                ret = 1;
            else
                ret = 0;
        }
else
        {
            // This switch acts upon the correct icon segment based upon the Operation specified by the user
               LCDBMEM[ idx - 1 ] &= ~bit;
        }

    return ( ret );
}


/* Display battery state of charge using the battery bar icon on the LCD */
void updateBatteryIcon(uint8_t percentage, uint8_t charging)
{

    LCD_showSymbol(LCD_CLEAR, LCD_BRACKETS, LCD_MEMORY_BLINKING);
    LCD_showSymbol(LCD_CLEAR, LCD_BATT, LCD_MEMORY_BLINKING);
    LCD_showSymbol(LCD_UPDATE, LCD_BRACKETS, LCD_MEMORY_MAIN);
    LCD_showSymbol(LCD_UPDATE, LCD_BATT, LCD_MEMORY_MAIN);

    // Calculate number of bars based on state of charge %
    int bar = percentage * 6 / 100;
    int i;

    for (i = 0; i < 6; i++)
    {

        LCD_showSymbol(LCD_CLEAR, LCD_B1+i, LCD_MEMORY_BLINKING);
        if (i < bar)
            LCD_showSymbol(LCD_UPDATE, LCD_B1+i, LCD_MEMORY_MAIN);// Draw bars corresponding to state of charge
        else
            LCD_showSymbol(LCD_CLEAR, LCD_B1+i, LCD_MEMORY_MAIN);// Clear battery indicator
    }
    // Blink extra bar if charging
    if (bar < 5 && charging)
        {
            LCD_showSymbol(LCD_UPDATE, LCD_B1+bar, LCD_MEMORY_BLINKING);
            //LCD_showSymbol(LCD_UPDATE, LCD_B1+bar, LCD_MEMORY_MAIN);
        }
}
//*****************************************************************************
// The LCD_C and LCD_E Driver Library modules vary in their implementations. In
// this case, it's not a difference in hardware, but with the fact that some of
// the routines, such as _setMemory(), were improved in the later LCD_E module.
//
// We created a new set of functions here for LCD_C that better match those
// found in the LCD_E implementation. Along with our "LCD" precursor, we also
// differentiated these functions by adding "Bits" to the end of the function
// names. For example, LCD_setMemoryBits().
/*****************************************************************************
void LCD_setMemoryBits( uint8_t LcdMemIndex , uint8_t SegmentMask )           // Set byte-wide LCD Memory with 8-bit SegmentMask
{
    LCDMEM[ LcdMemIndex - 1 ] = SegmentMask;                                    // 'Set' replaces the LCD memory value with the provided mask
}

void LCD_setBlinkingMemoryBits( uint8_t LcdMemIndex , uint8_t SegmentMask )   // Set byte-wide LCD Memory with 8-bit SegmentMask
{
    LCDBMEM[ LcdMemIndex - 1 ] = SegmentMask;                                   // 'Set' replaces the LCD memory value with the provided mask
}

void LCD_updateMemoryBits( uint8_t LcdMemIndex , uint8_t SegmentMask )        // Update LCD Memory with 8-bit SegmentMask
{
    LCDMEM[ LcdMemIndex - 1 ] |= SegmentMask;                                   // 'Update' OR's the 8-bit mask with the current LCD memory value
}

void LCD_updateBlinkingMemoryBits( uint8_t LcdMemIndex , uint8_t SegmentMask ) // Update LCD Memory with 8-bit SegmentMask
{
    LCDBMEM[ LcdMemIndex - 1 ] |= SegmentMask;                                  // 'Update' OR's the 8-bit mask with the current LCD memory value
}

void LCD_clearMemoryBits( uint8_t LcdMemIndex , uint8_t SegmentMask )         // Clear specified SegmentMask bits of LCD Memory
{
    LCDMEM[ LcdMemIndex - 1 ] &= ~SegmentMask;                                  // 'Clear' AND's the inverse of the provided 8-bit mask
}

void LCD_clearBlinkingMemoryBits( uint8_t LcdMemIndex , uint8_t SegmentMask ) // Clear specified SegmentMask bits of LCD Memory
{
    LCDBMEM[ LcdMemIndex - 1 ] &= ~SegmentMask;                                 // 'Clear' AND's the inverse of the provided 8-bit mask
}

void LCD_toggleMemoryBits( uint8_t LcdMemIndex , uint8_t SegmentMask )        // Toggle the bits specified by the 8-bit SegmentMask
{
    LCDMEM[ LcdMemIndex - 1 ] ^= SegmentMask;                                   // 'Toggle' exclusive-OR's the mask bits within the specified LCD memory location
}

void LCD_toggleBlinkingMemoryBits( uint8_t LcdMemIndex , uint8_t SegmentMask ) // Toggle the bits specified by the 8-bit SegmentMask
{
    LCDBMEM[ LcdMemIndex - 1 ] ^= SegmentMask;                                  // 'Toggle' exclusive-OR's the mask bits within the specified LCD memory location
}

uint8_t LCD_getMemoryBits( uint8_t LcdMemIndex )                              // Get the byte-wide LCD Memory location
{
    return( LCDMEM[ LcdMemIndex - 1 ] );                                        // 'Get' returns the current value of the specified LCD memory location
}

uint8_t LCD_getBlinkingMemoryBits( uint8_t LcdMemIndex )                      // Get the byte-wide LCD Memory location
{
    return( LCDBMEM[ LcdMemIndex - 1 ] );                                       // 'Get' returns the current value of the specified LCD memory location
}
*/
