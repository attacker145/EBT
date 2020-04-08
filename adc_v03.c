// Standard includes
#include <string.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>

// Driverlib includes
#include "utils.h"
#include "hw_memmap.h"
#include "hw_common_reg.h"
#include "hw_types.h"
#include "hw_adc.h"
#include "hw_ints.h"
#include "hw_gprcm.h"
#include "rom.h"
#include "rom_map.h"
#include "interrupt.h"
#include "prcm.h"
#include "uart.h"
#include "pinmux.h"
#include "pin.h"
#include "adc.h"
#include "common.h"

//#include "adc_userinput.h"
#include "uart_if.h"
#include "gpio.h"

//#define NO_OF_SAMPLES 		128
//#define NO_OF_SAMPLES       1
#define NO_OF_SAMPLES       4096
//#define NO_OF_SAMPLES       3072

extern void hexdec_long( uint32_t count );
extern char Rx_buf;
extern volatile tBoolean g_bFeedWatchdog;
extern void SendUART1_UC(unsigned char *str, unsigned char len);
extern void MessageUART1(const char *str);
extern void SendUART0_UC(unsigned char *str, unsigned char len);

unsigned long pulAdcSamples[4096];

void init_adc (void){

    /*
     * ADC_CH0 PIN57
     * ADC_CH1 PIN58
     * ADC_CH2 PIN59
     * ADC_CH3 PIN60 ****
     */

    MAP_PinTypeADC(PIN_60,PIN_MODE_255);// Pinmux for the selected ADC input pin

    MAP_ADCTimerConfig(ADC_BASE,2^17);// Configure ADC timer which is used to timestamp the ADC data samples

    MAP_ADCTimerEnable(ADC_BASE);// Enable ADC timer which is used to timestamp the ADC data samples

    MAP_ADCEnable(ADC_BASE);// Enable ADC module

    MAP_ADCChannelEnable(ADC_BASE, ADC_CH_3);// Enable ADC channel
}


float adc_read (uint64_t* run){

    unsigned int  uiIndex=0;
    float ADCsum;
    unsigned long ulSample;
#ifdef UART1_data
    unsigned char sendbyte[1];
#endif

    while(uiIndex < NO_OF_SAMPLES + 4)
    {
        if(MAP_ADCFIFOLvlGet(ADC_BASE, ADC_CH_3))
        {
            g_bFeedWatchdog = true;
            ulSample = MAP_ADCFIFORead(ADC_BASE, ADC_CH_3); //Fixed sampling interval of 16 µs per channel
            pulAdcSamples[uiIndex++] = ulSample;            //2048 * 0.000016 = 0.032768 s = 32.768 ms
            //MAP_UtilsDelay(160);//delay 0.00001s 11/6/2019
            *run += ulSample;
        }
    }
    *run = (*run / (NO_OF_SAMPLES + 4));
    uiIndex = 0;
#ifdef UART1_data
    while(uiIndex < NO_OF_SAMPLES) // Print out ADC samples UART1
    {
        g_bFeedWatchdog = true;

        sendbyte[0] = (unsigned char) ((pulAdcSamples[uiIndex] >> 8) & 0x00000000000000FF);//MSB
        SendUART1_UC(sendbyte, 1);
        sendbyte[0] = (unsigned char) (pulAdcSamples[uiIndex] & 0x00000000000000FF);//LSB little endian
        SendUART1_UC(sendbyte, 1);

        uiIndex++;
    }
#endif
    ADCsum = (float)(((((pulAdcSamples[4+uiIndex] >> 2 ) & 0x0FFF))*1.4)/4096);
    return ADCsum;
}






