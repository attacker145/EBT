
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
// SimpleLink includes
#include "simplelink.h"

// driverlib includes
#include "hw_ints.h"
#include "hw_types.h"
#include "rom.h"
#include "rom_map.h"
#include "prcm.h"
#include "utils.h"
#include "interrupt.h"
#include "adc.h"
#include "pin.h"

// common interface includes
#include "uart_if.h"
#include "uart.h"
#include "common.h"
#include "pinmux.h"
#include "i2c_if.h"
#include "hw_memmap.h"
#include "gpio.h"
//#include "serial_wifi.h" //not good
//#include "tmp006drv.h"
//#include "bma222drv.h"

#include "gpio_if.h"
#include "network_if.h"
// HTTP Client lib
#include <http/client/httpcli.h>
#include <http/client/common.h>
// JSON Parser
#include "jsmn.h"
#include <math.h>
#include "wdt.h"
#include "wdt_if.h"
#include "serial_wifi.h"

extern int ts_write_cmmnd (unsigned char commnd);
extern int ts_read_range(unsigned char *pucRegValue);
extern int ts1_write_cmmnd (unsigned char commnd);
extern int ts1_read_range(unsigned char *pucRegValue);
extern void hexdec_long( uint32_t count );
extern int AK9753AReadData(unsigned char *IR1L_, unsigned char *IR1H_,
                           unsigned char *IR2L_, unsigned char *IR2H_,
                           unsigned char *IR3L_, unsigned char *IR3H_,
                           unsigned char *IR4L_, unsigned char *IR4H_);
extern float getTemperature_AK9753(unsigned char *sgn);

extern unsigned char BLE_data_rdy;
extern unsigned int uiGPIOPort;//02/17/2017
extern unsigned char pucGPIOPin;//02/17/2017
extern unsigned char T_snr [2];
extern unsigned int T_snr_int;
extern unsigned char Lght;
extern uint8_t Rx_buf[10];
extern unsigned char snr_flg;
extern unsigned char snr_flg1;
extern char buf[300];
extern unsigned char door_flg;
extern unsigned char ucPinValue;
extern unsigned char flg;
extern unsigned char human;
extern unsigned int T_snr1_set;
extern unsigned int T_snr2_set;
extern unsigned char gaucOldMacDonald;
extern volatile tBoolean g_bFeedWatchdog;

#define alarm_on  0
#define alarm_off 1
#define SH_GPIO_9           9       // Red LED
#define SH_GPIO_12          12      /* PIN_03 - AK9753A INT */

//#define thrsh 300
void scan_IR(void){
    unsigned char sgn;
    unsigned char IR1L_ = 0;
    unsigned char IR1H_ = 0;
    unsigned char IR2L_ = 0;
    unsigned char IR2H_ = 0;
    unsigned char IR3L_ = 0;
    unsigned char IR3H_ = 0;
    unsigned char IR4L_ = 0;
    unsigned char IR4H_ = 0;
    unsigned int IR1 = 0;


    //unsigned char human = 0;
    float roomtemp;
    char rtbuf[10];
    //Read GPIO12: AK9753A INT
    GPIO_IF_GetPortNPin(SH_GPIO_12,&uiGPIOPort,&pucGPIOPin);    // Computes port and pin number from the GPIO number
    ucPinValue = GPIO_IF_Get(SH_GPIO_12,uiGPIOPort,pucGPIOPin); // Read pin status of GPIO22

    if(ucPinValue == 0){
        AK9753AReadData(&IR1L_, &IR1H_, &IR2L_, &IR2H_, &IR3L_, &IR3H_, &IR4L_, &IR4H_);    //* spiked
        roomtemp = getTemperature_AK9753(&sgn);
    }                                                                                       //*

    UART_PRINT("\n\rRoom Temperature is ");                                                 //*
    if (sgn == '+')                                                                         //*
        UART_PRINT("+%f\n\r", roomtemp);                                                    //*
    else                                                                                    //*
        UART_PRINT("-%f\n\r", roomtemp);                                                    //*

    IR1 = IR1H_;                                                                            //*
    IR1 = IR1 << 8;                                                                         //*
    IR1 = IR1 + (unsigned int)IR1L_;                                                        //*
    hexdec_long( (uint32_t) IR1 );                                                          //*
    UART_PRINT("\n\rIR1:");                                                                 //*
    UART_PRINT((const char*)Rx_buf);                                                        //*                                                                                             //*                                                                                             //*
    IR1 = IR2H_;                                                                            //*
    IR1 = IR1 << 8;                                                                         //*
    IR1 = IR1 + (unsigned int)IR2L_;                                                        //*
    hexdec_long( (uint32_t) IR1 );                                                          //*
    UART_PRINT("\n\rIR2:");                                                                 //*
    UART_PRINT((const char*)Rx_buf);                                                        //*
                                                                                            //*                                                                                            //*
    IR1 = IR3H_;                                                                            //*
    IR1 = IR1 << 8;                                                                         //*
    IR1 = IR1 + (unsigned int)IR3L_;                                                        //*
    hexdec_long( (uint32_t) IR1 );                                                          //*
    UART_PRINT("\n\rIR3:");                                                                 //*
    UART_PRINT((const char*)Rx_buf);                                                        //*
                                                                                            //*
    IR1 = IR4H_;                                                                            //*
    IR1 = IR1 << 8;                                                                         //*
    IR1 = IR1 + (unsigned int)IR4L_;                                                        //*

    hexdec_long( (uint32_t) IR1 );                                                          //*
    UART_PRINT("\n\rIR4:");                                                                 //*
    UART_PRINT((const char*)Rx_buf);                                                                     //*
    sprintf(rtbuf, "%.2f", roomtemp);
    hexdec_long( (uint32_t) T_snr_int );
}

float SignMag (unsigned int IR){
    float IRs;
    if (0x8000 & IR) {
        IRs = (-1)*(float)((0xFFFF - IR + 1));
    }else{
        IRs = IR;
    }
    return IRs;
}
