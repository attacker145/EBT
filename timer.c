
// Application Name     - Timer

// Standard include
#include <stdio.h>

// Driverlib includes
#include "hw_types.h"
#include "interrupt.h"
#include "hw_ints.h"
#include "hw_apps_rcm.h"
#include "hw_common_reg.h"
#include "prcm.h"
#include "rom.h"
#include "rom_map.h"
#include "hw_memmap.h"
#include "timer.h"
#include "utils.h"
#include "common.h"

// Common interface includes
#include "timer_if.h"
#include "gpio_if.h"

#include "pinmux.h"
#include "uart_if.h"



//*****************************************************************************
//                      MACRO DEFINITIONS
//*****************************************************************************
#define APPLICATION_VERSION        "1.1.1"
#define FOREVER                    1

//*****************************************************************************
//                      Global Variables for Vector Table
//*****************************************************************************
#if defined(ccs)
extern void (* const g_pfnVectors[])(void);
#endif
#if defined(ewarm)
extern uVectorEntry __vector_table;
#endif

//*****************************************************************************
//
// Globals used by the timer interrupt handler.
//
//*****************************************************************************
static volatile unsigned long g_ulSysTickValue;
static volatile unsigned long g_ulBase;
static volatile unsigned long g_ulRefBase;
static volatile unsigned long g_ulRefTimerInts = 0;
static volatile unsigned long g_ulIntClearVector;
extern volatile unsigned long g_ulTimerInts;
extern unsigned char door_flg;

extern unsigned char BLE_timeout;
extern unsigned char timeout;
extern unsigned char human;

void GPIO_IF_Toggle(unsigned char SH_GPIO_XX);

#define SH_GPIO_00          	0

//*****************************************************************************
//
//! The interrupt handler for the second timer interrupt.
//!
//! \param  None
//!
//! \return none
//
//*****************************************************************************
void TimerRefIntHandler(void)
{
    //
    // Clear the timer interrupt.
    //
    Timer_IF_InterruptClear(g_ulRefBase);

    g_ulRefTimerInts ++;
    GPIO_IF_LedToggle(MCU_RED_LED_GPIO);
}


//*****************************************************************************

//*****************************************************************************
//
//! The interrupt handler for the first timer interrupt.
//!
//! \param  None
//!
//! \return none
//
//*****************************************************************************
void TimerBaseIntHandler(void)
{

    //
    // Base address for first timer
    //
    g_ulBase = TIMERA0_BASE;

	//
    // Clear the timer interrupt.
    //
    Timer_IF_InterruptClear(g_ulBase);

    if ((timeout == 1) && (g_ulTimerInts==2)){
        //Report("Human is Reset");
        human = 0;
    }

    g_ulTimerInts ++;
    //GPIO_IF_LedToggle(MCU_RED_LED_GPIO);
    //if (g_ulTimerInts==2)
    	//GPIO_IF_Toggle(SH_GPIO_00);

    //BLE_timeout = 1;
    door_flg = 0;//reset door flag
    //human = 0;

    //else
    	//Report("Timer: BLE_timeout set to 0 \n\r");
}

//
//! main function demonstrates the use of the timers to generate
//! periodic interrupts.
//!
//! \param  None
//!
//! \return none
//
//*****************************************************************************
void timerA0_start(unsigned long ms)
{
    //
    // Base address for first timer
    //
    g_ulBase = TIMERA0_BASE;

    // Configuring the timers
    //
    Timer_IF_Init(PRCM_TIMERA0, g_ulBase, TIMER_CFG_PERIODIC, TIMER_A, 0);

    //
    // Setup the interrupts for the timer timeouts.
    //
    Timer_IF_IntSetup(g_ulBase, TIMER_A, TimerBaseIntHandler);

    //
    // Turn on the timers feeding values in mSec
    //
    Timer_IF_Start(g_ulBase, TIMER_A, ms);

    //Report("Timer is started \n\r");
}


void timerA0_stop(void)
{
    //
    // Base address for first timer
    //
    g_ulBase = TIMERA0_BASE;

    //
    // disable the timer
    //
    Timer_IF_Stop(g_ulBase, TIMER_A);

    //BLE_timeout = 0;
    //Report("timer STOP \n\r");
}
//*****************************************************************************
//
// Close the Doxygen group.
//! @}
//
//*****************************************************************************





