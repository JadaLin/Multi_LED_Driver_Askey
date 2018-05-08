${ProjName}:
Multi_Timer_Blinky


Intended purpose:
To demonstrate the configuration and use of the Self Wake-up Timer (WKT), the ARM Cortex
SysTick Timer (SysTick), and the Multi-Rate Timer (MRT).


Functional description:
* The WKT, SysTick, and two channels of the MRT are configured, and code enters the main while(1) loop.
* A timer is started, the currently referenced LED is lit, and the main code waits for two interrupts.
* On the first interrupt (in the ISR), the LED is turned off.
* On the second interrupt (in the ISR), the timer is stopped and the next LED is referenced.
* Main code proceeds to the next timer, and the process repeats.


External connections:
* None


Detailed Program Flow:
* This example runs at 30 MHz.
  See funtion SystemInit() and chip_setup.h.


* main() routine
   1. Turn on clocks to peripherals
   2. Configure the WKT
      A. Run from IRC/16 (this is the default clock)
      B. Enable the WKT interrupt
   3. Configure the SYSTICK
      A. SYST_CSR(2) = 1 (clksource = system clock div-by-1) SYST_CSR(1) = 1 int enabled
      B. SYST_RVR = value from header file
      C. Clear the SYST_CVR register by writing to it
      D. Enable the interrupt in the NVIC
   4. Configure the MRT channels 0 and 1
      A. Write to CTRL registers 1<<0 | 0<<1 (enable interrupt and Repeat mode)
      B. Enable the MRT0 interrupt in the NVIC
   5. Enter the main while(1) loop
      A. Light an LED, start a timer, and wait for two interrupts.
         First one turns off the LED, 2nd one stops the timer and switches to the next LED.
      B. Proceed to the next timer

  
This example runs from Flash. 


To run this code: 1. Build
                  2. Load to flash
                  3. Reset
              or:
                  1. Build
                  2. Download and Debug
                  3. Run or Go


Note: It may be necessary to power-cycle the board in order to regain control of the reset button after programming.





