/*
 * Example_Multi_Timer_Blinky_ISR.c
 *
 *  Created on: Apr 8, 2016
 *      Author:
 */

#include "LPC8xx.h"
#include "wkt.h"
#include "mrt.h"
#include "Multi_Timer_Blinky.h"
#include "utilities.h"


extern volatile enum {false, true} handshake;
extern volatile uint8_t interrupt_counter;
extern volatile enum {red, blue, green} current_led;

extern int sct_flag[24];


void change_current_led() {
  handshake = true;
  interrupt_counter = 0;
  switch (current_led) {
    case(red)  : current_led = blue; break;
    case(blue) : current_led = green; break;
    case(green): current_led = red; break;
  }
}


void WKT_IRQHandler(void) {

  // Clear the interrupt flag
  LPC_WKT->CTRL |= 1<<WKT_ALARMFLAG;

  printf("WKT_IRQHandler has been called\r\n");
  // If interrupt_counter = 0, turn off LED and restart the timer
  if (interrupt_counter++ == 0) {
    LED_Off(LED_RED);
    LED_Off(LED_BLUE);
    LED_Off(LED_GREEN);
    LPC_WKT->COUNT = WKT_TIME;
    return;
  }

  // If interrupt_counter = 1, set the handshake flag for main, reset interrupt_counter, and point to next LED
  else {
    change_current_led();
  }
  return;
}




void SysTick_Handler(void) {

  int i;

  //printf("SysTick_Handler has been called\r\n");
  // Clear the interrupt flag by reading the SysTick_CTRL register
  uint32_t temp = SysTick->CTRL;

  // If interrupt_counter = 0, turn off LED and let the timer run
  if (interrupt_counter++ == 0) {
    LED_Off(LED_RED);
    LED_Off(LED_BLUE);
    LED_Off(LED_GREEN);
    return;
  }

  // If interrupt_counter = 1, stop the timer, set the handshake flag for main, reset interrupt_counter, and point to next LED
  else {
		SysTick->CTRL = temp & ~(1<<SysTick_CTRL_ENABLE_Pos);
    change_current_led();
  }

  for(i=0;i<24;i++)
    sct_flag[i]=1;
  return;
}




void MRT_IRQHandler(void) {
  uint32_t chan;
	

  printf("MRT_IRQHandler has been called\r\n");

  // Read the interrupt flags, decide which channel to service
  if (LPC_MRT->IRQ_FLAG & (1<<GFLAG0)) {
    chan = 0;
  }
  else {
    chan = 1;
  }

  // Clear the interrupt flag
  LPC_MRT->Channel[chan].STAT = 1<<MRT_INTFLAG;

  // If interrupt_counter = 0, turn off LED and let the timer run
  if (interrupt_counter++ == 0) {
    LED_Off(LED_RED);
    LED_Off(LED_BLUE);
    LED_Off(LED_GREEN);
    return;
  }

  // If interrupt_counter = 1, stop the timer, set the handshake flag for main, reset interrupt_counter, and point to next LED
  else {
    LPC_MRT->Channel[chan].INTVAL = ForceLoad | 0x00000000;
		//LPC_MRT->Channel[0].INTVAL = 0x80000000;
    change_current_led();
  }



  return;
}

