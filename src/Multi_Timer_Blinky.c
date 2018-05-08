
/*
===============================================================================
 Name        : Multi_Timer_Blinky.c
 Author      : $(author)
 Version     :
 Copyright   : $(copyright)
 Description : main definition
===============================================================================
*/

#include "LPC8xx.h"
#include "core_cm0plus.h"
#include "Multi_Timer_Blinky.h"
#include "syscon.h"
#include "wkt.h"
#include "mrt.h"
#include "utilities.h"

// Program flow:
// 1. Turn on clocks to peripherals
// 2. Configure the WKT
//    A. Run from IRC/16 (this is the default clock)
//    B. Enable the WKT interrupt
// 3. Configure the SYSTICK
//    A. SYST_CSR(2) = 1 (clksource = system clock div-by-1) SYST_CSR(1) = 1 int enabled
//    B. SYST_RVR = value from header file
//    C. Clear the SYST_CVR register by writing to it
//    D. Enable the interrupt in the NVIC
// 4. Configure the MRT channels 0 and 1
//    A. Write to CTRL registers 1<<0 | 0<<1 (enable interrupt and Repeat mode)
//    B. Enable the MRT0 interrupt in the NVIC
// 5. Enter the main while(1) loop
//    A. Light an LED, start a timer, and wait for two interrupts
//       First one turns off the LED, 2nd one stops the timer and switches to the next LED.
//    B. Proceed to the next timer

#define UART0_DEV
#define I2C0_DEV
//#define BUTTON


volatile enum {false, true} handshake;
volatile uint8_t interrupt_counter;
volatile enum {red, blue, green} current_led;

void sct_pwm(int pwm_port,int token, int period, int mcu_port, int mcu_pin);
int sct_flag[24];
int tick_cnt[24];
int tick_token[24];
int tick_state[24];

int fr_cnt=0;


extern int uart_init(int uart_port);
extern int uart_decode(int uart_port);
extern int i2c_init(void);
extern int i2c_write(void);



void activate_current_led() {
  handshake = false;
  switch (current_led) {
    case(red)  : LED_On(LED_RED); break;
    case(blue) : LED_On(LED_BLUE); break;
    case(green): LED_On(LED_GREEN); break;
  }
}

void sct_pwm(int pwm_port,int token, int period, int mcu_port, int mcu_pin)
{



	  if(sct_flag[pwm_port]==1)
		{

			 if((tick_state[pwm_port]==0) && (tick_token[pwm_port]>0))
				 tick_state[pwm_port]=1;
			 else if((tick_state[pwm_port]==0) && (tick_token[pwm_port]==0))
				 tick_state[pwm_port]=2;
			 else if((tick_state[pwm_port]==1) && (tick_token[pwm_port]==0))
				  tick_state[pwm_port]=2;
			 else if((tick_state[pwm_port]==2) && (tick_cnt[pwm_port]==period))
				  tick_state[pwm_port]=0;

			 if(tick_state[pwm_port]==0)
			 {
				  //Chip_GPIO_PinSetState(LPC_GPIO_PORT,0, pin, 1);
				  GPIOSetBitValue( mcu_port, mcu_pin , 1 );
				  tick_token[pwm_port]=token;
			 }
			 else if(tick_state[pwm_port]==1)
			 {
				  //Chip_GPIO_PinSetState(LPC_GPIO_PORT,0, pin, 0);
				  GPIOSetBitValue(mcu_port, mcu_pin , 0 );
				  tick_token[pwm_port]--;
			 }
			 else if(tick_state[pwm_port]==2)
			 {
				  //Chip_GPIO_PinSetState(LPC_GPIO_PORT,0, pin, 1);
				  GPIOSetBitValue(mcu_port, mcu_pin , 1 );
			 }

			 if(tick_cnt[pwm_port]<period)
				 tick_cnt[pwm_port]++;
			 else
				 tick_cnt[pwm_port]=0;
			 sct_flag[pwm_port]=0;
			 //printf("pwm_port%d tick_state:%d tick_cnt:%d tick_token:%d\r\n",pwm_port,tick_state[pwm_port],tick_cnt[pwm_port],tick_token[pwm_port]);
		}
}


int main(void) {

  int i;
  // Configure the debug uart (see Serial.c)
  setup_debug_uart();

  // Enable clocks to relevant peripherals
  LPC_SYSCON->SYSAHBCLKCTRL0 |= (WKT|MRT|GPIO);

  // Configure the WKT ...
  // Give the  module a reset
  LPC_SYSCON->PRESETCTRL0 &= (WKT_RST_N);
  LPC_SYSCON->PRESETCTRL0 |= ~(WKT_RST_N);

  // Control register setup
  // WKT clock source is divided IRC (IRC/16)
  LPC_WKT->CTRL = (DIVIDED_IRC<<WKT_CLKSEL);

  // Enable the WKT interrupt in the NVIC
  NVIC_EnableIRQ(WKT_IRQn);

  // Configure the SYSTICK
  // clock = system_clock, tick interrupt enabled
  SysTick->CTRL = (1<<SysTick_CTRL_CLKSOURCE_Pos) | (1<<SysTick_CTRL_TICKINT_Pos);
  // Reload value
  SysTick->LOAD = SYSTICK_TIME;
  // Clear the counter and the countflag bit by writing any value to SysTick_VAL
  SysTick->VAL = 0;
  // Enable the SYSTICK interrupt in the NVIC
  NVIC_EnableIRQ(SysTick_IRQn);

  // Configure the MRT
  // Give the module a reset
  LPC_SYSCON->PRESETCTRL0 &= (MRT_RST_N);
  LPC_SYSCON->PRESETCTRL0 |= ~(MRT_RST_N);

  // Mode = repeat, interrupt = enable
  LPC_MRT->Channel[0].CTRL = (MRT_Repeat<<MRT_MODE) | (1<<MRT_INTEN);
  LPC_MRT->Channel[1].CTRL = (MRT_Repeat<<MRT_MODE) | (1<<MRT_INTEN);
  // Enable the MRT interrupt in the NVIC
  NVIC_EnableIRQ(MRT_IRQn);

  //Config_LED(LED_RED);
  //Config_LED(LED_BLUE);
  //Config_LED(LED_GREEN);

  Config_LED(P0_12);
  Config_LED(P1_15);
  Config_LED(P0_0);

  Config_Button(P0_4);

  // Initial values for current LED and interrupt counter
  current_led = red;
  interrupt_counter = 0;

  for(i=0;i<24;i++)
   {
    tick_cnt[i]=0;
    tick_token[i]=0;
    tick_state[i]=0;
   }


#ifdef UART0_DEV
  uart_init(0);
#endif

#ifdef UART1_DEV
  uart_init(1);
#endif

#ifdef I2C0_DEV
  i2c_init();
#endif


  while(1) {

    // Clear handshake flag, turn on current LED
    //activate_current_led();
    // Start the WKT
    //LPC_WKT->COUNT = WKT_TIME;
    // Wait until 2 interrupts are complete
    //while(!handshake);
    
    // Clear handshake flag, turn on current LED
    //activate_current_led();
    // Start the SYSTICK
	//SysTick->CTRL |= 1<<SysTick_CTRL_ENABLE_Pos;
    // Wait until 2 interrupts are complete
    //while(!handshake);

    // Clear handshake flag, turn on current LED
    //activate_current_led();
    // Start MRT channel 0
    //LPC_MRT->Channel[0].INTVAL = MRT0_TIME;
    // Wait until 2 interrupts are complete
    //while(!handshake);

    // Clear handshake flag, turn on current LED
    //activate_current_led();
    // Start MRT channel 1
    //LPC_MRT->Channel[1].INTVAL = MRT1_TIME;
    // Wait until 2 interrupts are complete
    //while(!handshake);

#ifdef UART0_DEV
	if(fr_cnt%1000==0)
	  	uart_decode(0);
#endif

#ifdef UART1_DEV
	if(fr_cnt%1000==110)
	  	uart_decode(1);
#endif

#ifdef I2C0_DEV
	if(fr_cnt%1000==200)
     i2c_write();
#endif

#ifdef BUTTON
	if(fr_cnt%1000==300)
	   printf("GPIO Value:%x \r\n", Get_Pin(P0_4));
#endif
    sct_pwm(2,64,128,0,0);
  	sct_pwm(1,32,128,1,15);
  	sct_pwm(0,16,128,0,12);
  	//SysTick->CTRL |= 1<<SysTick_CTRL_ENABLE_Pos;
  	//sct_pwm(3,16-1,128,17);
  	//sct_pwm(4,32-1,128,19);
  	//sct_pwm(5,64-1,128,21);
  	SysTick->CTRL |= 1<<SysTick_CTRL_ENABLE_Pos;

   if(fr_cnt<1000000)
     fr_cnt++;
   else
	   fr_cnt=0;


  } // end of while(1)
	
} // end of main
