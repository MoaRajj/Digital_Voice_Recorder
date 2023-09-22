////////////////////// main.c ///////////////////////
/*
 * Moayad Rajjoub
 * 171024078
 *
 */

#include "stm32g0xx.h"
#include "project3.h"
#include "time.h"
#include "stdio.h"

#define LEDDELAY 1600000U


int main(void) {

	BSP_System_init();
	
	init_adc() ;
   	init_timer1();
	init_I2C();
    
for(;;){

	Keypad_enable();

  }
    return 0;
}
