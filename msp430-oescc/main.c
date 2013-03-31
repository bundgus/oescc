/*
 * main.c
 */

#include <msp430.h>
#include "sw_uart.h"
#include "utils.h"
#include "onewire.h"

extern unsigned char search_romcode[8]; // using global variable from onewire.c

void main(void) {
	WDTCTL = WDTPW + WDTHOLD; //stop watchdog
 
        // Set the clock divider DIVS_3 = /8
        // SMCLK = 1 MHz / 8 = 125 KHz
        BCSCTL2 |= DIVS_3;
  
       // use the low freqency VLO clock
       // The internal VLO has a nominal frequency of 12kHz, 
       // but may vary from as low as 4kHz to as high as 20kHz, 
       // with a drift of 0.5% per degree C and 4% per volt Vcc at 25°C.
       BCSCTL3 |= LFXT1S_2;
       
       /* Timer A Capture/Compare 0 */
       // PWM period = 12 KHz / 12000 = 1 Hz
       //65535 max
       TACCR0 = 12000;

      // Source Timer A from ACLK (TASSEL_1), up mode (MC_1).
      // Up mode counts up to TACCR0. SLAU144E p.12-20
      TACTL = TASSEL_1 | MC_1;

      // OUTMOD_7 = Reset/set output when the timer counts to TACCR1/TACCR0
      TACCTL1 = OUTMOD_7;  /* PWM output mode: 7 - PWM reset/set */
  
      // CCIE = Interrupt when timer counts to TACCR1
      TACCTL0 = CCIE; 
     // starting value for capture/control register
     TACCR1 = 0;  

     // Make P1.6 (green led) an output.
     P1DIR |= BIT6;
     // P1.6 = TA0.1 (timer A's output).
     // sets p1.6 to use the output of timer A.
     // this is a device specific feature
     P1SEL |= BIT6;  
     
     // interrupts enabled
     __bis_SR_register(GIE);
     //__bis_SR_register(CPUOFF | GIE);
       
 //  ---------------------      
        BCSCTL1 = CALBC1_16MHZ; //set clock
	DCOCTL  = CALDCO_16MHZ;

	//__disable_interrupt(); //we do not use any interrupts
	sw_uart_init();
	//leds_init();


	unsigned char presence;
	unsigned char search_successful;
	signed char bytecount;
	unsigned char scrachpad[9];
	signed int temperature;
	unsigned int whole;
	unsigned int fract;

	for(;;){

		sw_uart_wait_for_rx();

#ifdef VERBOSE_PRINT
		//led1_set_on();
		sw_uart_puts("Received start signal. Querying bus...");
		sw_uart_putnewline();
		//led1_set_off();
#endif // VERBOSE_PRINT

		//led2_set_on();
		presence = ow_reset();
		//led2_set_off();

		if(presence){
#ifdef VERBOSE_PRINT
			//led1_set_on();
			sw_uart_puts("Received presence pulse. Requesting temp. conversion");
			sw_uart_putnewline();
			//led1_set_off();
#endif // VERBOSE_PRINT
			//led2_set_on();
			ow_reset();
			ow_write_byte(OW_SKIPROM);
			ow_write_byte(OW_CONVERTTEMP);
			//led2_set_off();
#ifdef VERBOSE_PRINT
			//led1_set_on();
			sw_uart_puts("Awaiting conversion");
			sw_uart_putnewline();
			//led1_set_off();
#endif // VERBOSE_PRINT
			delay_miliseconds(750);

#ifdef VERBOSE_PRINT
			//led1_set_on();
			sw_uart_puts("Searching for DS18B20 devices and reading data");
			sw_uart_putnewline();
			//led1_set_off();
#endif //VERBOSE_PRINT


			//led2_set_on();
			ow_target_setup(OW_DS18B20_FAMILY_CODE); // Search for devices with family code 28h (DS18B20) only
			search_successful = ow_first();
			//led2_set_off();
			while (search_successful)
			{
				if (search_romcode[0] != OW_DS18B20_FAMILY_CODE){ //if family code is different we stop further searching.
					break;
				}

				// Communicating with 1-wire starts
				//led2_set_on();
				ow_reset();
				// selecting matching device
				ow_write_byte(OW_MATCHROM);
				for (bytecount = 0; bytecount <= 7; bytecount++){
					ow_write_byte(search_romcode[bytecount]);
				}
				// reading data from selected device
				ow_write_byte(OW_READSCRATCHPAD);
				for (bytecount = 0; bytecount <= 8; bytecount++){
					scrachpad[bytecount] = ow_read_byte();
				}
				ow_reset();

				//led2_set_off();
				//1-wire communication ends

				//converting data received from device to more usable form
			    temperature = (scrachpad[1]<<8) + scrachpad[0]; // temperature is 16bit
			    whole = abs(temperature)>>4;
			    fract = (abs(temperature)&0x000F)*625;

                            // convert C to F
                            //whole = (whole*10000+fract)*9/5;
                            whole = (whole * 10);
                            whole = whole + (fract/1000);
                            whole = whole * 9 / 5;
                            fract = whole % 10;
                            whole = (int)whole / 10;
                            whole = whole + 32;

			    //communication over uart starts
			    //led1_set_on();
				#ifdef VERBOSE_PRINT
					sw_uart_puts("Scrachpad: ");
					for(bytecount = 0; bytecount <=8; bytecount++){
					sw_uart_putbyte_hex(scrachpad[bytecount]);
				}
				sw_uart_putnewline();
				sw_uart_puts("Romcode: ");
				#endif // VERBOSE_PRINT

				//printing romcode as in DS18B20 datasheet (CRC on MSB)
				for (bytecount = 7; bytecount >= 0; bytecount--){
					sw_uart_putbyte_hex(search_romcode[bytecount]);
				}

				#ifdef VERBOSE_PRINT
				sw_uart_putnewline();
				sw_uart_puts("Temperature:");
				#endif // VERBOSE_PRINT


			    sw_uart_putc(' ');
			    if(temperature<0){
				sw_uart_putc('-');
			    }else{
				sw_uart_putc('+');
				}
			    sw_uart_putint_unsigned_dec(whole,'0',3);
			    //sw_uart_putint_signed_dec(temperature>>4,1,'0',0); // whole part of temperature measurement (signed)
			    sw_uart_putc('.');
			    sw_uart_putint_unsigned_dec(fract,'0',1); // fraction part of measurement (constant length of 4 digits as the resolution is 1/16 of a degree - 0.0625 step)
				sw_uart_putnewline();
				//led1_set_off();

				//led2_set_on();
			    search_successful = ow_next();
			    //led2_set_off();
			}
			sw_uart_putnewline(); // extra newline on end of transmission for more readable output



		}
#ifdef VERBOSE_PRINT
		else
		{
			//led1_set_on();
			sw_uart_puts("Presence pulse not detected.");
			sw_uart_putnewline();
			//led1_set_off();
		}
#endif //VERBOSE_PRINT

	}

        
}
	

unsigned int pos = 0;   // Index to PWM's duty cycle table (= brightness)

// This will be called when timer counts to TACCR1.
// Timer A0 interrupt service routine
#pragma vector=TIMER0_A0_VECTOR
__interrupt void Timer_A0 (void)
{
  if (pos < 12000) {
	pos += 1000;
	TACCR1 = pos;
  }
  else if (pos < 24000) {
	pos += 1000;
	TACCR1 = 24000 - pos;
  }
  else {
    pos = 0;
  }
  // Clear interrupt flag
  TACCTL1 &= ~CCIFG;
}