
#include <inttypes.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "util.h"
#include <stdlib.h>
#include "hc595.h"

// shiftout & display: lower half of PORTC
// BCD out to 7442: bits 4,5,6 of PORTC
// PORTB: input rotary 0-3
// PORTA: 7 return lines from key matrix
#define HC595_PORT   PORTC
#define HC595_DDR    DDRC

#define HC595_DS_POS PC0      //Data pin (DS) pin location
#define HC595_SH_CP_POS PC2      //Shift Clock (SH_CP) pin location 
#define HC595_ST_CP_POS PC1      //Store Clock (ST_CP) pin location


//Sends a clock pulse on SH_CP line
void HC595Pulse(){
  //Pulse the Shift Clock
  HC595_PORT|=(1<<HC595_SH_CP_POS);//HIGH
  //wait(1);
  HC595_PORT&=(~(1<<HC595_SH_CP_POS));//LOW
}

//Sends a clock pulse on ST_CP line
void HC595Latch(){
  //Pulse the Store Clock
  HC595_PORT|=(1<<HC595_ST_CP_POS);//HIGH
  //wait(1);
  HC595_PORT&=(~(1<<HC595_ST_CP_POS));//LOW
  //wait(1);
}

void HC595Write(uint8_t data){
  //Send each 8 bits serially
  uint8_t i;
  
  //Order is MSB first
  for(i=0;i<8;i++)
  {
    //Output the data on DS line according to the
    //Value of MSB
    if(data & 0b10000000)
    {
      //MSB is 1 so output high
      HC595_PORT|=(1<<HC595_DS_POS);
    }
    else
    {
      HC595_PORT&=(~(1<<HC595_DS_POS));
      //MSB is 0 so output high
    }
    //MH
    //wait(1);
    HC595Pulse();  //Pulse the Clock line
    data=data<<1;  //Now bring next bit at MSB position
    
  }
  
  //Now all 8 bits have been transferred to shift register
  //Move them to output latch at one
  // HC595Latch();
}
