
// Hardware panel for Flightgear
// test programm for display, rotary encoders, key matrix
// ATMEGA32

/************************************************************************/
/*  credits:                                                            */
/*                      Debouncing 8 Keys                               */
/*                      Sampling 4 Times                                */
/*                      With Repeat Function                            */
/*                                                                      */
/*              Author: Peter Dannegger                                 */
/*                      danni@specs.de                                  */
/*                                                                      */
/*   BenBuxton: state machine for rotary encoders                       */
/*  http://www.buxtronix.net/2011/10/rotary-encoders-done-properly.html */ 
/*                                                                      */
/*  http://www.mikrocontroller.net/articles/AVR-GCC-Tutorial/Der_UART   */                                                       
/*                                                                      */
/*                                                                      */
/************************************************************************/

#include <inttypes.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "util.h"
#include <stdlib.h>

#ifndef F_CPU
#define F_CPU           16000000                   // processor clock frequency
#warning kein F_CPU definiert
#endif

#define KEY_PORT        kb4
#define KEY_PIN         kb4
#define KEY_PIN_R	kb0
#define KEY_PORT_R	kb0
#define KEY_PIN_P	kb5
#define KEY_PORT_P	kb5
// Rafi Tasten
#define KEY0            0
#define KEY1            1
#define KEY2            2
#define KEY3            3
#define KEY4            4
#define KEY5            5
#define KEY6            6

#define ALL_KEYS	255
//#define REPEAT_MASK     (1<<KEY1 | 1<<KEY2)       // repeat: key1, key2
#define REPEAT_MASK 0
#define REPEAT_START    50                        // after 500ms
#define REPEAT_NEXT     20                        // every 200ms

#define LED_DDR         DDRA
#define LED_PORT        outword[1]
#define LED_PORT_R      outword[2]

//#define LED0            0      // grüne LED 0 gibts nicht
// grüne LEDs
#define LED1            1
#define LED2            2
#define LED3            3
#define LED4            4
#define LED5            5
#define LED6            6
#define LED7            7

// Com Modes
#define COARSE          1
#define FINE            2

// keybyte[0]  efis Tasten
// keybyte[1]  3+2 Schalter
// keybyte[2]  6-position Schalter "ROSE"
// keybyte[3]  7-position Schalter "zoom"
// keybyte[4]  Rafi Taster
// keybyte[5]  rotary Taster
// keybyte[6]  7-position Schalter "COM/NAV"
// keybyte[7]  2 stueck 3-position Schalter

volatile uint8_t keybyte[8]    ={0,0,0,0,0,0,0,0};
volatile uint8_t keybyte_old[8]={0,0,0,0,0,0,0,0};

volatile int32_t time, time_old;
volatile uint8_t kb4, kb0,kb5, changes;

//green LEDs
volatile uint8_t key_state;    // debounced and inverted key state bit = 1: key pressed
volatile uint8_t key_press;                                // key press detect
volatile uint8_t key_rpt;                     // key long press and repeat

// RED LEDs
volatile uint8_t key_state_r;                                // debounced and inverted 
volatile uint8_t key_press_r;                                // key press detect
volatile uint8_t key_rpt_r;                                  // key long press and 
//rotary pushbuttons
volatile uint8_t key_state_p;                                // debounced and inverted 
volatile uint8_t key_press_p;                                // key press detect
volatile uint8_t key_rpt_p;

uint16_t hdg=180,hdg_old;
uint16_t spd=250,spd_old, spd_max=900,spd_min=100,spd_steps = 5;
uint32_t alt=0,  alt_old=77, alt_max=90000;
uint32_t com=127325,com_old, com_max=136975, com_min=118000, com_steps=25,com_steps_h=1000;
uint8_t com_mode=FINE;
uint32_t com2=122225,com2_old;
uint32_t nav=133335,nav_old;
uint32_t nav2=124445,nav2_old;
//TODO transponder
//TODO ADF
//TODO DME
uint8_t alt_steps = 100;
int vs=0, vs_old=77, vs_min=-3500,vs_max=4000,vs_steps = 100;
uint16_t baro=1013, baro_old;
  
// good baud rates for 16MHZ: 38400, 9600, 19200
// 0.2% error rate
#define USART_BAUD 19200ul
#define USART_UBBR_VALUE (F_CPU/16/USART_BAUD-1)

// shiftout & display: lower half of PORTC
// BCD out to 7442: bits 4,5,6 of PORTC
// PORTB: input rotary 0-3
// PORTA: 7 return lines from key matrix
#define HC595_PORT   PORTC
#define HC595_DDR    DDRC

#define HC595_DS_POS PC0      //Data pin (DS) pin location
#define HC595_SH_CP_POS PC2      //Shift Clock (SH_CP) pin location 
#define HC595_ST_CP_POS PC1      //Store Clock (ST_CP) pin location
#define WR PC3

#define CE 7   // bit 7 in shift-register 0

#define BIT_S(var,b) ((var&(1<<b))?1:0)

// 8 x CE  auf shift-register 3
uint8_t c_array[16]={
  0x81,9,5,0x5c,3,0x4a,0x46,0x4e,
  0x80,8,4,0x3c,2,0x2a,0x26,0x2e
};
// das 9. CE Bit  -  bit 7 in shift-register 0
uint8_t ce_array[16]={
  0,0,0,1,0,1,1,1,
  1,1,1,1,1,1,1,1
};

// out to the 4 shift registers
volatile uint8_t outword[4]={  255,255,255,255  };

char text[]="Hello World! 1234567890 abcdefghijklmnopqrstuvwxyz.,;<>!$%&/()=?";

//rotary state machine
#define R_CW_FINAL 0x1
#define R_CW_BEGIN 0x2
#define R_CW_NEXT 0x3
#define R_CCW_BEGIN 0x4
#define R_CCW_FINAL 0x5
#define R_CCW_NEXT 0x6
#define R_START 0x0
#define DIR_NONE 0x0
// Clockwise step.
#define DIR_CW 0x10
// Anti-clockwise step.
#define DIR_CCW 0x20

const unsigned char ttable[7][4] = {
  // R_START
  {R_START,    R_CW_BEGIN,  R_CCW_BEGIN, R_START},
  // R_CW_FINAL
  {R_CW_NEXT,  R_START,     R_CW_FINAL,  R_START | DIR_CW},
  // R_CW_BEGIN
  {R_CW_NEXT,  R_CW_BEGIN,  R_START,     R_START},
  // R_CW_NEXT
  {R_CW_NEXT,  R_CW_BEGIN,  R_CW_FINAL,  R_START},
  // R_CCW_BEGIN
  {R_CCW_NEXT, R_START,     R_CCW_BEGIN, R_START},
  // R_CCW_FINAL
  {R_CCW_NEXT, R_CCW_FINAL, R_START,     R_START | DIR_CCW},
  // R_CCW_NEXT
  {R_CCW_NEXT, R_CCW_FINAL, R_CCW_BEGIN, R_START},
};

unsigned char state [6] ={ R_START,R_START,R_START,R_START,R_START,R_START};
unsigned char stateB [6] ={ R_START,R_START,R_START,R_START,R_START,R_START};



ISR( TIMER0_OVF_vect )                            // every 10ms
{
  static uint8_t ct0, ct1, rpt;
  static uint8_t ct0_r, ct1_r, rpt_r;
  static uint8_t ct0_p, ct1_p, rpt_p;
  
  
  uint8_t i,i_r,i_p;
  
  time++;
  
  
  TCNT0 = (uint8_t)(int16_t)-(F_CPU / 1024 * 10e-3 + 0.5);  // preload for 10ms
  
  //LED_PORT ^= 1<<LED4;
  //show_lights(); 
       
  //ScanKeyMatrix();
  
  //green LEDs
  i = key_state ^ ~KEY_PIN;                       // key changed ?
  ct0 = ~( ct0 & i );                             // reset or count ct0
  ct1 = ct0 ^ (ct1 & i);                          // reset or count ct1
  i &= ct0 & ct1;                                 // count until roll over ?
  key_state ^= i;                                 // then toggle debounced state
  key_press |= key_state & i;                     // 0->1: key press detect
  // RED LEDs
  i_r = key_state_r ^ ~KEY_PIN_R;                       // key changed ?
  ct0_r = ~( ct0_r & i_r );                             // reset or count ct0
  ct1_r = ct0_r ^ (ct1_r & i_r);                          // reset or count ct1
  i_r &= ct0_r & ct1_r;                                 // count until roll over ?
  key_state_r^= i_r;                                 // then toggle debounced state
  key_press_r |= key_state_r & i_r;                     // 0->1: key press detect
  // rotary push
  i_p = key_state_p ^ ~KEY_PIN_P;                       // key changed ?
  ct0_p = ~( ct0_p & i_p );                             // reset or count ct0
  ct1_p = ct0_r ^ (ct1_p & i_p);                          // reset or count ct1
  i_p &= ct0_p & ct1_p;                                 // count until roll over ?
  key_state_p^= i_p;                                 // then toggle debounced state
  key_press_p |= key_state_p & i_p;
  
 /* if( (key_state & REPEAT_MASK) == 0 )            // check repeat function
     rpt = REPEAT_START;                          // start delay
     if( --rpt == 0 ){
       rpt = REPEAT_NEXT;                            // repeat delay
       key_rpt |= key_state & REPEAT_MASK;
     }
    */    
}

///////////////////////////////////////////////////////////////////
//
// check if a key has been pressed. Each pressed key is reported
// only once
//
uint8_t get_key_press( uint8_t key_mask )
{
  cli();                                          // read and clear atomic !
  key_mask &= key_press;                          // read key(s)
  key_press ^= key_mask;                          // clear key(s)
  sei();
  return key_mask;
}

uint8_t get_key_press_r( uint8_t key_mask_r )
{
  cli();                                          // read and clear atomic !
  key_mask_r &= key_press_r;                          // read key(s)
  key_press_r ^= key_mask_r;                          // clear key(s)
  sei();
  return key_mask_r;
}
uint8_t get_key_press_p( uint8_t key_mask_p )
{
  cli();                                          // read and clear atomic !
  key_mask_p &= key_press_p;                          // read key(s)
  key_press_p ^= key_mask_p;                          // clear key(s)
  sei();
  return key_mask_p;
}

uint8_t get_key_rpt( uint8_t key_mask )
{
  cli();                                          // read and clear atomic !
  key_mask &= key_rpt;                            // read key(s)
  key_rpt ^= key_mask;                            // clear key(s)
  sei();
  return key_mask;
}

///////////////////////////////////////////////////////////////////
//
// check if a key is pressed right now
//
uint8_t get_key_state( uint8_t key_mask )
{
  key_mask &= key_state;
  return key_mask;
}


uint8_t get_key_short( uint8_t key_mask )
{
  cli();                                          // read key state and key press atomic !
  return get_key_press( ~key_state & key_mask );
}

uint8_t get_key_short_r( uint8_t key_mask_r )
{
  cli();                                          // read key state and key press atomic !
  return get_key_press_r( ~key_state_r & key_mask_r );
}

uint8_t get_key_short_p( uint8_t key_mask_p )
{
  cli();                                          // read key state and key press atomic !
  return get_key_press_p( ~key_state_p & key_mask_p );
}


uint8_t get_key_long( uint8_t key_mask )
{
  return get_key_press( get_key_rpt( key_mask ));
}



void USART_vInit(void)
{
  UBRRH = (unsigned char)(USART_UBBR_VALUE>>8);
  UBRRL = (unsigned char)USART_UBBR_VALUE;
  UCSRB = (1<<RXEN)|(1<<TXEN);
  UCSRC = (0<<USBS)|(3<<UCSZ0)|(1<<URSEL);
}

void USART_vSendByte(uint8_t u8Data)
{
  //while( (UCSR0A & (1 << UDRE0)) == 0 ){};
  while(!(UCSRA&(1<<UDRE)));   
  UDR = u8Data;
}

uint8_t USART_vReceiveByte()
{
  while(!(UCSRA&(1<<RXC)));
  
  return UDR;
}

int uart_putc(unsigned char c)
{
    while (!(UCSRA & (1<<UDRE)))  /* warten bis Senden moeglich */
    {
    }                             
 
    UDR = c;                      /* sende Zeichen */
    return 0;
}
 
 
/* puts ist unabhaengig vom Controllertyp */
void uart_puts (char *s)
{
    while (*s)
    {   /* so lange *s != '\0' also ungleich dem "String-Endezeichen(Terminator)" */
        uart_putc(*s);
        s++;
    }
}


void ScanKeyMatrix(){
  uint8_t row=0;
  for (row=0;row<8;row++){
    
    PORTC = (PORTC & 0x0F) + (row << 4); 
    _delay_us(10);
    keybyte[row] = ~PINA;
    
  }
  //efis Tasten
  if (keybyte[0] != keybyte_old[0]){  
  //  led_clear(16-3,3);
  //  led_put_number(keybyte[0], 16);
    changes++;
    keybyte_old[0] = keybyte[0];
    kb0 = keybyte[0];
  }
  //schalter
  if (keybyte[1] != keybyte_old[1]){  
   // led_clear(20-3,3);
   // led_put_number(keybyte[1], 20);
   changes++;
    keybyte_old[1] = keybyte[1];
  }
  //ROSE
  if (keybyte[2] != keybyte_old[2]){
  //  led_clear(26-3,3);
  //  led_put_number(keybyte[2], 26);
    changes++;
    keybyte_old[2] = keybyte[2];
  }
  //zoom
  if (keybyte[3] != keybyte_old[3]){  
  //  led_clear(30-3,3);
  //  led_put_number(keybyte[3], 30);
    changes++;
    keybyte_old[3] = keybyte[3];
  }
  //Rafi
  if (keybyte[4] != keybyte_old[4]){  
    //led_clear(34-3,3);
   // led_put_number(keybyte[4], 34);
    changes++;
    keybyte_old[4] = keybyte[4];
    kb4=keybyte[4];
  }
  //rotary push
  if (keybyte[5] != keybyte_old[5]){  
   // led_clear(38-3,3);
   // led_put_number(keybyte[5], 38);
    changes++;
    keybyte_old[5] = keybyte[5];
    kb5 = keybyte[5];
    
  }
  //nav selector
  if (keybyte[6] != keybyte_old[6]){  
   // led_clear(42-3,3);
   // led_put_number(keybyte[6], 42);
    changes++;
    keybyte_old[6] = keybyte[6];
  }
  //2 switches
  if (keybyte[7] != keybyte_old[7]){  
   // led_clear(46-3,3);
   // led_put_number(keybyte[7], 46);
    changes++;
    keybyte_old[7] = keybyte[7];
  }
  
  
  
  
  
}


unsigned char RotaryProcessD( uint8_t knob) {
  // Grab state of input pins.
  uint8_t map[4]={
    0x03,
    0x0c,
    0x30,
    0xc0,};
    
    unsigned char pinstate;
    
    pinstate = PIND & map[knob] ;
    if (knob==1){
      pinstate >>= 2;
    }
    if (knob==2){
      pinstate >>= 4;
    }
    if (knob==3){
      pinstate >>= 6;
    }
    
    state[knob] = ttable[state[knob] & 0xf][pinstate];
    return state[knob];
}

unsigned char RotaryProcessB( uint8_t knob) {
  // Grab state of input pins.
  uint8_t map[4]={
    0x03,
    0x0c,
    0x30,
    0xc0,};
    
    unsigned char pinstate;
    
    pinstate = PINB & map[knob] ;
    if (knob==1){
      pinstate >>= 2;
    }
    if (knob==2){
      pinstate >>= 4;
    }
      //     if (knob==3){
	//       pinstate >>= 6;
	//     }
	
    stateB[knob] = ttable[stateB[knob] & 0xf][pinstate];
    return stateB[knob];
}



//Sends a clock pulse on SH_CP line
void HC595Pulse()
{
  //Pulse the Shift Clock
  HC595_PORT|=(1<<HC595_SH_CP_POS);//HIGH
  //wait(1);
  HC595_PORT&=(~(1<<HC595_SH_CP_POS));//LOW
}

//Sends a clock pulse on ST_CP line
void HC595Latch()
{
  //Pulse the Store Clock
  HC595_PORT|=(1<<HC595_ST_CP_POS);//HIGH
  //wait(1);
  HC595_PORT&=(~(1<<HC595_ST_CP_POS));//LOW
  //wait(1);
}

void HC595Write(uint8_t data)
{
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

// alphanumeric LED display: 1 char
void led_putc(uint8_t letter, uint8_t col)
{
  uint8_t b,ce,digit;
  b= col/4;
  digit = col % 4;      //auswahl von einer der 4 Stellen in einem DLO3416 Modul per A0 ind A1 
  
  if (digit == 0){
    digit=3;      
  }
  else{  
    if (digit == 3){
      digit=0;      
    }
  }
  
  outword[3] = c_array[b];
  outword[0] = letter;
  //TEST: one red LED(bit 3) on
  //outword[2] = 255-3;
  outword[2] |= digit;  // nur 2 bits ändern!!!
  
  ce=ce_array[b];
  if (ce == 0){
    outword[0] &= ~(1 << CE);      // CE Bit low
  }
  else
  {
    outword[0] |= (1 << CE);
  }
  
  HC595Write(outword[3])  ; // 8 x ce
  HC595Write(outword[2])  ;// LEDs + a0, a1
  //TEST - remove before flight :-)
  HC595Write(outword[1])  ;//  LEDs
  //wait(200);
  HC595Write(outword[0])  ;// D0..6  + CE9
  HC595Latch();
  
  //wait(1);
  PORTC &= ~(1 << WR);  // WR auf Low
  //wait(1);
  PORTC |= (1 << WR); 
  
  outword[2] &= 0xFC;  // a0 und a1 wieder auf 0
  
}

// alphanumeric LED display: singed int16_t
void led_put_number(int number, uint8_t col) 
{
  char Buffer[7];
  int c,j;
  char letter;
  
  
  //singed int
  itoa( number, Buffer, 10 );
  j = strlen(Buffer);    
  for (c=0;c<j;c++){
    letter = Buffer[c];
    if (letter == '0'){
      letter = 'O';
    }
    led_putc(letter,  col+c-j);
    //wait(500);
  }
}
//alphanumeric LED display: 32bit unsigend int
void led_put_u32_number(uint32_t number, uint8_t col) 
{
  char Buffer[7];
  int c,j;
  char letter;

  //unsinged 32 bit int
  ultoa( number, Buffer, 10 );
  j = strlen(Buffer);    
  for (c=0;c<j;c++){
    letter = Buffer[c];
    if (letter == '0'){
      letter = 'O';
    }
    led_putc(letter,  col+c-j);
    //wait(500);
  }
}

//alphanumeric LED display: com frequency
void led_put_com(uint32_t number, uint8_t col) 
{
  char Buffer[8];
  int c,j;
  char letter;
 
  //unsinged int
  ultoa( number, Buffer, 10 );
  j = strlen(Buffer);    
  for (c=0;c<3;c++){
    letter = Buffer[c];
    if (letter == '0'){
      letter = 'O';
    }
    led_putc(letter,  col+c-j);
    //wait(200);
  }  
  led_putc('.',  col+c-j);
  for (c=4;c<j;c++){
    letter = Buffer[c-1];
    if (letter == '0'){
      letter = 'O';
    }
    led_putc(letter,  col+c-j);
    //wait(200);
  } 

}



//alphanumeric LED display: fill area with ' '
void led_clear(uint8_t pos,uint8_t len)
{
  uint8_t c; 
  for (c=pos;c<(pos+len);c++){
    led_putc(' ',  c);
    //wait(100);
  }
}

//alphanumeric LED display: fille area with letter
void led_fill(char letter, uint8_t pos,uint8_t len)
{
  uint8_t c; 
  for (c=pos;c<(pos+len);c++){
    led_putc(letter,  c);
    //wait(100);
  }
}


//shiftout green and red LEDs
void show_lights()
 {
      changes++;
      HC595Write(0)  ; // 8 x ce
      HC595Write(outword[2])  ;// LEDs + a0, a1
      HC595Write(outword[1])  ;//  LEDs
      HC595Write(0)  ;// D0..6  + CE9
      HC595Latch();
    }

void green_leds()
{
uint8_t led_flag=0;
   if( get_key_short( 1<<KEY0 ))//ILS
     {
       LED_PORT ^= 1<<LED2;
       led_flag = 1;
     }
    if( get_key_short( 1<<KEY1 ))   //FD
    {
      LED_PORT ^= 1<<LED1;
      led_flag = 1;
    }
    if( get_key_short( 1<<KEY2 ))  //AP
    {
      LED_PORT ^= 1<<LED3;
      led_flag = 1;
    }
      if( get_key_short( 1<<KEY3 ))  //ATHR
    {
      LED_PORT ^= 1<<LED4;
      led_flag = 1;
    }
      if( get_key_short( 1<<KEY4 ))  //HEADING
    {
      LED_PORT ^= 1<<LED5;
      led_flag = 1;
    }
      if( get_key_short( 1<<KEY5 ))  //ALTITUDE
    {
      LED_PORT ^= 1<<LED6;
      led_flag = 1;
    }
      if( get_key_short( 1<<KEY6 ))  //VS
    {
      LED_PORT ^= 1<<LED7;
      led_flag = 1;
    }
    
    
    if (led_flag == 1)
    {
      show_lights();
      led_flag=0;
    }
   

  
  
}

void red_leds()
{
uint8_t led_flag=0;
   if( get_key_short_r( 1<<KEY0 ))
     {
       LED_PORT_R ^= 1<<LED6;
       led_flag = 1;
     }
    if( get_key_short_r( 1<<KEY1 ))
    {
      LED_PORT_R ^= 1<<LED5;
      led_flag = 1;
    }
    if( get_key_short_r( 1<<KEY2 ))  
    {
      LED_PORT_R ^= 1<<LED4;
      led_flag = 1;
    }
      if( get_key_short_r( 1<<KEY3 ))  
    {
      LED_PORT_R ^= 1<<LED3;
      led_flag = 1;
    }
      if( get_key_short_r( 1<<KEY4 ))  
    {
      LED_PORT_R ^= 1<<LED2;
      led_flag = 1;
    }
    
    if (led_flag == 1)
    {
      show_lights();
      led_flag=0;
    }
   
    
}


void serial_out()
{
  uint8_t i;
  uint8_t puffer[64];
 
  sprintf( puffer, "vs:%d,",vs );
  uart_puts( puffer );
  sprintf( puffer, "baro:%u,",baro );
  uart_puts( puffer );
  sprintf( puffer, "spd:%u,hdg:%u,alt:%lu,",spd,hdg,alt );
  uart_puts( puffer );
  sprintf( puffer, "com:%lu,com2:%lu,nav:%lu,nav2:%lu,bytes:",com,com2,nav,nav2 );
  uart_puts( puffer );
//TODO: trnasponder,DME, ADF
  for(i=0;i<8;i++){
    uart_putc(keybyte[i] );
  }
    for(i=1;i<3;i++){
    uart_putc(outword[i] );
  }
  uart_putc('\n' );
  changes=0;
}

void lightshow()
{
  uint8_t c;
   led_clear(0,64);
  for (c=32;c<36;c++){
    led_putc('X',  c);
    wait(30);
    led_putc(' ',  c);
  }
  for (c=8;c>0;c--)
  {
    LED_PORT_R = ~(1<<c);
    show_lights();
    wait(50);
  }   
  
  for (c=0;c<32;c++){
    led_putc('X',  c);
    wait(30);
    led_putc(' ',  c);
  }
  
  for (c=64;c>=32;c--){
    led_putc('X',  c);
    wait(30);
    led_putc(' ',  c);
  }
  
  for (c=0;c<8;c++)
  {
    
    LED_PORT = ~(1<<c);
    show_lights();
    wait(100);
  }    
  LED_PORT = 255;
  show_lights();
  
  wait(500);
  LED_PORT = 0;
  LED_PORT_R =0;
  show_lights();
  led_fill('3', 0,64);
  wait(1000);
  led_fill('2', 0,64);
  wait(1000);
  led_fill('1', 0,64);
  wait(1000);
  
  led_clear(0,64);	
  LED_PORT = 255;
  LED_PORT_R =255;
  show_lights();
}


int main()
{
  
  //     0b10000001,                      
  uint8_t c;
  
  uint8_t letter = 0;
  unsigned char ret;
  
  
  DDRC = 0xFF;  //Port C all outputs
  DDRB = 0x00;   // rotary inputs
  PORTB = 0x3F; // enable lower 4 pullup
  DDRD = 0x00;   // rotary inputs
  PORTD = 0xFF; // enable pullup
  
  
  DDRA = 0x00;   // 7 keyboard matrix input lines
  PORTA = 0xFF; // enable pullup
  
  TCCR0 = (1<<CS02)|(1<<CS00);         // divide by 1024
  TCNT0 = (uint8_t)(int16_t)-(F_CPU / 1024 * 10e-3 + 0.5);  // preload for 10ms
  TIMSK |= 1<<TOIE0;                   // enable timer interrupt
  
  sei();
  
  wait(500);
  lightshow();
  
  
  // testing serial IO
  USART_vInit();
  // 
  //         while(1)
  //         {
    //           data = USART_vReceiveByte();
  // 	  wait(1000);
  //           USART_vSendByte(data);
  //         }
  //         
  
  USART_vSendByte('o');
  USART_vSendByte('k');
  
  while(1)
  {
    
    //data = USART_vReceiveByte();
    
    // LED_PORT ^= 1<<LED5;
    //   show_lights(); 
       
    
    ScanKeyMatrix();
    
    //look for button presses and turn the LEDs on & off
    green_leds();
    red_leds();
    
    
    // time%100 -> 1HZ
    if(time%100==0 && time != time_old){
      led_put_number(time/100,64);
      //flicker LED
      // LED_PORT ^= 1<<LED4;
      // show_lights(); 
       
      led_fill(' ',54-5,5);
      led_put_number(changes,54);
      if (changes > 0){ 
	  serial_out();
      }
      time_old = time;
    }
    
    //level off
    if( get_key_short_p( 1<<KEY6 ))
     {
       vs=0;
       changes++;
     }
     
     //com steps
     if( get_key_short_p( 1<<KEY2 ))
     {
       changes++;
       if (com_mode==COARSE){
	 com_mode=FINE;
	 com_steps=25;
	 led_putc('>', 29);
       }
       else
       {
	 com_mode=COARSE;
	 com_steps=1000;
	 led_putc('<', 29);
       }
     }
   
   // all rotary encoders ===========================================
      
      // SPEED 
      ret = RotaryProcessD(1);
      if (ret == 0x10)
      {
	if (spd >spd_min){
	  spd -=spd_steps;
	
	}
	
      }
      
      if (ret == 0x20)
      { 
	if (spd < spd_max){
	  spd += spd_steps;
	  
	}
      }
      
      if (spd != spd_old)
      {
	led_clear(3-3,3);
	led_put_number(spd,3);
	spd_old = spd;
	changes++;
      }    
      
      // HEADING
      ret = RotaryProcessD(2);
      if (ret == 0x20)
      {
	if (hdg == 0){
	  hdg=359;
	}
	else{
	  hdg -=1 ;
	}
      }
      if (ret == 0x10)
      {
	if (hdg == 359){
	  hdg=0;
	}
	else{
	  hdg += 1;
	}
      }
      if (hdg != hdg_old)
      {
	led_fill('O',7-3,3);
	led_put_number(hdg,7);
	hdg_old = hdg;
	changes++;
      }  
      
      // ALTITUDE
      alt_steps=100; 
      ret = RotaryProcessD(3);
      if (ret == 0x20)
      {
	if (alt >= alt_steps){
	  alt -=alt_steps;
	}
	
      }
      if (ret == 0x10)
      {
	if (alt < alt_max){
	  alt+=alt_steps;
	}
	
      }
      if (alt != alt_old)
      {
	led_fill(' ',14-5,5);
	led_put_u32_number(alt,14);
	alt_old = alt;
	changes++;
      }  
      //BARO
      ret = RotaryProcessB(2);
      if (ret == 0x20)
      {
	if (baro > 0){
	  baro--;
	}
	
      }
      if (ret == 0x10)
      {
	baro++;
      }
      if (baro != baro_old)
      {
	led_fill(' ',36-4,4);
	led_put_number(baro,36);
	baro_old = baro;
	changes++;
      }  
      // VS
      ret = RotaryProcessB(0);
      if (ret == 0x10)
      {
	if (vs > vs_min){
	  vs-=vs_steps;
	}
	
      }
      if (ret == 0x20)
      {
	if (vs < vs_max){
	  vs+=vs_steps;
	}
      }
      if (vs != vs_old)
      {
	led_fill(' ',21-5,5);
	led_put_number(vs,21);
	vs_old = vs;
	changes++;
      } 
      
      //COM
      ret = RotaryProcessB(1);
      if (ret == 0x20)
      {
	if (com > com_min){
	 com-=com_steps;
	}
	
      }
      if (ret == 0x10)
      {
	if (com < com_max){
	  com+=com_steps;
	}
      }
      if (com != com_old)
      {
	led_fill(' ',32-7,7);
	led_put_com(com,32);
	com_old = com;
	changes++;
      }    
      
      
      
  } // while 1
  
} //main