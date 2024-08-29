/*
 * flight_controller.c
 *
 * Created: 8/25/2024 12:28:47 AM
 * Author : Mario
 */ 
#define F_CPU 16000000UL  // 16 MHz clock speed
#define BAUD 9600
#define BUFFER_SIZE 128

#include <avr/io.h>
#include <util/delay.h>	
#include <avr/interrupt.h>

// PORTB GPIO Registers 
#define DDRB_REG *(volatile uint8_t *)(0x24) //using _REG to avoid same macro definitions as with avr/io.h
#define PORTB_REG *(volatile uint8_t *)(0x25)
// Timer 1 Registers
#define TCCR1B_REG *(volatile uint8_t *)(0x81)
#define OCR1A_REG *(volatile uint16_t *)(0x88)
#define TIMSK1_REG *(volatile uint8_t *)(0x6F)

// Timer 0 Registers
#define TCCR0A_REG *(volatile uint8_t *)(0x44) 
#define TCCR0B_REG *(volatile uint8_t *)(0x45) 
#define TCNT0_REG *(volatile uint8_t *)(0x46) // where the ticks of the counter are stored from 0-255
#define TIMSK0_REG *(volatile uint8_t *)(0x6E)

// Status Register
#define SREG_REG *(volatile uint8_t *)(0x5F)

// All remaining registers macros are going to be used from avr/io.h 
//


// PORTB GPIO
#define PIN8_DIR 0
#define PIN9_DIR 1
#define PIN10_DIR 2
#define PIN11_DIR 3
#define PIN12_DIR 4
#define PIN13_DIR 5

#define PIN8 0
#define PIN9 1
#define PIN10 2
#define PIN11 3
#define PIN12 4
#define PIN13 5

volatile unsigned long timer0_ms = 0;
volatile unsigned long timer0_ms_frac = 0;

volatile char serialRingBuffer[BUFFER_SIZE];
volatile char *bufferHead = serialRingBuffer;
volatile char *bufferTail = serialRingBuffer;
volatile char tx_busy = 0;

//////////////////////////////////////////////////////////

void timer0_init();
void USART0_init(unsigned int baud);
unsigned long millis();
unsigned long micros();
void USART_write_char(char c);
void USART_write(char *c);



int main(void)
{
	// set pin 8 to 13 as output
	DDRB_REG = 0b00111111; 
	
	/////// TIMER1 config //////
	
	// set prescaler of 256 to TIMER1
	TCCR1B_REG |= (1<<2);
	// set to CTC mode (compare match clear)
	TCCR1B_REG |= (1<<3);
	// set to trigger interrupt when TIMER1 counts to 15624 - 1 second
	OCR1A_REG = 62499;
	// enable interrupt on compare match
	TIMSK1_REG |= (1<<1);
	
	///////---------///////////////////
	
	// Timer 0 init
	timer0_init();
	// USART0 init
	USART0_init(9600);
	// enable global interrupts
	sei();
	
		
	
    while (1) 
    {
		
		//PORTB_REG |= (1<<PIN13); // turn LED on
		//_delay_ms(1000);
		//PORTB_REG &= ~(1<<PIN13); // turn LED off
		//_delay_ms(1000);	 
		 
    }
}

void timer0_init()
{

	// set timer0 to normal mode - interrupt triggers on overflow
	TCCR0A_REG = 0;
	// set prescaler to 64 - every tick of the counter is 4 us, 256 ticks are 1 ms
	TCCR0B_REG |= ((1<<0) | (1<<1));
	// enable overflow interrupt for timer0
	TIMSK0_REG |= (1<<0);
	
	
}

// NB! to perform a check if a bit is high or low of register
// if (reg & (1<<5) == true) --> when we do bitwise AND of reg and bit 5 of a 8bit we get TRUE when that bit is set and false if its 0, the anding with the other bits doesnt change them
void USART0_init(unsigned int baud)
{
	unsigned int BRC = (F_CPU/16/baud - 1);
	// set the baudrate - UBRR0 register must hold the value calculated in BRC
	// extract the high byte by shifting the value 8 bits to the right and type casting to uint8_t
	UBRR0H = (uint8_t)(BRC>>8);
	// extract the low byte by type casting the value
	UBRR0L = (uint8_t)(BRC);
	
	// enable tx/rx and also tx/rx interrupt on completion
	UCSR0B = (1<<RXEN0) | (1<<TXEN0) | (1<<TXCIE0) | (1<<RXCIE0);
	
	// 8bit, 1 stop bit (by default), no parity (by default)
	UCSR0C |= (1<<UCSZ01) | (1<<UCSZ00); // sets only to 8 bits
}

// return an atomic read of the volatile variable that is being incremented in the ISR
// this will correspond the current millis() since the program started
unsigned long millis()
{
	unsigned long ms;
	// store the current value of status register where global interrupt flag is stored
	uint8_t SREG_prev = SREG_REG;
	// disable interrupts to allow for atomic read
	cli();
	ms = timer0_ms;
	// enable back interrupts
	SREG_REG = SREG_prev;
	
	return ms;
}

// this will return the current us with resolution of 4 so we can count only 4,8,12,16 ..
unsigned long micros()
{
	unsigned long microsec;
	
	uint8_t SREG_prev = SREG_REG;
	cli();
	// TCNT0_REG stores the current count of timer0
	microsec = (timer0_ms * 1024) + TCNT0_REG * 4;
	SREG_REG = SREG_prev;
	
	return microsec;
}

// write char to the ring buffer
void USART_write_char(char c)
{
	volatile char *nextHead = bufferHead + 1;
	
	// Wrap around when reaching end of buffer
	if (nextHead >= serialRingBuffer + BUFFER_SIZE) // serialRingBuffer is the address of 0 element + BUFFER_SIZE gives the address of position 128
	{
		nextHead = serialRingBuffer; // Wrap around to the start address of the buffer (position 0 in array)
	}
	
	// wait if the buffer is full to avoid overwriting
	while (nextHead == bufferTail);
	
	// we write to bufferHead the character, bufferHead points to the ring buffer
	*bufferHead = c;
	bufferHead = nextHead;
	
	// If transmission is not already in progress, start it
	if (!tx_busy) // if tx_busy is 1, this wont execute
	{
		tx_busy = 1;  // Set the busy flag
		UDR0 = *bufferTail;  // Load the first character into the USART Data Register
		bufferTail++;

		// wrap around when reaching the end of buffer
		if (bufferTail >= bufferTail + BUFFER_SIZE)
		{
			bufferTail = serialRingBuffer;  // Wrap around to the start address of the buffer (position 0 in array)
		}
	}
}

void USART_write(char *c)
{
	while(*c) // it will be true, until the end of the string when it reaches the null terminator
	{
		// send each char of the string
		USART_write_char(*c++); // dereference and pass the char and then increment the ptr
	}
}


////////////////// ISR//////////////

// increment var every 1024 us - used for millis() and micros()
ISR(TIMER0_OVF_vect)
{
	// copy volatiles to local variable to perform computations as working with volatile var require direct memory access each time (slower process)
	unsigned long timer0_ms_loc = timer0_ms;
	unsigned long timer0_ms_frac_loc = timer0_ms_frac;
	
	// every 1024 us this value is being incremented by 1
	timer0_ms_loc++;
	//store the remaining us to a fractional variable and increment by 24 on each interrupt
	timer0_ms_frac_loc += 24;
	// when the fractional part exceeds 1000 a full ms have passed
	if (timer0_ms_frac_loc > 1000)
	{
		// increment the ms variable again
		timer0_ms_loc++;
		// store the leftover after 1000
		timer0_ms_frac_loc -= 1000;
	}
	
	// copy back to the volatile global
	timer0_ms = timer0_ms_loc;
	timer0_ms_frac = timer0_ms_frac_loc;

}


ISR(TIMER1_COMPA_vect)
{
	// toggle LED every 1 sec
	PORTB_REG ^= (1<<PIN13);
}

// triggers when a byte is transmitted
ISR(USART_TX_vect) 
{
	// if buffer is not empty then more data must be sent
	if(bufferTail != bufferHead)
	{
		// on finished byte transmission, the next tail must be loaded for transmission
		UDR0 = *bufferTail;
		bufferTail++;
		
		// wrap around when reaching the end of buffer
		if (bufferTail >= bufferTail + BUFFER_SIZE)
		{
			bufferTail = serialRingBuffer;  // Wrap around to the start address of the buffer (position 0 in array)
		}
	} else 
	{
		// no more data to be sent as ring buffer is empty, new serial transmission can be re-strated with USART_write()
		tx_busy = 0;
	}

}




