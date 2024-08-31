#define F_CPU 16000000UL  // 16 MHz clock speed
#define BAUD 9600
#define BUFFER_SIZE 128

#include <avr/io.h>
#include <util/delay.h>	
#include <avr/interrupt.h>
#include <stdio.h>
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




volatile unsigned long timer0_ms = 0;
volatile unsigned long timer0_ms_frac = 0;

volatile char serialRingBuffer[BUFFER_SIZE];
volatile char *bufferHead = serialRingBuffer;
volatile char *bufferTail = serialRingBuffer;
volatile char tx_busy = 0;
unsigned long current_time,ch1_time,ch2_time,ch3_time,ch4_time;
uint8_t prev_channel_1_state,prev_channel_2_state,prev_channel_3_state,prev_channel_4_state;
volatile unsigned long receiver_input[4];


//////////////////////////////////////////////////////////

void timer0_init();
void USART0_init(unsigned int baud);
void pinChangeInterrupt_init();
unsigned long millis();
unsigned long micros();
void USART_write_char(char c);
void USART_write(char *c);
void read_receiver();



int main(void)
{
	// Timer 0 init
	timer0_init();
	// USART0 init
	USART0_init(9600);
	// Pin Change Interrupt for Port B - PCINT0-PCINT3 init
	pinChangeInterrupt_init();
	// enable global interrupts
	sei();
	
		
    while (1) 
    {
		read_receiver();

    }
}

void read_receiver()
{
	
	char buffer[128];
	sprintf(buffer,"CH1: %lu\t CH2: %lu\t CH3: %lu\t CH4: %lu\r\n",receiver_input[0],receiver_input[1],receiver_input[2],receiver_input[3]);
	USART_write(buffer);
	_delay_ms(250);
}





//////////////

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

void pinChangeInterrupt_init()
{
	// Set pin 8-11 as inputs
	DDRB_REG = 0;
	// No pull-ups for the pins
	PORTB_REG = 0;
	// Enable Pin Change Interrupt for Port B
	PCICR |= (1<<PCIE0);
	

	// Enable Arduino pins 8-11 to trigger interrupt on pin change
	PCMSK0 |= ((1<<PCINT3) | (1<<PCINT2) | (1<<PCINT1) | (1<<PCINT0));
	
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
		if (bufferTail >= serialRingBuffer + BUFFER_SIZE)
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
	//PORTB_REG ^= (1<<PIN13);
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
		if (bufferTail >= serialRingBuffer + BUFFER_SIZE)
		{
			bufferTail = serialRingBuffer;  // Wrap around to the start address of the buffer (position 0 in array)
		}
	} else 
	{
		// no more data to be sent as ring buffer is empty, new serial transmission can be re-strated with USART_write()
		tx_busy = 0;
	}

}

ISR(PCINT0_vect)
{
	// need to check which of the 4 pins fired the interrupt and measure the PWM
	
	current_time = micros();

	// Channel 1
	if(PINB & 1<<PINB0)								// check if channel_1 is high
	{
		if(prev_channel_1_state == 0)				// pin change from 0 to 1
		{
			prev_channel_1_state = 1;
			// record the time
			ch1_time = current_time;
		}
	}
	else if (prev_channel_1_state == 1)				// channel_1 is not high, pin change from 1 to 0
	{
		prev_channel_1_state = 0;
		// record values only in that range, in order to discard occasional +-1000us jumps
		if(((current_time-ch1_time) <= 2000) && ((current_time-ch1_time) >= 1000))	
		{
			// record the pulse width of the signal
			receiver_input[0] = current_time - ch1_time;
		}
	}
	// Channel 2
	if(PINB & 1<<PINB1)								// check if channel_2 is high
	{
		if(prev_channel_2_state == 0)				// pin change from 0 to 1
		{
			prev_channel_2_state = 1;
			// record the time
			ch2_time = current_time;
		}
	}
	else if (prev_channel_2_state == 1)				// channel_2 is not high, pin change from 1 to 0
	{
		prev_channel_2_state = 0;
		// record values only in that range, in order to discard occasional +-1000us jumps
		if(((current_time-ch2_time) <= 2000) && ((current_time-ch2_time) >= 1000))
		{
			// record the pulse width of the signal
			receiver_input[1] = current_time - ch2_time;
		}
	}
	// Channel 3
	if(PINB & 1<<PINB2)								// check if channel_3 is high
	{
		if(prev_channel_3_state == 0)				// pin change from 0 to 1
		{
			prev_channel_3_state = 1;
			// record the time
			ch3_time = current_time;
		}
	}
	else if (prev_channel_3_state == 1)				// channel_3 is not high, pin change from 1 to 0
	{
		prev_channel_3_state = 0;
		// record values only in that range, in order to discard occasional +-1000us jumps
		if(((current_time-ch3_time) <= 2000) && ((current_time-ch3_time) >= 1000))
		{
			// record the pulse width of the signal
			receiver_input[2] = current_time - ch3_time;
		}
	}
	// Channel 4
	if(PINB & 1<<PINB3)								// check if channel_4 is high
	{
		if(prev_channel_4_state == 0)				// pin change from 0 to 1
		{
			prev_channel_4_state = 1;
			// record the time
			ch4_time = current_time;
		}
	}
	else if (prev_channel_4_state == 1)				// channel_4 is not high, pin change from 1 to 0
	{
		prev_channel_4_state = 0;
		// record values only in that range, in order to discard occasional +-1000us jumps
		if(((current_time-ch4_time) <= 2000) && ((current_time-ch4_time) >= 1000))
		{
			// record the pulse width of the signal
			receiver_input[3] = current_time - ch4_time;
		}
	}
	

}




