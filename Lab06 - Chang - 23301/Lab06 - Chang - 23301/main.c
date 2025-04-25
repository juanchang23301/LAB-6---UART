/************************************************************************
* Universidad del Valle de Guatemala
* IE2023: Programaci�n de Microcontroladores
* Conexi�n UART
*
* Autor: Juan Ren� Chang Lam
*
* Descripci�n: Implementaci�n de comunicaci�n serial UART con
* visualizaci�n de caracteres recibidos en LEDs y funci�n de eco.
* El programa inicializa la UART, env�a un car�cter inicial y
* luego responde a cada car�cter recibido mostr�ndolo en los LEDs
* y devolvi�ndolo por la UART.
*
* Conexiones:
* - PD5 - PD7: Bits menos significativos (LEDs)
* - PB0 - PB4: Bits m�s significativos (LEDs)
* - PD0: RX (entrada UART)
* - PD1: TX (salida UART)
************************************************************************/

#define F_CPU 16000000UL 
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#define BAUD 9600         
#define UBRR_VALUE ((F_CPU/(16UL*BAUD))-1)  // C�lculo del valor de UBRR

void UART_init(void);
void UART_sendChar(unsigned char data);
void Leds_send(unsigned char character);

int main(void) {
	DDRD |= 0b11100000;  // PIND5-PIND7 como salidas
	DDRB |= 0b00011111;  // PINB0-PINB4 como salidas
	
	UART_init();    // Inicializaci�n de la comunicaci�n UART
	
	sei();          
	
	UART_sendChar('A');  
	
	while (1) {
	}
	
	return 0;
}

void UART_init(void) {
	// Configuraci�n del baudrate
	UBRR0H = (uint8_t)(UBRR_VALUE >> 8);
	UBRR0L = (uint8_t)UBRR_VALUE;
	
	UCSR0B = (1 << TXEN0) | (1 << RXEN0) | (1 << RXCIE0);
	UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
}

void UART_sendChar(unsigned char data) {
	while (!(UCSR0A & (1 << UDRE0)));
	
	UDR0 = data;  
}

void Leds_send(unsigned char character) {
	// Distribuir los 8 bits del car�cter en los LEDs 
	PORTD = (PORTD & 0b00011111) | ((character & 0b00000111) << 5);
	PORTB = (PORTB & 0b11100000) | ((character >> 3) & 0b00011111);
}

// Interrupci�n de recepci�n UART
ISR(USART_RX_vect) {
	char received_data = UDR0;  // Leer el dato recibido
	
	Leds_send(received_data);   // Mostrar el car�cter en los LEDs
	
	UART_sendChar(received_data);  // Enviar eco del car�cter recibido
}