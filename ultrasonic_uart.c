#ifndef F_CPU
#define F_CPU 16000000UL
#endif

#include <avr/io.h>
#include <util/delay.h>
#include <stdint.h>

#define BAUD_RATE 9600
#define UBRR_VALUE ((F_CPU / (16UL * BAUD_RATE)) - 1)

#define TRIG_DDR  DDRB
#define TRIG_PORT PORTB
#define TRIG_PIN  PB0 // Arduino D8

#define ECHO_DDR  DDRD
#define ECHO_PINR PIND
#define ECHO_PORT PORTD
#define ECHO_PIN  PD7 // Arduino D7

void uart_init(void)
{
    UBRR0H = (unsigned char)(UBRR_VALUE >> 8);
    UBRR0L = (unsigned char)(UBRR_VALUE);
    UCSR0B = (1 << RXEN0) | (1 << TXEN0);
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
    DDRD |= (1 << DDD1); // Set TX (PD1) as output
}

void uart_send(unsigned char data)
{
    while (!(UCSR0A & (1 << UDRE0)));
    UDR0 = data;
}

void uart_print(const char *s)
{
    while (*s) uart_send(*s++);
}

void uart_print_uint(uint16_t n)
{
    char buf[6];
    uint8_t i = 0;

    if (n == 0) {
        uart_send('0');
        return;
    }
    
    while (n > 0) {
        buf[i++] = '0' + (n % 10);
        n /= 10;
    }
    while (i > 0) {
        uart_send(buf[--i]);
    }
}

void ultrasonic_init(void)
{
    TRIG_DDR  |=  (1 << TRIG_PIN);  // TRIG = Output
    ECHO_DDR  &= ~(1 << ECHO_PIN);  // ECHO = Input
    TRIG_PORT &= ~(1 << TRIG_PIN);  // TRIG Low
    ECHO_PORT &= ~(1 << ECHO_PIN);  // Disable internal pull-up on ECHO pin (FIXED)
}

void timer1_init(void)
{
    TCCR1A = 0;
    TCCR1B = (1 << CS11);   /* Prescaler 8: 16 MHz / 8 = 2 MHz (1 tick = 0.5 us) */
    TCNT1  = 0;
}

uint16_t ultrasonic_read_us_timer(void)
{
    uint16_t pulse_ticks;

    // 1. Send 10us Trigger Pulse
    TRIG_PORT &= ~(1 << TRIG_PIN);
    _delay_us(2);
    TRIG_PORT |=  (1 << TRIG_PIN);
    _delay_us(10);
    TRIG_PORT &= ~(1 << TRIG_PIN);

    // 2. Reset Timer1 before waiting
    TCNT1 = 0;

    // 3. Wait for ECHO HIGH (Max timeout: ~30ms = 60,000 ticks)
    while (!(ECHO_PINR & (1 << ECHO_PIN))) {
        if (TCNT1 > 60000) return 0; // Hardware timeout: Returns 0 if sensor fails to respond
    }

    // 4. Reset Timer1 right as ECHO goes HIGH
    TCNT1 = 0;

    // 5. Wait for ECHO LOW (Max pulse width: ~30ms = 60,000 ticks)
    while (ECHO_PINR & (1 << ECHO_PIN)) {
        if (TCNT1 > 60000) break; // Safety cap if signal gets stuck HIGH
    }

    pulse_ticks = TCNT1;
    return pulse_ticks / 2; /* 2 ticks = 1 µs */
}

/*int main(void)
{
    uart_init();
    ultrasonic_init();
    timer1_init();

    while (1) {
        uint16_t us = ultrasonic_read_us_timer(); // Returns duration in microseconds
        uint16_t cm = (us / 58)/100;                     // Direct conversion to centimeters

        uart_print("Distance: ");
        uart_print_uint(cm);
        uart_print(" cm\r\n");

        _delay_ms(100);
    }
    return 0;
}*/

int main(void)
{
    uart_init();
    ultrasonic_init();
    timer1_init();

    uart_print("\r\n--- HC-SR04 HARDWARE DIAGNOSTIC ---\r\n");

    while (1) {
        // Test 1: Is ECHO pin already stuck HIGH before trigger?
        if (ECHO_PINR & (1 << ECHO_PIN)) {
            uart_print("ERR: D7 is stuck HIGH before trigger! Check pin D7 wiring.\r\n");
            _delay_ms(1000);
            continue;
        }

        // Send 10us Trigger Pulse
        TRIG_PORT &= ~(1 << TRIG_PIN);
        _delay_us(2);
        TRIG_PORT |=  (1 << TRIG_PIN);
        _delay_us(10);
        TRIG_PORT &= ~(1 << TRIG_PIN);

        TCNT1 = 0;
        // Wait for ECHO HIGH
        while (!(ECHO_PINR & (1 << ECHO_PIN))) {
            if (TCNT1 > 60000) break;
        }

        uint16_t response_delay = TCNT1;
        if (response_delay > 60000) {
            uart_print("ERR: Sensor ignored trigger (ECHO never went HIGH).\r\n");
            _delay_ms(1000);
            continue;
        }

        // Reset timer at start of pulse
        TCNT1 = 0;
        while (ECHO_PINR & (1 << ECHO_PIN)) {
            if (TCNT1 > 60000) break;
        }
        uint16_t pulse_ticks = TCNT1;

        uart_print("SUCCESS | Trig-to-Echo Delay: ");
        uart_print_uint(response_delay / 2);
        uart_print(" us | Pulse: ");
        uart_print_uint(pulse_ticks / 2);
        uart_print(" us | Dist: ");
        uart_print_uint((pulse_ticks / 2) / 58);
        uart_print(" cm\r\n");

        _delay_ms(500);
    }
    return 0;
}