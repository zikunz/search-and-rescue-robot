#if !(defined(__AVR) || defined(AVR) || defined(__AVR_ATmega328P__))
#define __AVR_ATmega328P__
#endif

#include <avr/interrupt.h>
#include <avr/io.h>
#include <avr/sleep.h>
#include <inttypes.h>
#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include <util/delay.h>
#include "buffer.h"
#include "constants.h"
#include "packet.h"
#include "serialize.h"

// use L and H for 16 bit regs
#define rbi(sfr, bit) (_SFR_BYTE(sfr) & _BV(bit))
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#define BUF_LEN 512

// Number of ticks per revolution from the wheel encoder.
#define COUNTS_PER_REV 200

// Wheel circumference in cm used to calculate forward/backward distance
// traveled by taking revs * WHEEL_CIRC.
#define WHEEL_CIRC 20.42

// Motor control pins.
#define LF 5            // Left forward pin
#define LR 6            // Left reverse pin
#define RF 10           // Right forward pin
#define RR 9            // Right reverse pin
#define ALEX_LENGTH 16  // alex length
#define ALEX_BREADTH 6  // alex breath

// TCS3200/230 colour sensor pins
#define S0 0
#define S1 1
#define S2 4
#define S3 7
#define OUT 8

// Mask definitions for power management
#define PRR_TWI_MASK 0b10000000
#define PRR_TIMER2_MASK 0b01000000
#define PRR_TIMER0_MASK 0b00100000
#define PRR_SPI_MASK 0b00000100
#define PRR_ADC_MASK 0b00000001
#define PRR_TIMER1_MASK 0b00001000
#define SMCR_SLEEP_ENABLE_MASK 0b00000001
#define SMCR_IDLE_MODE_MASK 0b00000001
#define ADCSRA_ADC_MASK 0b1000000

// Ultrasonic sensor pins
#define TRIGGER_PIN 11  // Trigger pin of ultrasonic sensor (orange)
#define ECHO_PIN 12     // Echo pin of ultrasonic sensor (green)

// Direction values
typedef enum {
    STOP = 0,
    FORWARD = 1,
    BACKWARD = 2,
    LEFT = 3,
    RIGHT = 4
} TDirection;

volatile TBuffer sendbuf, recvbuf;

volatile TDirection dir = STOP;

// Alex's diagonal. We compute and store this value once since it is
// expensive to compute and really does not change
float alexDiagonal = 0.0;
float alexCirc = 0.0;

// Ticks from Alex's left and right encoders.
volatile unsigned long leftForwardTicks = 0;
volatile unsigned long rightForwardTicks = 0;
volatile unsigned long leftReverseTicks = 0;
volatile unsigned long rightReverseTicks = 0;

volatile unsigned long leftForwardTicksTurns = 0;
volatile unsigned long rightForwardTicksTurns = 0;
volatile unsigned long leftReverseTicksTurns = 0;
volatile unsigned long rightReverseTicksTurns = 0;

// Revolutions on Alex's left and right wheels
volatile unsigned long leftRevs = 0;
volatile unsigned long rightRevs = 0;

// Forward and backward distance traveled
volatile unsigned long forwardDist = 0;
volatile unsigned long reverseDist = 0;

// Variables to keep track of whether we have moved a command distance
volatile unsigned long deltaDist = 0;
volatile unsigned long newDist = 0;

// Variables to keep track of our turning angle
volatile unsigned long deltaTicks = 0;
volatile unsigned long targetTicks = 0;

// TCS3200/230 colour sensor initial RGB values
volatile unsigned long red = 0;
volatile unsigned long blue = 0;
volatile unsigned long green = 0;

// Colour detected by TCS3200/230 colour sensor
volatile unsigned long
    colour;  // 'u' for unknown, 'r' for red and 'g' for green

// Ultrasonic sensor reading
volatile int near = 0;  // 0 for not very near and 1 for very near

void WDT_off(void) {
    // Global interrupt should be turned OFF here if not already done so
    cli();

    // Clear WDRF in MCUSR
    MCUSR &= ~(1 << WDRF);

    // Write logical one to WDCE and WDE
    // Keep old prescaler setting to prevent unintentional time-out
    WDTCSR |= (1 << WDCE) | (1 << WDE);

    // Turn off WDT
    WDTCSR = 0x00;

    // Global interrupt should be turned ON here if subsequent operations
    // after calling this function do not require turning off global interrupt
    sei();
}

void setupPowerSaving() {
    // Turn off the Watchdog Timer
    WDT_off();

    // Modify PRR to shut down TWI
    PRR |= PRR_TWI_MASK;

    // Modify PRR to shut down SPI
    PRR |= PRR_SPI_MASK;

    // Modify ADCSRA to disable ADC,
    // then modify PRR to shut down ADC
    ADCSRA |= ADCSRA_ADC_MASK;
    PRR |= PRR_ADC_MASK;

    // Set the SMCR to choose the IDLE sleep mode
    // Do not set the Sleep Enable (SE) bit yet
    SMCR &= SMCR_IDLE_MODE_MASK;

    // Set Port B Pin 5 as output pin, then write a logic LOW to it such
    // that the LED tied to Arduino's Pin 13 is OFF.
    DDRB |= 0b00100000;
    PORTB &= 0b11011111;
}

void putArduinoToIdle() {
    // Modify PRR to shut down TIMER 0, 1 and 2
    PRR |= PRR_TIMER0_MASK;
    PRR |= PRR_TIMER1_MASK;
    PRR |= PRR_TIMER2_MASK;

    // Modify SE bit in SMCR to enable (i.e., allow) sleep
    SMCR |= SMCR_SLEEP_ENABLE_MASK;

    // The following function puts ATmega328P's MCU into sleep;
    // it wakes up from sleep when USART serial data arrives sleep_cpu();
    sleep_cpu();

    // Modify SE bit in SMCR to disable (i.e., disallow) sleep
    SMCR &= (~SMCR_SLEEP_ENABLE_MASK);

    // Modify PRR to power up TIMER 0, 1, and 2
    PRR &= (~PRR_TIMER0_MASK);
    PRR &= (~PRR_TIMER1_MASK);
    PRR &= (~PRR_TIMER2_MASK);
}

void pwmWrite(uint8_t pin, int val) {
    volatile uint8_t* timer_comp;

    switch (pin) {
        case LF:
            timer_comp = &OCR0B;
            break;

        case LR:
            timer_comp = &OCR0A;
            break;

        case RF:
            timer_comp = &OCR1AL;
            break;

        case RR:
            timer_comp = &OCR1BL;
            break;

        default:
            return;
    }
    *timer_comp = val;
}

// Read the serial port. Return the read character in
// ch if available. Also return TRUE if ch is valid.
int readSerial(char* buffer) {
    int count = 0;

    for (count = 0; dataAvailable(&recvbuf); count += 1) {
        readBuffer(&recvbuf, (unsigned char*)&buffer[count]);
    }

    return count;
}

// Write to the serial port.
void writeSerial(const char* buffer, int len) {
    for (int i = 0; i < len; i += 1) {
        writeBuffer(&sendbuf, buffer[i]);
    }

    sbi(UCSR0B, UDRIE0);
}

// Alex Communication Routines.
TResult readPacket(TPacket* packet) {
    // Read in data from the serial port and
    // deserialize it.Return deserialized
    // data in "packet".
    char buffer[PACKET_SIZE];

    int len = readSerial(buffer);

    if (len == 0) {
        return PACKET_INCOMPLETE;
    } else {
        return deserialize(buffer, len, packet);
    }
}

void sendResponse(TPacket* packet) {
    // Take a packet, serialize it then send it out
    // over the serial port.
    char buffer[PACKET_SIZE];
    int len;

    len = serialize(buffer, packet, sizeof(TPacket));
    writeSerial(buffer, len);
}

#define pulseWidth(sfr, bit)                                                                                          \
    __extension__({                                                                                                   \
        const uint8_t initial = rbi(sfr, bit);                                                                        \
        while (rbi(sfr, bit) == initial)                                                                              \
            ; /*  for last pulse to end  */                                                                           \
        while (rbi(sfr, bit) != initial)                                                                              \
            ; /*  for next pulse to begin  */                                                                         \
        volatile uint32_t width =                                                                                     \
            0; /*sufficient to provide 2^32/16M=268s of count*/                                                       \
        /*                                                                                                            \
        while (rbi(sfr, bit) == initial) {                                                                            \
            width += 1;                                                                                               \
        }                                                                                                             \
        */                                                                                                            \
        __asm__ volatile ( /*  prevent compiler from optimize this since the timing of instructions is critical  */ \
        /*  first zero them out  */ \
        "clr r24 \n\t" \
        "clr r25 \n\t" \
        "clr r26 \n\t" \
        "clr r27 \n\t" \
		/*  begin loop  */ \
        "wait_while_%=: \n\t" /*  prevent name collision  */ \
        /*  use special high reg pairs to take space advantage of word operations  */ \
        /*  increment the 32 bits by 1  */ \
        "adiw r24,1 \n\t" \
        "adc r26,__zero_reg__ \n\t" \
        "adc r27,__zero_reg__ \n\t" \
        /*  branching  */ \
        "in __tmp_reg__,%[pinb] \n\t" /*  read pind, clock: 1  */ \
        "and __tmp_reg__,%[initial] \n\t" \
        "cp __tmp_reg__,__zero_reg__ \n\t" \
        "breq wait_while_%= \n\t" /*  continue loop if still unchanged  */ \
		/*  end loop  */ \
        /*  move back  */ \
        "movw %A[width],r24 \n\t" \
        "movw %C[width],r26 \n\t" \
        : [width] "=r" (width) \
        : [pinb] "I" (_SFR_IO_ADDR(sfr)), [initial] "r" (initial) \
        : "r24", "r25", "r26", "r27" /*  clobber list  */ \
    ); \
        width * 9 * 1000000 / F_CPU;                                                                                  \
    })

long microsecondsToCentimeters(long microseconds) {
    return microseconds / 29 / 2;
}

int calculateUltrasonic() {
    long duration, distance;

    DDRB |= 0b00001000;
    // pinMode(TRIGGER_PIN, OUTPUT);
    PORTB &= 0b11110111;
    // digitalWrite(TRIGGER_PIN, LOW);
    _delay_us(2);
    PORTB |= 0b00001000;
    // digitalWrite(TRIGGER_PIN, HIGH);
    _delay_us(10);
    PORTB &= 0b11110111;
    // digitalWrite(TRIGGER_PIN, LOW);
    DDRB &= 0b11101111;
    // pinMode(ECHO_PIN, INPUT);
    duration = pulseWidth(PINB, PINB4);
    distance = microsecondsToCentimeters(duration);

    // Serial.print(distance);
    // Serial.print("cm");
    if (distance <= 6) {
        // Serial.print(" Too Close!!!");
        near = 1;
    } else {
        near = 0;
    }

    return distance;
    // Serial.println();
    // delay(100);
}

void sendStatus() {
    // Implement code to send back a packet containing some parameters listed
    // below The params array stores the parameters with set packetType and
    // command files sendResponse sends out the packet.
    TPacket statusPacket;
    statusPacket.packetType = PACKET_TYPE_RESPONSE;
    statusPacket.command = RESP_STATUS;
    //  statusPacket.params[0] = leftForwardTicks;
    //  statusPacket.params[1] = rightForwardTicks;
    //  statusPacket.params[2] = leftReverseTicks;
    //  statusPacket.params[3] = rightReverseTicks;
    //  statusPacket.params[4] = leftForwardTicksTurns;
    //  statusPacket.params[5] = rightForwardTicksTurns;
    //  statusPacket.params[6] = leftReverseTicksTurns;
    //  statusPacket.params[7] = rightReverseTicksTurns;
    //  statusPacket.params[8] = forwardDist;
    //  statusPacket.params[9] = reverseDist;

    // S2/S3 levels define which set of photodiodes we are using.
    // LOW/LOW is for RED LOW/HIGH is for Blue and HIGH/HIGH is for green
    // Here we wait until "out" go LOW, we start measuring the duration and stop
    // when "out" is HIGH again

    // Bare-metal version
    PORTD &= 0b01101111;
    red = pulseWidth(PINB, PINB0);
    _delay_ms(20);
    PORTD |= 0b10000000;
    blue = pulseWidth(PINB, PINB0);
    _delay_ms(20);
    PORTD |= 0b00010000;
    green = pulseWidth(PINB, PINB0);
    _delay_ms(20);

    /*
// Sketch version
digitalWrite(S2, LOW);
digitalWrite(S3, LOW);
red = pulseIn(OUT, digitalRead(OUT) == HIGH ? LOW : HIGH);
delay(20);
digitalWrite(S3, HIGH);
blue = pulseIn(OUT, digitalRead(OUT) == HIGH ? LOW : HIGH);
delay(20);
digitalWrite(S2, HIGH);
green = pulseIn(OUT, digitalRead(OUT) == HIGH ? LOW : HIGH);
delay(20);
    */

    // Darren's calibration in lab
    // if (Red <= 30 && Green <=15 && Blue <= 15)
    // colour = 0; // strcpy(colour, "White");
    //  if (Red >= 30 && Blue >= 30 && Green >= 30)
    //    colour = 1; // strcpy(colour, "Red");
    //  else if (Green <= 29 && Red <= 29 && Blue <= 29)
    //    colour = 2; // strcpy(colour, "Green");

    // Default calibration
    if (red < blue && red <= green && red < 22 && green > 35) {
        colour = 1;  // red
    } else if (green < red && green - blue <= 8) {
        colour = 2;  // green
    } else {
        colour = 0;
    }
    statusPacket.params[0] = colour;
    // delay(200);

    statusPacket.params[1] = red;
    statusPacket.params[2] = green;
    statusPacket.params[3] = blue;

    int distance = calculateUltrasonic();
    statusPacket.params[4] = distance;

    sendResponse(&statusPacket);
}

void sendMessage(const char* message) {
    // Send text messages back to the Pi. Useful for debugging.
    TPacket messagePacket;

    messagePacket.packetType = PACKET_TYPE_MESSAGE;
    strncpy(messagePacket.data, message, MAX_STR_LEN);
    sendResponse(&messagePacket);
}

void dbprint(const char* format, ...) {
    va_list args;
    char buffer[128];

    va_start(args, format);
    vsprintf(buffer, format, args);
    sendMessage(buffer);
}

void sendBadPacket() {
    // Tell the Pi that it sent us a packet with a bad magic number.
    TPacket badPacket;

    badPacket.packetType = PACKET_TYPE_ERROR;
    badPacket.command = RESP_BAD_PACKET;
    sendResponse(&badPacket);
}

void sendBadChecksum() {
    // Tell the Pi that it sent us a packet with a bad checksum.
    TPacket badChecksum;

    badChecksum.packetType = PACKET_TYPE_ERROR;
    badChecksum.command = RESP_BAD_CHECKSUM;
    sendResponse(&badChecksum);
}

void sendBadCommand() {
    // Tell the Pi that we don't understand its command sent to us.
    TPacket badCommand;

    badCommand.packetType = PACKET_TYPE_ERROR;
    badCommand.command = RESP_BAD_COMMAND;
    sendResponse(&badCommand);
}

void sendBadResponse() {
    TPacket badResponse;

    badResponse.packetType = PACKET_TYPE_ERROR;
    badResponse.command = RESP_BAD_RESPONSE;
    sendResponse(&badResponse);
}

void sendOK() {
    TPacket okPacket;

    okPacket.packetType = PACKET_TYPE_RESPONSE;
    okPacket.command = RESP_OK;
    sendResponse(&okPacket);
}

// Setup and start codes for external interrupts and pullup resistors.
// Enable pull up resistors on pins 2 and 3
void enablePullups() {
    // Enable the pull-up resistors on pins 2 and 3. These are pins PD2 and PD3
    // respectively. Set bits 2 and 3 in DDRD to 0 to make them inputs.
    DDRD &= 0b11110011;

    // PIND |= 0b110;
    PORTD |= 0b00001100;
}

// Functions to be called by INT0 and INT1 ISRs.
void leftISR() {
    if (dir == FORWARD) {
        leftForwardTicks++;
        forwardDist = (unsigned long)((float)leftForwardTicks / COUNTS_PER_REV *
                                      WHEEL_CIRC);
    } else if (dir == BACKWARD) {
        leftReverseTicks++;
        reverseDist = (unsigned long)((float)leftReverseTicks / COUNTS_PER_REV *
                                      WHEEL_CIRC);
    } else if (dir == LEFT) {
        leftReverseTicksTurns++;
    } else if (dir == RIGHT) {
        leftForwardTicksTurns++;
    }
}

void rightISR() {
    if (dir == FORWARD) {
        rightForwardTicks++;
    } else if (dir == BACKWARD) {
        rightReverseTicks++;
    } else if (dir == LEFT) {
        rightForwardTicksTurns++;
    } else if (dir == RIGHT) {
        rightReverseTicksTurns++;
    }
}

// Set up the external interrupt pins INT0 and INT1 for falling edge triggered.
void setupEINT() {
    // Configure pins 2 and 3 to be falling edge triggered.
    // Enable INT0 and INT1 interrupts.
    EIMSK |= 0b11;
    EICRA = 0b1010;
}

// Implement the external interrupt ISRs.
// INT0 ISR should call leftISR while INT1 ISR should call rightISR.
ISR(INT0_vect) {
    leftISR();
}

ISR(INT1_vect) {
    rightISR();
}

// Setup and start codes for serial communications
// Set up the serial connection.
void setupSerial() {
    initBuffer(&sendbuf, BUF_LEN);
    initBuffer(&recvbuf, BUF_LEN);

    // async
    cbi(UCSR0C, UMSEL00);
    cbi(UCSR0C, UMSEL01);
    cbi(UCSR0C, UCPOL0);

    // parity: none
    cbi(UCSR0C, UPM00);
    cbi(UCSR0C, UPM01);

    // stop bit: 1
    cbi(UCSR0C, USBS0);

    // data size: 8
    sbi(UCSR0C, UCSZ00);
    sbi(UCSR0C, UCSZ01);
    cbi(UCSR0B, UCSZ02);
    cbi(UCSR0B, TXB80);

    // baud rate: 9600
    uint16_t b = F_CPU / 16 / 9600 - 1;
    UBRR0H = (uint8_t)(b >> 8);
    UBRR0L = (uint8_t)b;

    // single processor, normal transmission speed
    cbi(UCSR0A, MPCM0);
    cbi(UCSR0A, U2X0);
}

ISR(USART_RX_vect) {
    unsigned char data = UDR0;

    writeBuffer(&recvbuf, data);
}

ISR(USART_UDRE_vect) {
    unsigned char data;

    if (readBuffer(&sendbuf, &data) == BUFFER_OK) {
        UDR0 = data;
    } else {
        cbi(UCSR0B, UDRIE0);
    }
}

// Start the serial connection.
void startSerial() {
    // enable rx and tx
    sbi(UCSR0B, TXEN0);
    sbi(UCSR0B, RXEN0);

    // enable interrupts
    cbi(UCSR0B, TXCIE0);
    sbi(UCSR0B, RXCIE0);
    cbi(UCSR0B, UDRIE0);  // data reg empty interrupt
}

// Alex's motor drivers.
// Set up the PWMs to drive the motors.
void setupMotors() {
    // set as output
    DDRB |= 0b110;
    DDRD |= 0b1100000;

    // No prescaling, max freq
    sbi(TCCR0B, CS00);
    cbi(TCCR0B, CS01);
    cbi(TCCR0B, CS02);
    sbi(TCCR1B, CS10);
    cbi(TCCR1B, CS11);
    cbi(TCCR1B, CS12);

    // initialise counter
    TCNT0 = 0;
    TCNT1L = 0;
    TCNT1H = 0;
    OCR0A = 0;
    OCR0B = 0;
    OCR1AL = 0;
    OCR1AH = 0;
    OCR1BL = 0;
    OCR1BH = 0;

    // phase correct
    sbi(TCCR0A, WGM00);
    cbi(TCCR0A, WGM01);
    cbi(TCCR0B, WGM02);
    sbi(TCCR1A, WGM10);
    cbi(TCCR1A, WGM11);
    cbi(TCCR1B, WGM12);
    cbi(TCCR1B, WGM13);

    /* Our motor set up is:
       A1IN - Pin 5, PD5, OC0B
       A2IN - Pin 6, PD6, OC0A
       B1IN - Pin 9, PB1, OC1A
       B2IN - Pin 10, PB2, OC1B
    */
}

// Start the PWM for Alex's motors.
void startMotors() {
    // clear on compare, non-inverted
    cbi(TCCR0A, COM0A0);
    sbi(TCCR0A, COM0A1);
    cbi(TCCR0A, COM0B0);
    sbi(TCCR0A, COM0B1);
    cbi(TCCR1A, COM1A0);
    sbi(TCCR1A, COM1A1);
    cbi(TCCR1A, COM1B0);
    sbi(TCCR1A, COM1B1);
}

// Convert percentages to PWM values
int pwmVal(float speed) {
    if (speed < 0.0) {
        speed = 0;
    }

    if (speed > 100.0) {
        speed = 100.0;
    }

    return (int)((speed / 100.0) * 255.0);
}

// Move Alex forward "dist" cm at speed "speed".
// "speed" is expressed as a percentage. E.g. 50 is
// move forward at half speed.
// Specifying a distance of 0 means Alex will
// continue moving forward indefinitely.
void forward(float dist, float speed) {
    // Code tells us how far to move
    if (dist > 0) {
        deltaDist = dist;
    } else {
        deltaDist = 9999999;
    }

    newDist = forwardDist + deltaDist;

    dir = FORWARD;
    int val = pwmVal(speed);

    // LF = Left forward pin, LR = Left reverse pin
    // RF = Right forward pin, RR = Right reverse pin
    // This will be replaced later with bare-metal code.
    pwmWrite(LF, val);
    pwmWrite(RF, val - 5);
    pwmWrite(LR, 0);
    pwmWrite(RR, 0);
}

// Reverse Alex "dist" cm at speed "speed".
// "speed" is expressed as a percentage. E.g. 50 is
// reverse at half speed.
// Specifying a distance of 0 means Alex will
// continue reversing indefinitely.
void reverse(float dist, float speed) {
    // Code tells us how far to move
    if (dist > 0) {
        deltaDist = dist;
    } else {
        deltaDist = 9999999;
    }

    newDist = reverseDist + deltaDist;

    dir = BACKWARD;

    int val = pwmVal(speed);

    // LF = Left forward pin, LR = Left reverse pin
    // RF = Right forward pin, RR = Right reverse pin
    // This will be replaced later with bare-metal code.
    pwmWrite(LR, val);
    pwmWrite(RR, val - 5);
    pwmWrite(LF, 0);
    pwmWrite(RF, 0);
}

// New function to estimate number of wheel leftTicks needed to turn an angle
unsigned long computeDeltaTicks(float ang) {
    // We will assume that angular distance  moved == linear distance moved in
    // one wheels revolution.This is (probably) incorrect but simplifes
    // calculation. Number of wheel revs to make one full 360 turn is
    // alexCirc/WHEEL_CIRC This is for 360. For ang degrees it will be(ang
    // *alexCirc)/(360 * WHEEL_CIRC). To convert to ticks, we multiply by
    // COUNTS_PER_REV.

    unsigned long ticks =
        (unsigned long)((ang * alexCirc * COUNTS_PER_REV) / (360 * WHEEL_CIRC));

    return ticks;
}

// Turn Alex left "ang" degrees at speed "speed".
// "speed" is expressed as a percentage. E.g. 50 is
// turn left at half speed.
// Specifying an angle of 0 degrees will cause Alex to
// turn left indefinitely.
void left(float ang, float speed) {
    int val = pwmVal(speed);

    dir = LEFT;

    if (ang == 0) {
        deltaTicks = 99999999;
    } else {
        deltaTicks = computeDeltaTicks(ang);
    }

    targetTicks = leftReverseTicksTurns + deltaTicks;

    pwmWrite(LR, val);
    pwmWrite(RF, val - 5);
    pwmWrite(LF, 0);
    pwmWrite(RR, 0);
}

// Turn Alex right "ang" degrees at speed "speed".
// "speed" is expressed as a percentage. E.g. 50 is
// turn left at half speed.
// Specifying an angle of 0 degrees will cause Alex to
// turn right indefinitely.
void right(float ang, float speed) {
    int val = pwmVal(speed);

    dir = RIGHT;

    if (ang == 0) {
        deltaTicks = 99999999;
    } else {
        deltaTicks = computeDeltaTicks(ang);
    }

    targetTicks = rightReverseTicksTurns + deltaTicks;

    // To turn right we reverse the right wheel and move
    // the left wheel forward.
    pwmWrite(RR, val - 7);
    pwmWrite(LF, val);
    pwmWrite(LR, 0);
    pwmWrite(RF, 0);
}

// Stop Alex.
void stop() {
    pwmWrite(LF, 0);
    pwmWrite(LR, 0);
    pwmWrite(RF, 0);
    pwmWrite(RR, 0);
}

// Alex's setup and run codes

// Clear all the counters
void clearCounters() {
    leftForwardTicks = 0;
    rightForwardTicks = 0;
    leftReverseTicks = 0;
    rightReverseTicks = 0;

    leftForwardTicksTurns = 0;
    rightForwardTicksTurns = 0;
    leftReverseTicksTurns = 0;
    rightReverseTicksTurns = 0;

    leftRevs = 0;
    rightRevs = 0;
    forwardDist = 0;
    reverseDist = 0;
}

// Clears one particular counter
void clearOneCounter(int which) {
    clearCounters();
}

// Intialize Alex's internal states
void initializeState() {
    clearCounters();
}

void handleCommand(TPacket* command) {
    switch (command->command) {
        // For movement commands, param[0] = distance, param[1] = speed.
        case COMMAND_FORWARD:
            sendOK();
            forward((float)command->params[0], (float)command->params[1]);
            break;

        case COMMAND_REVERSE:
            sendOK();
            reverse((float)command->params[0], (float)command->params[1]);
            break;

        case COMMAND_TURN_LEFT:
            sendOK();
            left((float)command->params[0], (float)command->params[1]);
            break;

        case COMMAND_TURN_RIGHT:
            sendOK();
            right((float)command->params[0], (float)command->params[1]);
            break;

        case COMMAND_STOP:
            sendOK();
            stop();
            break;

        case COMMAND_GET_STATS:
            sendOK();
            sendStatus();
            break;

        case COMMAND_CLEAR_STATS:
            sendOK();
            clearOneCounter(command->params[0]);
            break;

        default:
            sendBadCommand();
    }
}

void waitForHello() {
    int exit = 0;

    while (!exit) {
        TPacket hello;
        TResult result;

        do {
            result = readPacket(&hello);
        } while (result == PACKET_INCOMPLETE);

        if (result == PACKET_OK) {
            if (hello.packetType == PACKET_TYPE_HELLO) {
                sendOK();
                exit = 1;
            } else {
                sendBadResponse();
            }
        } else if (result == PACKET_BAD) {
            sendBadPacket();
        } else if (result == PACKET_CHECKSUM_BAD) {
            sendBadChecksum();
        }
    }  // !exit
}

void setupColourSensor() {
    DDRD |= 0b10010011;
    DDRB &= 0b11111110;
    PORTD |= 0b00000011;
    /*
pinMode(S0, OUTPUT);
pinMode(S1, OUTPUT);
pinMode(S2, OUTPUT);
pinMode(S3, OUTPUT);
pinMode(OUT, INPUT);

digitalWrite(S0, HIGH);
digitalWrite(S1, HIGH);
    */
}

void setupUltrasonicSensor() {
    // pinMode(trigPin, OUTPUT);
    // pinMode(echoPin, INPUT);
}

void setup() {
    // put your setup code here, to run once:
    alexDiagonal =
        sqrt((ALEX_LENGTH * ALEX_LENGTH) + (ALEX_BREADTH * ALEX_BREADTH));

    alexCirc = M_PI * alexDiagonal;

    cli();
    setupEINT();
    setupSerial();
    startSerial();
    setupMotors();
    startMotors();
    enablePullups();
    initializeState();
    setupColourSensor();
    setupUltrasonicSensor();
    // setupPowerSaving();
    sei();
}

void handlePacket(TPacket* packet) {
    switch (packet->packetType) {
        case PACKET_TYPE_COMMAND:
            handleCommand(packet);
            break;

        case PACKET_TYPE_RESPONSE:
            break;

        case PACKET_TYPE_ERROR:
            break;

        case PACKET_TYPE_MESSAGE:
            break;

        case PACKET_TYPE_HELLO:
            break;
    }
}

void loop() {
    // Code to run repeatedly:
    PORTD |= 0b11;

    TPacket recvPacket;  // This holds commands from the Pi

    TResult result = readPacket(&recvPacket);

    if (result == PACKET_OK) {
        handlePacket(&recvPacket);
    } else if (result == PACKET_BAD) {
        sendBadPacket();
    } else if (result == PACKET_CHECKSUM_BAD) {
        sendBadChecksum();
    }

    if (deltaDist > 0) {
        if (dir == FORWARD) {
            if (forwardDist >= newDist || near == 1) {
                deltaDist = 0;
                newDist = 0;
                int temp = calculateUltrasonic();  // this is wrong
                stop();
            }
        } else if (dir == BACKWARD) {
            if (reverseDist >= newDist) {
                deltaDist = 0;
                newDist = 0;
                stop();
            }
        } else if (dir == STOP) {
            deltaDist = 0;
            newDist = 0;
            stop();
            // putArduinoToIdle();
        }
    }

    if (deltaTicks > 0) {
        if (dir == LEFT) {
            if (leftReverseTicksTurns >= targetTicks) {
                deltaTicks = 0;
                targetTicks = 0;
                stop();
            }
        } else if (dir == RIGHT) {
            if (rightReverseTicksTurns >= targetTicks) {
                deltaTicks = 0;
                targetTicks = 0;
                stop();
            }
        } else if (dir == STOP) {
            deltaTicks = 0;
            targetTicks = 0;
            stop();
            // putArduinoToIdle();
        }
    }
}

#ifndef ARDUINO
int main() {
    setup();
    while (1) {
        // writeSerial("please", 7);
        loop();
    }
}
#endif
