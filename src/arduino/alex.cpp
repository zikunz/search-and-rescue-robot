#if !(defined(__AVR) || defined(AVR) || defined(__AVR_ATmega328P__))
#define __AVR_ATmega328P__
#endif
#include <avr/interrupt.h>
#include <avr/io.h>
#include <inttypes.h>
#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include "buffer.h"
#include "constants.h"
#include "packet.h"
#include "serialize.h"

#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#define NOT_ON_TIMER 0
#define BUF_LEN 512
#define TIMER0A 1
#define TIMER0B 2
#define TIMER1A 3
#define TIMER1B 4
#define TIMER2A 6
#define TIMER2B 7

volatile TBuffer sendbuf, recvbuf;

void pwmWrite(uint8_t pin, int val) {
    volatile uint8_t* timer_reg;
    int timer_pos;
    volatile uint8_t* timer_comp;
    volatile uint8_t* port_reg;
    int pin_pos;
    switch (pin) {
        case 5:
            timer_reg = &TCCR0A;
            timer_pos = COM0B1;
            timer_comp = &OCR0B;
            port_reg = &PORTD;
            pin_pos = 5;
            break;
        case 6:
            timer_reg = &TCCR0A;
            timer_pos = COM0A1;
            timer_comp = &OCR0A;
            port_reg = &PORTD;
            pin_pos = 6;
            break;
        case 9:
            timer_reg = &TCCR1A;
            timer_pos = COM1A1;
            timer_comp = &OCR1A;
            port_reg = &PORTB;
            pin_pos = 1;
            break;
        case 10:
            timer_reg = &TCCR1A;
            timer_pos = COM1B1;
            timer_comp = &OCR1B;
            port_reg = &PORTB;
            pin_pos = 2;
            break;
    }
    cbi(timer_reg, timer_pos);  // disable PWM
    if (val == 0) {
        cbi(port_reg, pin_pos);
        return;
    }
    if (val == 255) {
        sbi(port_reg, pin_pos);
        return;
    }
    sbi(timer_reg, timer_pos);
    *timer_comp = val;
}

typedef enum {
    STOP = 0,
    FORWARD = 1,
    BACKWARD = 2,
    LEFT = 3,
    RIGHT = 4
} TDirection;

volatile TDirection dir = STOP;

/*
 * Alex's configuration constants
 */

// Number of ticks per revolution from the
// wheel encoder.

#define COUNTS_PER_REV 200

// Wheel circumference in cm.
// We will use this to calculate forward/backward distance traveled
// by taking revs * WHEEL_CIRC

#define WHEEL_CIRC 20.42

// Motor control pins. You need to adjust these till
// Alex moves in the correct direction
#define LF 6            // Left forward pin
#define LR 5            // Left reverse pin
#define RF 10           // Right forward pin
#define RR 9            // Right reverse pin
#define ALEX_LENGTH 16  // alex length
#define ALEX_BREADTH 6  // alex breath

// Alex's diagonal. We compute and store this value once
// since it is expensive to compute and really does not change
float alexDiagonal = 0.0;
float alexCirc = 0.0;

/*
 *    Alex's State Variables
 */

// Store the ticks from Alex's left and
// right encoders.
volatile unsigned long leftForwardTicks = 0;
volatile unsigned long rightForwardTicks = 0;
volatile unsigned long leftReverseTicks = 0;
volatile unsigned long rightReverseTicks = 0;

volatile unsigned long leftForwardTicksTurns = 0;
volatile unsigned long rightForwardTicksTurns = 0;
volatile unsigned long leftReverseTicksTurns = 0;
volatile unsigned long rightReverseTicksTurns = 0;

// Store the revolutions on Alex's left
// and right wheels
volatile unsigned long leftRevs = 0;
volatile unsigned long rightRevs = 0;

// Forward and backward distance traveled
volatile unsigned long forwardDist = 0;
volatile unsigned long reverseDist = 0;

// Variables to keep track of whether we have moved a command distance//
volatile unsigned long deltaDist = 0;
volatile unsigned long newDist = 0;

// Variables to keep track of our turning angle
volatile unsigned long deltaTicks = 0;
volatile unsigned long targetTicks = 0;

// Read the serial port. Returns the read character in
// ch if available. Also returns TRUE if ch is valid.
// This will be replaced later with bare-metal code.

int readSerial(char* buffer) {
    int count = 0;
    for (count = 0; dataAvailable(&recvbuf); count += 1) {
        readBuffer(&recvbuf, (unsigned char*)&buffer[count]);
    }
    return count;
}

// Write to the serial port. Replaced later with
// bare-metal code

void writeSerial(const char* buffer, int len) {
    for (int i = 0; i < len; i += 1) {
        writeBuffer(&sendbuf, buffer[i]);
    }
    sbi(UCSR0B, UDRIE0);
}

/*
 *
 * Alex Communication Routines.
 *
 */

TResult readPacket(TPacket* packet) {
    // Reads in data from the serial port and
    // deserializes it.Returns deserialized
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
    // Takes a packet, serializes it then sends it out
    // over the serial port.
    char buffer[PACKET_SIZE];
    int len;

    len = serialize(buffer, packet, sizeof(TPacket));
    writeSerial(buffer, len);
}

void sendStatus() {
    // Implement code to send back a packet containing key
    // information like leftTicks, rightTicks, leftRevs, rightRevs
    // forwardDist and reverseDist
    // Use the params array to store this information, and set the
    // packetType and command files accordingly, then use sendResponse
    // to send out the packet. See sendMessage on how to use sendResponse.
    //
    TPacket statusPacket;
    statusPacket.packetType = PACKET_TYPE_RESPONSE;
    statusPacket.command = RESP_STATUS;
    statusPacket.params[0] = leftForwardTicks;
    statusPacket.params[1] = rightForwardTicks;
    statusPacket.params[2] = leftReverseTicks;
    statusPacket.params[3] = rightReverseTicks;
    statusPacket.params[4] = leftForwardTicksTurns;
    statusPacket.params[5] = rightForwardTicksTurns;
    statusPacket.params[6] = leftReverseTicksTurns;
    statusPacket.params[7] = rightReverseTicksTurns;
    statusPacket.params[8] = forwardDist;
    statusPacket.params[9] = reverseDist;
    sendResponse(&statusPacket);
}

void sendMessage(const char* message) {
    // Sends text messages back to the Pi. Useful
    // for debugging.

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
    // Tell the Pi that it sent us a packet with a bad
    // magic number.

    TPacket badPacket;
    badPacket.packetType = PACKET_TYPE_ERROR;
    badPacket.command = RESP_BAD_PACKET;
    sendResponse(&badPacket);
}

void sendBadChecksum() {
    // Tell the Pi that it sent us a packet with a bad
    // checksum.

    TPacket badChecksum;
    badChecksum.packetType = PACKET_TYPE_ERROR;
    badChecksum.command = RESP_BAD_CHECKSUM;
    sendResponse(&badChecksum);
}

void sendBadCommand() {
    // Tell the Pi that we don't understand its
    // command sent to us.

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

/*
 * Setup and start codes for external interrupts and
 * pullup resistors.
 *
 */
// Enable pull up resistors on pins 2 and 3
void enablePullups() {
    DDRD &= 0b11110011;
    // PIND |= 0b110;
    PORTD |= 0b00001100;
    // Use bare-metal to enable the pull-up resistors on pins
    // 2 and 3. These are pins PD2 and PD3 respectively.
    // We set bits 2 and 3 in DDRD to 0 to make them inputs.
}

void printTicks() {
    dbprint("%d, %d, %d, %d, %d, %d, %d, %d", leftForwardTicks,
            leftReverseTicks, leftForwardTicksTurns, leftReverseTicksTurns,
            rightForwardTicks, rightReverseTicks, rightForwardTicksTurns,
            rightReverseTicksTurns);
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

// Set up the external interrupt pins INT0 and INT1
// for falling edge triggered. Use bare-metal.
void setupEINT() {
    // Use bare-metal to configure pins 2 and 3 to be
    // falling edge triggered. Remember to enable
    // the INT0 and INT1 interrupts.
    EIMSK |= 0b11;
    EICRA = 0b1010;
}

// Implement the external interrupt ISRs below.
// INT0 ISR should call leftISR while INT1 ISR
// should call rightISR.

ISR(INT0_vect) {
    leftISR();
}

ISR(INT1_vect) {
    rightISR();
}

// Implement INT0 and INT1 ISRs above.

/*
 * Setup and start codes for serial communications
 *
 */
// Set up the serial connection. For now we are using
// Arduino Wiring, you will replace this later
// with bare-metal code.
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

// Start the serial connection. For now we are using
// Arduino wiring and this function is empty. We will
// replace this later with bare-metal code.

void startSerial() {
    // enable rx and tx
    sbi(UCSR0B, TXEN0);
    sbi(UCSR0B, RXEN0);
    // enable interrupts
    cbi(UCSR0B, TXCIE0);
    sbi(UCSR0B, RXCIE0);
    sbi(UCSR0B, UDRIE0);  // data reg empty interrupt
}

/*
 * Alex's motor drivers.
 *
 */

// Set up Alex's motors. Right now this is empty, but
// later you will replace it with code to set up the PWMs
// to drive the motors.
void setupMotors() {
    DDRB |= 0b110;
    DDRD |= 0b110000;
    /* Our motor set up is:
     *    A1IN - Pin 5, PD5, OC0B
     *    A2IN - Pin 6, PD6, OC0A
     *    B1IN - Pin 10, PB2, OC1B
     *    B2In - pIN 11, PB3, OC2A
     */
}

// Start the PWM for Alex's motors.
// We will implement this later. For now it is
// blank.
void startMotors() {}

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
    dbprint("%d", val);

    // For now we will ignore dist and move
    // forward indefinitely. We will fix this
    // in Week 9.

    // LF = Left forward pin, LR = Left reverse pin
    // RF = Right forward pin, RR = Right reverse pin
    // This will be replaced later with bare-metal code.

    pwmWrite(LF, val);
    pwmWrite(RF, val);
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

    // For now we will ignore dist and
    // reverse indefinitely. We will fix this
    // in Week 9.

    // LF = Left forward pin, LR = Left reverse pin
    // RF = Right forward pin, RR = Right reverse pin
    // This will be replaced later with bare-metal code.
    pwmWrite(LR, val);
    pwmWrite(RR, val);
    pwmWrite(LF, 0);
    pwmWrite(RF, 0);
}
// New function to estimate number of wheel leftTicks
// needed to turn an angle
unsigned long computeDeltaTicks(float ang) {
    // We will assume that angular distance  moved=linear distance moved in one
    // wheels revolution.This is (probably) incorrect but simplifes calculation.
    //# of wheel revs to make one full 360 turn is alexCirc/WHEEL_CIRC
    // This is for 360.For ang degrees it will be(ang *alexCirc)/(360 *
    // WHEEL_CIRC)
    // To convert to ticks, we multiply by COUNTS_PER_REV.

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

    // For now we will ignore ang. We will fix this in Week 9.
    // We will also replace this code with bare-metal later.
    // To turn left we reverse the left wheel and move
    // the right wheel forward.
    pwmWrite(LR, val);
    pwmWrite(RF, val);
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

    // For now we will ignore ang. We will fix this in Week 9.
    // We will also replace this code with bare-metal later.
    // To turn right we reverse the right wheel and move
    // the left wheel forward.
    pwmWrite(RR, val);
    pwmWrite(LF, val);
    pwmWrite(LR, 0);
    pwmWrite(RF, 0);
}

// Stop Alex. To replace with bare-metal code later.
void stop() {
    dbprint("Stop");
    pwmWrite(LF, 0);
    pwmWrite(LR, 0);
    pwmWrite(RF, 0);
    pwmWrite(RR, 0);
}

/*
 * Alex's setup and run codes
 *
 */

// Clears all our counters
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
// Intialize Vincet's internal states

void initializeState() {
    clearCounters();
}

void handleCommand(TPacket* command) {
    switch (command->command) {
        // For movement commands, param[0] = distance, param[1] = speed.
        case COMMAND_FORWARD:
            sendOK();
            dbprint("%lu,%lu", command->params[0], command->params[1]);
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
    // Uncomment the code below for Step 2 of Activity 3 in Week 8 Studio 2

    // Uncomment the code below for Week 9 Studio 2

    // put your main code here, to run repeatedly:
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
            if (forwardDist >= newDist) {
                deltaDist = 0;
                newDist = 0;
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
        }
    }

    if (deltaTicks > 0) {
        if (dir == LEFT) {
            if (leftForwardTicksTurns >= targetTicks) {
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
        }
    }
}

int main() {
    setup();
    while (1) {
        // writeSerial("please", 7);
        loop();
    }
}
