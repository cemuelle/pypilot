/*
 * Minimal DIR+PWM motor driver for pypilot motor controller.
 * DIR: D3 (0 = reverse, 1 = forward)
 * PWM: D9 (0 = brake/coast, higher = faster)
 */

#include <Arduino.h>
#include <stdint.h>
#include <HardwareSerial.h>
#include "crc.h"

#define DIV_CLOCK 2  // 1=16mhz, 2=8mhz, 4=4mhz

#define DIR_PIN 3
#define PWM_PIN 9

#define led_pin 13 // led is on when engaged

enum commands {
    COMMAND_CODE=0xc7,
    RESET_CODE=0xe7,
    MAX_SLEW_CODE=0x71,
    DISENGAGE_CODE=0x68
};

enum results {FLAGS_CODE=0x8f};

enum {
    SYNC=1,
    ENGAGED=8,
    INVALID=16,
    REBOOTED=32768
};

uint16_t flags = REBOOTED;
uint8_t serialin;

uint8_t timeout;
uint16_t serial_data_timeout;

uint8_t in_bytes[3];
uint8_t sync_b = 0, in_sync_count = 0;

uint8_t out_sync_b = 0;
uint8_t crcbytes[3];

uint16_t lastpos = 1000; // command range 0..2000, 1000=neutral
uint16_t command_value = 1000;
uint8_t max_slew_speed = 50, max_slew_slow = 75;

void setup_pwm_timer()
{
    // TIMER1 Fast PWM on OC1A (D9)
    TCNT1 = 0;
    TCCR1A = _BV(COM1A1) | _BV(WGM11);        // non-inverted
    TCCR1B = _BV(WGM13) | _BV(WGM12) | _BV(CS10); // prescaler=1
    ICR1 = 16000 / DIV_CLOCK;  // ~1kHz
    TIMSK1 = 0;
}

void position(uint16_t value)
{
    lastpos = value;
    uint16_t mag = abs((int)value - 1000);
    OCR1A = mag * 16 / DIV_CLOCK;

    if(value > 1040) {
        digitalWrite(DIR_PIN, HIGH); // forward
    } else if(value < 960) {
        digitalWrite(DIR_PIN, LOW); // reverse
    } else {
        OCR1A = 0; // brake/coast when PWM=0
    }
}

void stop()
{
    position(1000);
    command_value = 1000;
}

void update_command()
{
    int16_t speed_rate = max_slew_speed;
    int16_t slow_rate = max_slew_slow;
    uint16_t cur_value = lastpos;

    int16_t diff = (int)command_value - (int)cur_value;

    if(diff > 0) {
        if(cur_value < 1000) {
            if(diff > slow_rate)
               diff = slow_rate;
        } else if(diff > speed_rate)
            diff = speed_rate;
    } else {
        if(cur_value > 1000) {
            if(diff < -slow_rate)
                diff = -slow_rate;
        } else if(diff < -speed_rate)
            diff = -speed_rate;
    }

    position(cur_value + diff);
}

void disengage()
{
    stop();
    if(flags | ENGAGED) {
        flags &= ~ENGAGED;
        timeout = 30; // detach in about 62ms
    }
}

void detach()
{
    TIMSK1 = 0;
    TCCR1A = 0;
    TCCR1B = 0;
    digitalWrite(PWM_PIN, LOW);
    digitalWrite(DIR_PIN, LOW);
    digitalWrite(led_pin, LOW);
    timeout = 33;
}

void engage()
{
    if(flags & ENGAGED)
        return;

    setup_pwm_timer();
    position(1000);

    digitalWrite(led_pin, HIGH);
    flags |= ENGAGED;
}

void process_packet()
{
    flags |= SYNC;
    uint16_t value = in_bytes[1] | in_bytes[2]<<8;
    switch(in_bytes[0]) {
    case RESET_CODE:
        break;
    case COMMAND_CODE:
        timeout = 0;
        if(serialin < 12)
            serialin+=4;
        if(value > 2000) {
            // ignore invalid range
        } else {
            command_value = value;
            engage();
        }
        break;
    case DISENGAGE_CODE:
        if(serialin < 12)
            serialin+=4;
        disengage();
        break;
    case MAX_SLEW_CODE:
        max_slew_speed = in_bytes[1];
        max_slew_slow = in_bytes[2];
        if(max_slew_speed > 250)
            max_slew_speed = 250;
        if(max_slew_slow > 250)
            max_slew_slow = 250;
        if(max_slew_speed < 1)
            max_slew_speed = 1;
        if(max_slew_slow < 1)
            max_slew_slow = 1;
        break;
    }
}

void setup()
{
    PCICR = 0;
    PCMSK2 = 0;

    cli();
    CLKPR = _BV(CLKPCE);
#if DIV_CLOCK==4
    CLKPR = _BV(CLKPS1); // divide by 4
#elif DIV_CLOCK==2
    CLKPR = _BV(CLKPS0); // divide by 2
#else
    CLKPR = 0; // divide by 1
#endif
    sei();

    Serial.begin(38400*DIV_CLOCK);

    pinMode(DIR_PIN, OUTPUT);
    pinMode(PWM_PIN, OUTPUT);
    digitalWrite(DIR_PIN, LOW);
    digitalWrite(PWM_PIN, LOW);

    digitalWrite(led_pin, LOW);
    pinMode(led_pin, OUTPUT);

    // use timer0 as timeout counter
    TCNT0 = 0;
    TCCR0A = 0;
    TCCR0B = _BV(CS02) | _BV(CS00); // divide 1024

    serialin = 0;
    timeout = 0;
    serial_data_timeout = 250;
}

void loop()
{
    TIMSK0 = 0;
    uint8_t ticks = TCNT0;
    if(ticks > 78) {
        static uint8_t timeout_d;
        if(++timeout_d >= 4/DIV_CLOCK) {
            if(flags & ENGAGED)
                update_command();
            timeout_d = 0;
            timeout++;
            serial_data_timeout++;
        }
        TCNT0 -= 78;
    }

    if(timeout == 30)
        disengage();
    if(timeout > 32)
        detach();

    while(Serial.available()) {
        uint8_t c = Serial.read();
        serial_data_timeout = 0;
        if(sync_b < 3) {
            in_bytes[sync_b] = c;
            sync_b++;
        } else {
            if(c == crc8(in_bytes, 3)) {
                if(in_sync_count >= 2) {
                    process_packet();
                } else
                    in_sync_count++;

                sync_b = 0;
                flags &= ~INVALID;
            } else {
                flags &= ~SYNC;
                stop();
                in_sync_count = 0;
                in_bytes[0] = in_bytes[1];
                in_bytes[1] = in_bytes[2];
                in_bytes[2] = c;
                flags |= INVALID;
            }
            break;
        }
    }

    // output flags only (single frame)
    switch(out_sync_b) {
    case 0: {
        if(serialin < 4)
            return;
        uint16_t v = flags;
        flags &= ~REBOOTED;
        crcbytes[0] = FLAGS_CODE;
        crcbytes[1] = v;
        crcbytes[2] = v>>8;
        // fall through
    }
    case 1: case 2:
        Serial.write(crcbytes[out_sync_b]);
        out_sync_b++;
        break;
    case 3:
        Serial.write(crc8(crcbytes, 3));
        out_sync_b = 0;
        break;
    }
}
