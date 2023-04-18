#include <Arduino.h>

/*
    https://www.mikrocontroller.net/topic/101728
    https://sourceforge.net/p/amforth/mailman/message/34859069/

    Kenwood XS/XS8/SL16 system remote control
    pinout: sleeve = GND ring = KEN_DATA tip = KEN_BUSY

               R
    pin3 o---[   ]----o tip
              330

               R
    pin2 o---[   ]----o ring
              330
*/

const uint8_t KEN_DATA = 16; // ring
const uint8_t KEN_BUSY = 17; // tip

#define KEN_STATE_IDLE 0
#define KEN_STATE_START_BIT_L 1
#define KEN_STATE_START_BIT_H 2
#define KEN_STATE_BIT_L 3
#define KEN_STATE_BIT_H 4

uint8_t interface = 8;

// timing XS8 in ms
const float START_BIT_L_8 = 1.0f;
const float START_BIT_H_8 = 10.0f;
const float BIT_0L_8 = 10.0f;
const float BIT_1L_8 = 5.0f;
const float BIT_H_8 = 5.0f;

// timing SL16 in ms
const float START_BIT_L_16 = 5.0f;
const float START_BIT_H_16 = 5.0f;
const float BIT_0L_16 = 5.0f;
const float BIT_1L_16 = 2.5f;
const float BIT_H_16 = 2.5f;

// delay in µs
uint32_t START_BIT_L;
uint32_t START_BIT_H;
uint32_t BIT_0L;
uint32_t BIT_1L;
uint32_t BIT_H;

uint16_t ken_rx_buffer;
uint32_t ken_rx_buffer_pos = 0;
uint32_t ken_rx_state = KEN_STATE_IDLE;
uint32_t ken_rx_last_micros = 0;

static void IRAM_ATTR ken_rx_busy_isr()
{
    if (digitalRead(KEN_BUSY))
    {
        ken_rx_buffer = 0;
        ken_rx_buffer_pos = 0;
        ken_rx_last_micros = micros();
        ken_rx_state = KEN_STATE_START_BIT_L;
    }
    else
    {
        ken_rx_state = KEN_STATE_IDLE;
    }
}

static bool ken_rx_is_micros(uint32_t delta, uint32_t expeced)
{
    if (delta > expeced - 800 && delta < expeced + 800)
    {
        return true;
    }

    return false;
}

static void ken_rx_queue_bit(int level)
{
    ken_rx_buffer <<= 1;
    ken_rx_buffer |= level ? 1 : 0;
    ken_rx_buffer_pos++;
}

static void IRAM_ATTR ken_rx_data_isr()
{
    if (ken_rx_state == KEN_STATE_IDLE)
    {
        return;
    }
    uint32_t cur_micros = micros();
    uint32_t delta = cur_micros - ken_rx_last_micros;
    ken_rx_last_micros = cur_micros;

    switch (ken_rx_state)
    {
    case KEN_STATE_START_BIT_L:
        if (ken_rx_is_micros(delta, START_BIT_L) && digitalRead(KEN_DATA))
        {
            ken_rx_state = KEN_STATE_START_BIT_H;
        }
        else
        {
            ken_rx_state = KEN_STATE_IDLE;
        }
        break;

    case KEN_STATE_START_BIT_H:
        if (ken_rx_is_micros(delta, START_BIT_H))
        {
            ken_rx_state = KEN_STATE_BIT_L;
        }
        else
        {
            ken_rx_state = KEN_STATE_IDLE;
        }
        break;

    case KEN_STATE_BIT_L:
        if (ken_rx_is_micros(delta, BIT_1L))
        {
            ken_rx_queue_bit(1);
            ken_rx_state = KEN_STATE_BIT_H;
        }
        else if (ken_rx_is_micros(delta, BIT_0L))
        {
            ken_rx_queue_bit(0);
            ken_rx_state = KEN_STATE_BIT_H;
        }
        else
        {
            ken_rx_state = KEN_STATE_IDLE;
        }
        break;

    case KEN_STATE_BIT_H:
        if (ken_rx_is_micros(delta, BIT_H))
        {
            ken_rx_state = KEN_STATE_BIT_L;
        }
        else
        {
            ken_rx_state = KEN_STATE_IDLE;
        }
        break;
    }
}

void ken_rx_isr_disable()
{
    detachInterrupt(digitalPinToInterrupt(KEN_BUSY));
    detachInterrupt(digitalPinToInterrupt(KEN_DATA));
}

void ken_rx_isr_enable()
{
    attachInterrupt(digitalPinToInterrupt(KEN_BUSY), ken_rx_busy_isr, CHANGE);
    attachInterrupt(digitalPinToInterrupt(KEN_DATA), ken_rx_data_isr, CHANGE);
}

void ken_set_interface(int bits)
{
    switch (bits)
    {
    case 8:
        interface = 8;
        START_BIT_L = START_BIT_L_8 * 1000.0f;
        START_BIT_H = START_BIT_H_8 * 1000.0f;
        BIT_0L = BIT_0L_8 * 1000.0f;
        BIT_1L = BIT_1L_8 * 1000.0f;
        BIT_H = BIT_H_8 * 1000.0f;
        break;

    case 16:
        interface = 16;
        START_BIT_L = START_BIT_L_16 * 1000.0f;
        START_BIT_H = START_BIT_H_16 * 1000.0f;
        BIT_0L = BIT_0L_16 * 1000.0f;
        BIT_1L = BIT_1L_16 * 1000.0f;
        BIT_H = BIT_H_16 * 1000.0f;
        break;
    }
}

void ken_enable(bool state)
{
    pinMode(KEN_BUSY, state ? OUTPUT : INPUT);
    pinMode(KEN_DATA, state ? OUTPUT : INPUT);
}

void ken_set(int pin, bool state)
{
    digitalWrite(pin, state ? HIGH : LOW);
}

void ken_send_cmd(uint16_t cmd)
{
    ken_set(KEN_DATA, LOW);
    ken_set(KEN_BUSY, HIGH);
    ken_enable(true);

    delayMicroseconds(START_BIT_L);
    ken_set(KEN_DATA, HIGH);
    delayMicroseconds(START_BIT_H);

    for (int bit_pos = interface - 1; bit_pos >= 0; bit_pos--)
    {
        uint16_t mask = 1 << bit_pos;
        bool bit_set = cmd & mask;

        ken_set(KEN_DATA, LOW);
        delayMicroseconds(bit_set ? BIT_1L : BIT_0L);
        ken_set(KEN_DATA, HIGH);
        delayMicroseconds(BIT_H);
    }

    ken_set(KEN_BUSY, LOW);
    ken_set(KEN_DATA, LOW);
    ken_enable(false);
}

void ken_try(int start, int count)
{
    for (uint16_t cmd = start; cmd < start + count; cmd++)
    {
        ken_send_cmd(cmd);
        delayMicroseconds(20000);
    }
}

void ken_setup()
{
    ken_enable(false);

    gpio_set_drive_capability((gpio_num_t)KEN_BUSY, GPIO_DRIVE_CAP_3);
    gpio_set_drive_capability((gpio_num_t)KEN_DATA, GPIO_DRIVE_CAP_3);
    ken_set_interface(16);

    ken_rx_isr_enable();
}

bool ken_loop()
{
    if ((ken_rx_state == 0) && (ken_rx_buffer_pos != 0))
    {
        char msg[128];

        snprintf(msg, sizeof(msg), "Received %d bits: 0x%04X", ken_rx_buffer_pos, ken_rx_buffer);
        mqtt_publish_string("feeds/string/%s/error", msg);
        Serial.printf("%s\n", msg);

        ken_rx_buffer_pos = 0;
        ken_rx_buffer = 0;
    }
    return true;
}
