
#define LED_GPIO 4

bool led_inhibit = false;

void led_setup()
{
}

void led_set_adv(uint8_t n, uint8_t r, uint8_t g, uint8_t b, bool commit)
{
}

void led_set(uint8_t n, uint8_t r, uint8_t g, uint8_t b)
{
    return led_set_adv(n, r, g, b, true);
}

void led_set_all(uint8_t r, uint8_t g, uint8_t b)
{
}

void led_set_inhibit(bool state)
{
    led_inhibit = state;
}

bool led_loop()
{
    return false;
}
