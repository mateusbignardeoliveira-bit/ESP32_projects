#include <Arduino.h>
#include <FastLED.h>

#define LED_PIN     32
#define NUM_LEDS    8
#define BRIGHTNESS  255

CRGB leds[NUM_LEDS];

void setup()
{
    FastLED.addLeds<WS2812B, LED_PIN, GRB>(leds, NUM_LEDS);
    FastLED.setBrightness(BRIGHTNESS);

    // Começa apagado
    FastLED.clear();
    FastLED.show();
}

void loop()
{
    // Percorre todo o espectro de cores
    for (uint8_t hue = 0; hue < 255; hue++)
    {
        CRGB cor = CHSV(hue, 255, 255);

        // Todos os 8 LEDs recebem a mesma cor
        for (int i = 0; i < NUM_LEDS; i++)
        {
            leds[i] = cor;
        }

        FastLED.show();

        // Velocidade da transição
        delay(50);
    }
}