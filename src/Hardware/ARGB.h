#ifndef ARGB_H
#define ARGB_H

#include <Arduino.h>
#include <FastLED.h>

class ARGB
{
private:

    static const uint8_t NUM_LEDS = 8;
    static const uint8_t LED_PIN = 25;

    CRGB leds[NUM_LEDS];

    uint8_t brilho;

    void mostrar();

public:

    ARGB();

    void begin();

    void apagar();

    // ========================================================
    // Linha
    // Cada LED representa um sensor do array.
    // Preto = azul.
    // ========================================================

    void mostrarLinha(const float sensores[8]);

    // ========================================================
    // Verde
    // Esquerda = LEDs 1 e 2
    // Direita   = LEDs 7 e 8
    // ========================================================

    void mostrarVerde(
        bool esquerda,
        bool direita
    );

    // ========================================================
    // Estados especiais
    // ========================================================

    void mostrarObstaculo();

    void mostrarVermelho();

    // ========================================================
    // Brilho
    // ========================================================

    void setBrilho(uint8_t valor);
};

#endif