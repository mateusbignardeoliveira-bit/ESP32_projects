#include "ARGB.h"

// ============================================================
// CONSTRUTOR
// ============================================================

ARGB::ARGB()
{
    brilho = 60;

    for (uint8_t i = 0; i < NUM_LEDS; i++)
    {
        leds[i] = CRGB::Black;
    }
}

// ============================================================
// INICIALIZAÇÃO
// ============================================================

void ARGB::begin()
{
    FastLED.addLeds<WS2812B, LED_PIN, GRB>(
        leds,
        NUM_LEDS
    );

    FastLED.setBrightness(brilho);

    apagar();
}

// ============================================================
// MOSTRA LEDS
// ============================================================

void ARGB::mostrar()
{
    FastLED.show();
}

// ============================================================
// APAGA TODOS
// ============================================================

void ARGB::apagar()
{
    for (uint8_t i = 0; i < NUM_LEDS; i++)
    {
        leds[i] = CRGB::Black;
    }

    mostrar();
}

// ============================================================
// MOSTRA LINHA
//
// Cada sensor:
// preto -> azul
// branco -> apagado
//
// O valor recebido é a intensidade normalizada:
// 0.0 = branco
// 1.0 = preto
// ============================================================

void ARGB::mostrarLinha(const float sensores[8])
{
    for (uint8_t i = 0; i < NUM_LEDS; i++)
    {
        if (sensores[i] >= 0.35f)
        {
            leds[i] = CRGB::Blue;
        }
        else
        {
            leds[i] = CRGB::Black;
        }
    }

    mostrar();
}

// ============================================================
// VERDE
//
// Esquerda:
// LED 7 e LED 8
//
// Direita:
// LED 1 e LED 2
//
// Se ambos:
// LED 1 e 2, 7 e 8
// ============================================================

void ARGB::mostrarVerde(
    bool esquerda,
    bool direita
)
{
    apagar();

    if (esquerda)
    {
        leds[6] = CRGB::Green;
        leds[7] = CRGB::Green;
    }

    if (direita)
    {
        leds[0] = CRGB::Green;
        leds[1] = CRGB::Green;
    }

    mostrar();
}

// ============================================================
// OBSTÁCULO
// Todos amarelos
// ============================================================

void ARGB::mostrarObstaculo()
{
    for (uint8_t i = 0; i < NUM_LEDS; i++)
    {
        leds[i] = CRGB::Yellow;
    }

    mostrar();
}

// ============================================================
// VERMELHO
// Todos vermelhos
// ============================================================

void ARGB::mostrarVermelho()
{
    for (uint8_t i = 0; i < NUM_LEDS; i++)
    {
        leds[i] = CRGB::Red;
    }

    mostrar();
}

// ============================================================
// CINZA
// Todos brancos
// ============================================================

void ARGB::mostrarCinza()
{
    for (uint8_t i = 0; i < NUM_LEDS; i++)
    {
        leds[i] = CRGB::White;
    }

    mostrar();
}

// ============================================================
// BRILHO
// ============================================================

void ARGB::setBrilho(uint8_t valor)
{
    brilho = valor;

    FastLED.setBrightness(brilho);

    mostrar();
}