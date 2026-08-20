#ifndef AS7341_H
#define AS7341_H

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_AS7341.h>

#include "TCA.h"


struct AS7341Data
{
    uint16_t F1;
    uint16_t F2;
    uint16_t F3;
    uint16_t F4;

    uint16_t F5;
    uint16_t F6;
    uint16_t F7;
    uint16_t F8;

    uint16_t clear;
    uint16_t nir;

    bool valido;

    unsigned long tempo;
};


class AS7341Sensores
{

private:

    TCA& tca;


    Adafruit_AS7341 sensorDireita;
    Adafruit_AS7341 sensorEsquerda;


    static const uint8_t CANAL_DIREITA = 1;
    static const uint8_t CANAL_ESQUERDA = 2;


    // Mantém exatamente a configuração atual
    static const uint8_t ATIME = 5;
    static const uint16_t ASTEP = 5;


    static const as7341_gain_t GANHO =
        AS7341_GAIN_32X;


    AS7341Data dadosDireita;
    AS7341Data dadosEsquerda;


    bool lendoDireita;
    bool lendoEsquerda;


    bool configurarSensor(
        Adafruit_AS7341& sensor
    );


    void armazenarLeitura(
        AS7341Data& destino,
        uint16_t* readings
    );


public:

    AS7341Sensores(
        TCA& controladorTCA
    );


    bool begin();


    // Não bloqueante
    bool update();


    AS7341Data getDireita();

    AS7341Data getEsquerda();

};

#endif