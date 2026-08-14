#ifndef AS7341_H
#define AS7341_H

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_AS7341.h>
#include "TCA.h"


// ============================================================
// Dados de um AS7341
// ============================================================

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



// ============================================================
// Classe responsável pelos dois AS7341
// ============================================================

class AS7341Sensores
{

private:

    // ========================================================
    // TCA9548A
    // ========================================================

    TCA& tca;


    // ========================================================
    // Objetos dos dois sensores
    // ========================================================

    Adafruit_AS7341 sensorDireita;
    Adafruit_AS7341 sensorEsquerda;


    // ========================================================
    // Canais do multiplexador
    // ========================================================

    static const uint8_t CANAL_DIREITA = 1;
    static const uint8_t CANAL_ESQUERDA = 2;


    // ========================================================
    // Configuração do AS7341
    // ========================================================

    static const uint8_t ATIME = 29;
    static const uint16_t ASTEP = 599;


    // ========================================================
    // Dados atuais
    // ========================================================

    AS7341Data dadosDireita;
    AS7341Data dadosEsquerda;


    // ========================================================
    // Configura um sensor
    // ========================================================

    bool configurarSensor(
        Adafruit_AS7341& sensor
    );


    // ========================================================
    // Converte leitura do sensor para nossa estrutura
    // ========================================================

    void armazenarLeitura(
        AS7341Data& destino,
        uint16_t* readings
    );


public:

    // ========================================================
    // Construtor
    // ========================================================

    AS7341Sensores(
        TCA& controladorTCA
    );


    // ========================================================
    // Inicializa os dois sensores
    // ========================================================

    bool begin();


    // ========================================================
    // Faz nova leitura dos dois sensores
    // ========================================================

    bool update();


    // ========================================================
    // Retorna dados da direita
    // ========================================================

    AS7341Data getDireita();


    // ========================================================
    // Retorna dados da esquerda
    // ========================================================

    AS7341Data getEsquerda();

};

#endif