#ifndef AS7341_H
#define AS7341_H

#include <Arduino.h>

#include "TCA.h"

#include "DEV_Config.h"
#include "Waveshare_AS7341.h"


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
};



class AS7341
{

private:

    TCA& tca;

    uint8_t canal;

    AS7341Data dados;


public:


    // Recebe o TCA que controla o sensor
    AS7341(TCA& multiplexador);



    // Inicializa o sensor em determinado canal
    void begin(uint8_t canalTCA);



    // Faz uma nova leitura
    void update();



    // Retorna os últimos dados medidos
    AS7341Data getData();

};



#endif