#ifndef LINHA_ANALISE_H
#define LINHA_ANALISE_H

#include <Arduino.h>
#include "../hardware/ArrayLinha.h"


struct LinhaData
{
    bool linhaDetectada;

    float posicao;

    float erro;

    float intensidade;

    float largura;

    // Intensidade normalizada de cada sensor
    // 0.0 = branco
    // 1.0 = preto
    float sensores[8];
};


class LinhaAnalise
{

private:

    const int branco[8] =
    {
        213,
        180,
        177,
        177,
        181,
        180,
        184,
        213
    };


    const int preto[8] =
    {
        3394,
        2820,
        2979,
        3031,
        2992,
        2763,
        3019,
        3349
    };


    const float pesos[8] =
    {
        -7.0f,
        -5.0f,
        -3.0f,
        -1.0f,
         1.0f,
         3.0f,
         5.0f,
         7.0f
    };


    LinhaData resultado;


    float normalizarSensor(
        int valor,
        int indice
    );


public:

    LinhaAnalise();

    void update(
        const ArrayData& dados
    );

    LinhaData getData();
};

#endif