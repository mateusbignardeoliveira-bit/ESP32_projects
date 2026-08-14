#ifndef TOF_ANALISE_H
#define TOF_ANALISE_H

#include <Arduino.h>
#include "../hardware/TOF200F.h"


class TOFAnalise
{

private:

    TOF200F& tof;

    int distancia;

    bool valido;

    bool obstaculo;


    // Limite de obstáculo
    static constexpr int DISTANCIA_OBSTACULO_MM = 100;


public:

    TOFAnalise(TOF200F& sensor);


    // Atualiza a interpretação do sensor
    void update();


    // Retorna a distância atual em mm
    int getDistancia();


    // Indica se a leitura é válida
    bool isValido();


    // Indica se existe obstáculo
    bool temObstaculo();


    // Retorna o limite configurado
    int getLimiteObstaculo();

};

#endif