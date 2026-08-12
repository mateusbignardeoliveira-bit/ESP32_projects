#ifndef OBSTACULO_H
#define OBSTACULO_H

#include <Arduino.h>
#include "../hardware/TOF200F.h"


struct ObstaculoData
{
    int distancia;

    bool detectado;
};


class Obstaculo
{

private:

    // Distância abaixo da qual consideramos
    // que existe um obstáculo.
    int distanciaLimite;


    ObstaculoData resultado;


public:

    Obstaculo(
        int limiteMM = 150
    );


    void update(
        int distanciaMM
    );


    ObstaculoData getData();

};


#endif