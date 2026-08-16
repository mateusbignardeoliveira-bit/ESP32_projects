#include "TOFAnalise.h"


// ============================================================
// CONSTRUTOR
// ============================================================

TOFAnalise::TOFAnalise(TOF200F& sensor)
:
tof(sensor)
{

    distancia = 0;

    valido = false;

    obstaculo = false;

}


// ============================================================
// UPDATE
// ============================================================

void TOFAnalise::update()
{

    distancia = tof.getDistance();


    // --------------------------------------------------------
    // Verifica se a leitura é válida
    // --------------------------------------------------------

    if(distancia <= 0)
    {

        valido = false;

        obstaculo = false;

        return;

    }


    valido = true;


    // --------------------------------------------------------
    // Verifica obstáculo
    // --------------------------------------------------------

    if(distancia <= DISTANCIA_OBSTACULO_MM)
    {

        obstaculo = true;

    }
    else
    {

        obstaculo = false;

    }

}


// ============================================================
// DISTÂNCIA
// ============================================================

int TOFAnalise::getDistancia()
{

    return distancia;

}


// ============================================================
// LEITURA VÁLIDA
// ============================================================

bool TOFAnalise::isValido() const
{

    return valido;

}


// ============================================================
// OBSTÁCULO
// ============================================================

bool TOFAnalise::temObstaculo() const
{

    return obstaculo;

}


// ============================================================
// LIMITE
// ============================================================

int TOFAnalise::getLimiteObstaculo()
{

    return DISTANCIA_OBSTACULO_MM;

}