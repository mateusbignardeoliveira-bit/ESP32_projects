#include "Obstaculo.h"



Obstaculo::Obstaculo(
    int limiteMM
)
{

    distanciaLimite = limiteMM;


    resultado.distancia = 0;

    resultado.detectado = false;

}



void Obstaculo::update(
    int distanciaMM
)
{

    resultado.distancia = distanciaMM;


    if(distanciaMM > 0 &&
       distanciaMM <= distanciaLimite)
    {
        resultado.detectado = true;
    }
    else
    {
        resultado.detectado = false;
    }

}



ObstaculoData Obstaculo::getData()
{

    return resultado;

}