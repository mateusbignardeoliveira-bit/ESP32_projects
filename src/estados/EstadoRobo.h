#ifndef ESTADO_ROBO_H
#define ESTADO_ROBO_H

#include <Arduino.h>


enum EstadoRobo
{
    ESTADO_SEGUINDO_LINHA,

    ESTADO_CURVA_ESQUERDA,

    ESTADO_CURVA_DIREITA,

    ESTADO_CRUZAMENTO
};


class EstadoRoboControl
{

private:

    EstadoRobo estadoAtual;


public:

    EstadoRoboControl();


    void definirEstado(
        EstadoRobo novoEstado
    );


    EstadoRobo getEstado();

};

#endif