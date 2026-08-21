#ifndef CONTROLE_DIRECAO_H
#define CONTROLE_DIRECAO_H

#include <Arduino.h>


struct MotoresData
{
    int m1;
    int m2;
    int m3;
    int m4;
};


class ControleDirecao
{

private:

    int velocidadeBase;

    int velocidadeMaxima;


    MotoresData resultado;


public:

    ControleDirecao(
        int base = 200,
        int maxima = 400
    );


    void setVelocidadeBase(
        int velocidade
    );


    void setVelocidadeMaxima(
        int velocidade
    );


    void update(
        float correcao
    );


    void parar();


    MotoresData getData();

};

#endif