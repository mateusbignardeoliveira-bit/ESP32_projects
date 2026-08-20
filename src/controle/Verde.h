#ifndef VERDE_H
#define VERDE_H

#include <Arduino.h>

#include "hardware/MotorControlador.h"
#include "sensores/AS7341Analise.h"


class Verde
{
public:

    enum Estado
    {
        NORMAL,

        FREANDO,

        AVANCANDO,

        CURVA_ESQUERDA,

        CURVA_DIREITA,

        MEIA_VOLTA,

        FINALIZADO
    };


    Verde(
        MotorControlador& motores,
        AS7341Analise& analise
    );


    void update(
        const AS7341Data& dadosEsquerda,
        const AS7341Data& dadosDireita
    );


    void reset();


    bool estaExecutando() const;


    bool finalizado() const;


    Estado getEstado() const;


    bool detectouVerdeEsquerda() const;


    bool detectouVerdeDireita() const;


private:

    MotorControlador& motores;

    AS7341Analise& analise;


    Estado estado;


    bool verdeEsquerda;

    bool verdeDireita;


    unsigned long tempoInicio;


    int leiturasSemVerde;


    void parar();

    void andarNormal();

    void andarDevagar();

    void frear();

    void curvaEsquerda();

    void curvaDireita();

    void meiaVolta();

    void iniciarCurvaEsquerda();

    void iniciarCurvaDireita();

    void iniciarMeiaVolta();

    void finalizar();
};

#endif