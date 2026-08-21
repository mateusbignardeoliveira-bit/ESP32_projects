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

        AVANCO_LINHA,

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
        const AS7341Data& dadosDireita,
        int quantidadePretos
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


    // ========================================================
    // MAIOR QUANTIDADE DE PRETOS VISTA DURANTE O AVANÇO
    // ========================================================

    int maiorQuantidadePretos;


    unsigned long tempoInicio;


    int leiturasSemVerde;


    // ========================================================
    // FUNÇÕES DE MOVIMENTO
    // ========================================================

    void parar();

    void andarNormal();

    void andarDevagar();

    void andarReto();

    void frear();

    void curvaEsquerda();

    void curvaDireita();

    void meiaVolta();


    // ========================================================
    // INÍCIO DAS MANOBRAS
    // ========================================================

    void iniciarAvancoLinha();

    void iniciarCurvaEsquerda();

    void iniciarCurvaDireita();

    void iniciarMeiaVolta();


    // ========================================================
    // FINALIZA
    // ========================================================

    void finalizar();
};

#endif