#ifndef CURVA_ANALISE_H
#define CURVA_ANALISE_H

#include <Arduino.h>

#include "LinhaAnalise.h"


enum DirecaoCurva
{
    CURVA_NENHUMA,

    CURVA_ESQUERDA,

    CURVA_DIREITA
};


struct CurvaData
{
    bool curva90;

    DirecaoCurva direcao;

    int sensoresEsquerda;

    int sensoresDireita;

    bool linhaCentral;

    bool linhaLarga;
};


class CurvaAnalise
{

private:

    // Intensidade mínima para considerar
    // um sensor como preto.

    const float LIMIAR_PRETO =
        0.35f;


    // Quantidade mínima de sensores
    // de um lado.

    const int MINIMO_SENSORES_LADO =
        3;


    // Diferença mínima entre os lados.

    const int DOMINANCIA_MINIMA =
        2;


    CurvaData resultado;


public:

    CurvaAnalise();


    void update(
        const LinhaData& linha
    );


    CurvaData getData();
};

#endif