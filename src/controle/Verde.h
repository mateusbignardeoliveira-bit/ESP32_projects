#ifndef VERDE_H
#define VERDE_H

#include <Arduino.h>

#include "sensores/AS7341Analise.h"


struct AvaliacaoCorData
{
    bool avaliando;

    bool finalizada;

    bool verdeEsquerda;

    bool verdeDireita;

    bool vermelhoEsquerda;

    bool vermelhoDireita;

    bool cinzaEsquerda;

    bool cinzaDireita;

    int maiorQuantidadePretos;

    bool encontrouAmbosLados;

    bool encontrouVermelho;

    bool encontrouCinza;
};


class Verde
{
public:

    Verde(
        AS7341Analise& analise
    );


    void iniciar(
        int quantidadePretosInicial
    );


    void update(
        const AS7341Data& dadosEsquerda,
        const AS7341Data& dadosDireita,
        int quantidadePretos
    );


    void finalizar();


    void reset();


    bool estaAvaliando() const;


    bool finalizado() const;


    bool detectouVerdeEsquerda() const;


    bool detectouVerdeDireita() const;


    bool detectouVerdeDosDoisLados() const;


    bool detectouVermelho() const;


    bool detectouCinza() const;


    int getMaiorQuantidadePretos() const;


    AvaliacaoCorData getData() const;


private:

    AS7341Analise& analise;


    AvaliacaoCorData resultado;
};

#endif