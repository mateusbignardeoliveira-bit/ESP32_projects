#ifndef VERDE_H
#define VERDE_H

#include <Arduino.h>

#include "../sensores/AS7341Analise.h"


// ============================================================
// RESULTADO DA AVALIAÇÃO DE COR
// ============================================================

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

    // Verdadeiro quando verde foi detectado
    // nos dois lados em qualquer momento da avaliação.
    bool encontrouAmbosLados;

    bool encontrouVermelho;

    bool encontrouCinza;
};


// ============================================================
// CLASSE
// ============================================================

class Verde
{
public:

    Verde(
        AS7341Analise& analise
    );


    // Inicia uma nova avaliação.
    void iniciar(
        int quantidadePretosInicial
    );


    // Processa uma nova leitura dos dois AS7341.
    void update(
        const AS7341Data& dadosEsquerda,
        const AS7341Data& dadosDireita,
        int quantidadePretos
    );


    // Finaliza a avaliação.
    void finalizar();


    // Limpa todo o histórico.
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