#ifndef AS7341_ANALISE_H
#define AS7341_ANALISE_H

#include "hardware/AS7341.h"


// ============================================================
// RESULTADO DA ANÁLISE DE UM AS7341
// ============================================================

struct AS7341Resultado
{
    bool valido;

    // Intensidade geral
    float intensidade;

    // Componentes espectrais agrupados
    float azul;
    float verde;
    float vermelho;

    // Relações entre componentes
    float razaoVermelhoVerde;
    float razaoAzulVerde;

    // NIR em relação ao Clear
    float razaoNIR;

    // Canal dominante
    uint8_t canalDominante;

};


// ============================================================
// RESULTADO COMPARATIVO
// ============================================================

struct AS7341Comparacao
{
    float diferencaIntensidade;

    float diferencaVermelho;

    float diferencaVerde;

    float diferencaAzul;

    bool direitaMaisIntensa;

    bool esquerdaMaisIntensa;
};


// ============================================================
// CLASSE
// ============================================================

class AS7341Analise
{

public:

    AS7341Analise();


    // --------------------------------------------------------
    // Analisa um sensor
    // --------------------------------------------------------

    AS7341Resultado analisar(
        const AS7341Data& dados
    );


    // --------------------------------------------------------
    // Compara direita e esquerda
    // --------------------------------------------------------

    AS7341Comparacao comparar(
        const AS7341Resultado& direita,
        const AS7341Resultado& esquerda
    );


private:

    float calcularIntensidade(
        const AS7341Data& dados
    );


    float calcularAzul(
        const AS7341Data& dados
    );


    float calcularVerde(
        const AS7341Data& dados
    );


    float calcularVermelho(
        const AS7341Data& dados
    );


    uint8_t descobrirCanalDominante(
        float azul,
        float verde,
        float vermelho
    );

};

#endif