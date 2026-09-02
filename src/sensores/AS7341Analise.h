#ifndef AS7341_ANALISE_H
#define AS7341_ANALISE_H

#include "hardware/AS7341.h"


// ============================================================
// RESULTADO DA ANÁLISE DE UM AS7341
// ============================================================

struct AS7341Resultado
{
    bool valido;


    // ========================================================
    // INTENSIDADE GERAL
    // ========================================================

    float intensidade;


    // ========================================================
    // COMPONENTES ESPECTRAIS AGRUPADOS
    // ========================================================

    float azul;
    float verde;
    float vermelho;


    // ========================================================
    // RELAÇÕES ENTRE COMPONENTES
    // ========================================================

    float razaoVermelhoVerde;
    float razaoAzulVerde;

    float razaoNIR;


    // ========================================================
    // CANAL DOMINANTE
    //
    // 0 = inválido
    // 1 = azul
    // 2 = verde
    // 3 = vermelho
    // ========================================================

    uint8_t canalDominante;


    // ========================================================
    // DETECÇÃO DE CORES
    // ========================================================

    bool verdeDetectado;

    bool vermelhoDetectado;

    bool cinzaDetectado;
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


    // ========================================================
    // ANALISA UM SENSOR
    // ========================================================

    AS7341Resultado analisar(
        const AS7341Data& dados
    );


    // ========================================================
    // COMPARA DIREITA / ESQUERDA
    // ========================================================

    AS7341Comparacao comparar(
        const AS7341Resultado& direita,
        const AS7341Resultado& esquerda
    );


private:

    // ========================================================
    // CÁLCULOS
    // ========================================================

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


    // ========================================================
    // DETECÇÃO DE CORES
    // ========================================================

    bool detectarVerde(
        const AS7341Data& dados
    );


    bool detectarVermelho(
        const AS7341Data& dados
    );


    bool detectarCinza(
        const AS7341Data& dados
    );
};


#endif