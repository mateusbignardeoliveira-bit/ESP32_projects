#ifndef AS7341_ANALISE_H
#define AS7341_ANALISE_H

#include <Arduino.h>
#include "../Hardware/AS7341.h"


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

    // Relações espectrais
    float razaoVermelhoVerde;
    float razaoAzulVerde;
    float razaoNIR;

    // Canal dominante
    // 0 = inválido
    // 1 = azul
    // 2 = verde
    // 3 = vermelho
    uint8_t canalDominante;

    // Detecção de cores
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

    AS7341Resultado analisar(
        const AS7341Data& dados
    );

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