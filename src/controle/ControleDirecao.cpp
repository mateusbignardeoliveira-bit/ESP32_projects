#include "ControleDirecao.h"


ControleDirecao::ControleDirecao(
    int base,
    int maxima
)
{
    velocidadeBase =
        base;


    velocidadeMaxima =
        maxima;


    resultado.m1 =
        0;


    resultado.m2 =
        0;


    resultado.m3 =
        0;


    resultado.m4 =
        0;
}


// ============================================================
// VELOCIDADE BASE
// ============================================================

void ControleDirecao::setVelocidadeBase(
    int velocidade
)
{
    velocidadeBase =
        velocidade;
}


// ============================================================
// VELOCIDADE MÁXIMA
// ============================================================

void ControleDirecao::setVelocidadeMaxima(
    int velocidade
)
{
    if(
        velocidade < 0
    )
    {
        velocidade =
            -velocidade;
    }


    velocidadeMaxima =
        velocidade;
}


// ============================================================
// UPDATE
// ============================================================

void ControleDirecao::update(
    float correcao
)
{
    // ========================================================
    // CORREÇÃO POSITIVA
    //
    // Linha à direita.
    //
    // Robô precisa virar para a direita.
    //
    // Esquerda -> acelera
    // Direita  -> desacelera / pode inverter
    // ========================================================


    float esquerda =
        velocidadeBase +
        correcao;


    float direita =
        velocidadeBase -
        correcao;


    // ========================================================
    // LIMITAÇÃO
    //
    // IMPORTANTE:
    //
    // Pode chegar a -velocidadeMaxima.
    //
    // Portanto o robô pode realmente inverter uma roda.
    // ========================================================

    if(
        esquerda >
        velocidadeMaxima
    )
    {
        esquerda =
            velocidadeMaxima;
    }


    if(
        esquerda <
        -velocidadeMaxima
    )
    {
        esquerda =
            -velocidadeMaxima;
    }


    if(
        direita >
        velocidadeMaxima
    )
    {
        direita =
            velocidadeMaxima;
    }


    if(
        direita <
        -velocidadeMaxima
    )
    {
        direita =
            -velocidadeMaxima;
    }


    // ========================================================
    // MOTORES
    //
    // M1 + M2 = esquerda
    // M3 + M4 = direita
    // ========================================================

    resultado.m1 =
        (int)esquerda;


    resultado.m2 =
        (int)esquerda;


    resultado.m3 =
        (int)direita;


    resultado.m4 =
        (int)direita;
}


// ============================================================
// PARAR
// ============================================================

void ControleDirecao::parar()
{
    resultado.m1 =
        0;


    resultado.m2 =
        0;


    resultado.m3 =
        0;


    resultado.m4 =
        0;
}


// ============================================================
// GET
// ============================================================

MotoresData ControleDirecao::getData()
{
    return resultado;
}