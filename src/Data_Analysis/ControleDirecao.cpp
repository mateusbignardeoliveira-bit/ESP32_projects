#include "ControleDirecao.h"


ControleDirecao::ControleDirecao(
    int base,
    int maxima
)
{
    velocidadeBase = base;
    velocidadeMaxima = maxima;

    resultado.m1 = 0;
    resultado.m2 = 0;
    resultado.m3 = 0;
    resultado.m4 = 0;
}


void ControleDirecao::setVelocidadeBase(
    int velocidade
)
{
    velocidadeBase = velocidade;
}


void ControleDirecao::setVelocidadeMaxima(
    int velocidade
)
{
    velocidadeMaxima = velocidade;
}


void ControleDirecao::update(
    float correcao
)
{
    // Linha à direita:
    // correção positiva
    //
    // Para virar para a direita:
    // esquerda aumenta
    // direita diminui

    float esquerda =
        velocidadeBase + correcao;

    float direita =
        velocidadeBase - correcao;


    // Limita a velocidade esquerda

    if(esquerda > velocidadeMaxima)
    {
        esquerda = velocidadeMaxima;
    }

    if(esquerda < -velocidadeMaxima)
    {
        esquerda = -velocidadeMaxima;
    }


    // Limita a velocidade direita

    if(direita > velocidadeMaxima)
    {
        direita = velocidadeMaxima;
    }

    if(direita < -velocidadeMaxima)
    {
        direita = -velocidadeMaxima;
    }


    // M1 e M2 = esquerda
    // M3 e M4 = direita

    resultado.m1 = (int)esquerda;
    resultado.m2 = (int)esquerda;

    resultado.m3 = (int)direita;
    resultado.m4 = (int)direita;
}


void ControleDirecao::parar()
{
    resultado.m1 = 0;
    resultado.m2 = 0;
    resultado.m3 = 0;
    resultado.m4 = 0;
}


MotoresData ControleDirecao::getData()
{
    return resultado;
}