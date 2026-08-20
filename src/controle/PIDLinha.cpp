#include "PIDLinha.h"



PIDLinha::PIDLinha(
    float kp,
    float ki,
    float kd
)
{

    Kp = kp;

    Ki = ki;

    Kd = kd;


    erroAnterior = 0.0f;

    integral = 0.0f;


    limiteIntegral = 100.0f;

    limiteCorrecao = 500.0f;


    resultado.erro = 0.0f;

    resultado.proporcional = 0.0f;

    resultado.derivativo = 0.0f;

    resultado.integral = 0.0f;

    resultado.correcao = 0.0f;

}




void PIDLinha::reset()
{

    erroAnterior = 0.0f;

    integral = 0.0f;


    resultado.erro = 0.0f;

    resultado.proporcional = 0.0f;

    resultado.derivativo = 0.0f;

    resultado.integral = 0.0f;

    resultado.correcao = 0.0f;

}




void PIDLinha::update(
    float erro,
    float deltaTime
)
{

    // Evita divisão por zero

    if(deltaTime <= 0.0f)
    {
        deltaTime = 0.001f;
    }



    resultado.erro = erro;



    // =====================================================
    // TERMO PROPORCIONAL
    // =====================================================

    float proporcional =
        Kp * erro;



    // =====================================================
    // TERMO INTEGRAL
    // =====================================================

    integral +=
        erro * deltaTime;



    // Limita o acúmulo da integral

    if(integral > limiteIntegral)
    {
        integral = limiteIntegral;
    }


    if(integral < -limiteIntegral)
    {
        integral = -limiteIntegral;
    }


    float termoIntegral =
        Ki * integral;



    // =====================================================
    // TERMO DERIVATIVO
    // =====================================================

    float derivada =
        (erro - erroAnterior) /
        deltaTime;


    float termoDerivativo =
        Kd * derivada;



    // =====================================================
    // CORREÇÃO FINAL
    // =====================================================

    float correcao =
        proporcional +
        termoIntegral +
        termoDerivativo;



    // Limita a correção

    if(correcao > limiteCorrecao)
    {
        correcao = limiteCorrecao;
    }


    if(correcao < -limiteCorrecao)
    {
        correcao = -limiteCorrecao;
    }



    // Guarda os valores

    resultado.proporcional =
        proporcional;


    resultado.integral =
        termoIntegral;


    resultado.derivativo =
        termoDerivativo;


    resultado.correcao =
        correcao;



    // Guarda o erro atual
    // para a próxima iteração

    erroAnterior = erro;

}




PIDData PIDLinha::getData()
{

    return resultado;

}




void PIDLinha::setGanho(
    float kp,
    float ki,
    float kd
)
{

    Kp = kp;

    Ki = ki;

    Kd = kd;

}




void PIDLinha::setLimiteCorrecao(
    float limite
)
{

    limiteCorrecao = limite;

}