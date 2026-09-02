#include "PIDLinha.h"


// ============================================================
// CONSTRUTOR
// ============================================================

PIDLinha::PIDLinha(
    float kp,
    float ki,
    float kd
)
{

    // --------------------------------------------------------
    // Ganhos
    // --------------------------------------------------------

    Kp = kp;
    Ki = ki;
    Kd = kd;


    // --------------------------------------------------------
    // Memória
    // --------------------------------------------------------

    erroAnterior = 0.0f;

    integral = 0.0f;

    derivadaFiltrada = 0.0f;


    // --------------------------------------------------------
    // Limite da integral
    // --------------------------------------------------------

    limiteIntegral = 2.0f;


    // --------------------------------------------------------
    // Limite da correção
    //
    // A saída poderá posteriormente ser somada/subtraída
    // da velocidade base dos motores.
    // --------------------------------------------------------

    limiteCorrecao = 800.0f;


    // --------------------------------------------------------
    // Filtro da derivada
    //
    // 0.20 = forte filtragem
    // 1.00 = sem filtragem
    // --------------------------------------------------------

    fatorFiltroDerivativo = 0.20f;


    // --------------------------------------------------------
    // Resultado inicial
    // --------------------------------------------------------

    resultado.erro = 0.0f;

    resultado.proporcional = 0.0f;

    resultado.derivativo = 0.0f;

    resultado.integral = 0.0f;

    resultado.correcao = 0.0f;

}



// ============================================================
// RESET
// ============================================================

void PIDLinha::reset()
{

    erroAnterior = 0.0f;

    integral = 0.0f;

    derivadaFiltrada = 0.0f;


    resultado.erro = 0.0f;

    resultado.proporcional = 0.0f;

    resultado.derivativo = 0.0f;

    resultado.integral = 0.0f;

    resultado.correcao = 0.0f;

}



// ============================================================
// UPDATE
// ============================================================

void PIDLinha::update(
    float erro,
    float deltaTime
)
{

    // ========================================================
    // PROTEÇÃO DO TEMPO
    // ========================================================

    if(deltaTime <= 0.0001f)
    {
        deltaTime = 0.001f;
    }


    // Evita uma derivada exagerada caso o loop fique
    // temporariamente bloqueado.

    if(deltaTime > 0.1f)
    {
        deltaTime = 0.1f;
    }


    // ========================================================
    // GUARDA ERRO
    // ========================================================

    resultado.erro = erro;


    // ========================================================
    // TERMO PROPORCIONAL
    // ========================================================

    float proporcional =
        Kp * erro;


    // ========================================================
    // TERMO INTEGRAL
    // ========================================================

    integral +=
        erro * deltaTime;


    // --------------------------------------------------------
    // Anti-windup
    // --------------------------------------------------------

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


    // ========================================================
    // TERMO DERIVATIVO
    // ========================================================

    float derivada =
        (
            erro -
            erroAnterior
        )
        /
        deltaTime;


    // ========================================================
    // FILTRO DA DERIVADA
    // ========================================================

    derivadaFiltrada =
        (
            fatorFiltroDerivativo *
            derivada
        )
        +
        (
            (1.0f - fatorFiltroDerivativo) *
            derivadaFiltrada
        );


    float termoDerivativo =
        Kd *
        derivadaFiltrada;


    // ========================================================
    // SOMA DOS TERMOS
    // ========================================================

    float correcao =
        proporcional
        +
        termoIntegral
        +
        termoDerivativo;


    // ========================================================
    // LIMITA CORREÇÃO
    // ========================================================

    if(correcao > limiteCorrecao)
    {
        correcao = limiteCorrecao;
    }


    if(correcao < -limiteCorrecao)
    {
        correcao = -limiteCorrecao;
    }


    // ========================================================
    // SALVA RESULTADOS
    // ========================================================

    resultado.proporcional =
        proporcional;


    resultado.integral =
        termoIntegral;


    resultado.derivativo =
        termoDerivativo;


    resultado.correcao =
        correcao;


    // ========================================================
    // MEMÓRIA
    // ========================================================

    erroAnterior =
        erro;

}



// ============================================================
// GET DATA
// ============================================================

PIDData PIDLinha::getData()
{

    return resultado;

}



// ============================================================
// CONFIGURA GANHOS
// ============================================================

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



// ============================================================
// CONFIGURA LIMITE DA CORREÇÃO
// ============================================================

void PIDLinha::setLimiteCorrecao(
    float limite
)
{

    if(limite < 0.0f)
    {
        limite = -limite;
    }


    limiteCorrecao =
        limite;

}



// ============================================================
// CONFIGURA LIMITE DA INTEGRAL
// ============================================================

void PIDLinha::setLimiteIntegral(
    float limite
)
{

    if(limite < 0.0f)
    {
        limite = -limite;
    }


    limiteIntegral =
        limite;

}



// ============================================================
// CONFIGURA FILTRO DA DERIVADA
// ============================================================

void PIDLinha::setFiltroDerivativo(
    float fator
)
{

    if(fator < 0.0f)
    {
        fator = 0.0f;
    }


    if(fator > 1.0f)
    {
        fator = 1.0f;
    }


    fatorFiltroDerivativo =
        fator;

}