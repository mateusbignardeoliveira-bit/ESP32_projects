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


    erroAnterior =
        0.0f;


    integral =
        0.0f;


    derivadaFiltrada =
        0.0f;


    // --------------------------------------------------------
    // Limite da integral
    // --------------------------------------------------------

    limiteIntegral =
        2.0f;


    // --------------------------------------------------------
    // Agora permitimos uma correção muito maior.
    //
    // Com base = 200:
    //
    // +800 -> esquerda +1000 / direita -600
    //        depois limitado pelo ControleDirecao
    //
    // Na prática o ControleDirecao transforma isso em
    // uma curva extremamente agressiva.
    // --------------------------------------------------------

    limiteCorrecao =
        800.0f;


    // --------------------------------------------------------
    // Filtro da derivada
    //
    // 0.20 = bastante filtragem
    // 1.00 = sem filtragem
    // --------------------------------------------------------

    fatorFiltroDerivativo =
        0.20f;


    resultado.erro =
        0.0f;


    resultado.proporcional =
        0.0f;


    resultado.derivativo =
        0.0f;


    resultado.integral =
        0.0f;


    resultado.correcao =
        0.0f;
}


// ============================================================
// RESET
// ============================================================

void PIDLinha::reset()
{
    erroAnterior =
        0.0f;


    integral =
        0.0f;


    derivadaFiltrada =
        0.0f;


    resultado.erro =
        0.0f;


    resultado.proporcional =
        0.0f;


    resultado.derivativo =
        0.0f;


    resultado.integral =
        0.0f;


    resultado.correcao =
        0.0f;
}


// ============================================================
// UPDATE
// ============================================================

void PIDLinha::update(
    float erro,
    float deltaTime
)
{
    // --------------------------------------------------------
    // Proteção contra delta inválido
    // --------------------------------------------------------

    if(
        deltaTime <= 0.0001f
    )
    {
        deltaTime =
            0.001f;
    }


    // --------------------------------------------------------
    // Guarda erro
    // --------------------------------------------------------

    resultado.erro =
        erro;


    // ========================================================
    // P
    // ========================================================

    float proporcional =
        Kp *
        erro;


    // ========================================================
    // I
    // ========================================================

    integral +=
        erro *
        deltaTime;


    // --------------------------------------------------------
    // Anti-windup
    // --------------------------------------------------------

    if(
        integral >
        limiteIntegral
    )
    {
        integral =
            limiteIntegral;
    }


    if(
        integral <
        -limiteIntegral
    )
    {
        integral =
            -limiteIntegral;
    }


    float termoIntegral =
        Ki *
        integral;


    // ========================================================
    // D
    // ========================================================

    float derivada =
        (
            erro -
            erroAnterior
        )
        /
        deltaTime;


    // --------------------------------------------------------
    // Filtro passa-baixa da derivada
    //
    // Evita que pequenas oscilações dos sensores provoquem
    // correções violentas.
    // --------------------------------------------------------

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
    // PID TOTAL
    // ========================================================

    float correcao =
        proporcional +
        termoIntegral +
        termoDerivativo;


    // ========================================================
    // SATURAÇÃO FINAL
    // ========================================================

    if(
        correcao >
        limiteCorrecao
    )
    {
        correcao =
            limiteCorrecao;
    }


    if(
        correcao <
        -limiteCorrecao
    )
    {
        correcao =
            -limiteCorrecao;
    }


    // ========================================================
    // RESULTADOS
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
// GANHOS
// ============================================================

void PIDLinha::setGanho(
    float kp,
    float ki,
    float kd
)
{
    Kp =
        kp;


    Ki =
        ki;


    Kd =
        kd;
}


// ============================================================
// LIMITE DA CORREÇÃO
// ============================================================

void PIDLinha::setLimiteCorrecao(
    float limite
)
{
    if(
        limite < 0.0f
    )
    {
        limite =
            -limite;
    }


    limiteCorrecao =
        limite;
}


// ============================================================
// LIMITE DA INTEGRAL
// ============================================================

void PIDLinha::setLimiteIntegral(
    float limite
)
{
    if(
        limite < 0.0f
    )
    {
        limite =
            -limite;
    }


    limiteIntegral =
        limite;
}


// ============================================================
// FILTRO DA DERIVADA
// ============================================================

void PIDLinha::setFiltroDerivativo(
    float fator
)
{
    if(
        fator < 0.0f
    )
    {
        fator =
            0.0f;
    }


    if(
        fator > 1.0f
    )
    {
        fator =
            1.0f;
    }


    fatorFiltroDerivativo =
        fator;
}