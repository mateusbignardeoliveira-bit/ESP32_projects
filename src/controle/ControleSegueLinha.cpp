#include "ControleSegueLinha.h"


// ============================================================
// CONSTRUTOR
// ============================================================

ControleSegueLinha::ControleSegueLinha()
{

    Kp = 0.0f;
    Ki = 0.0f;
    Kd = 0.0f;


    erroAnterior = 0.0f;

    integral = 0.0f;


    tempoAnterior = millis();


    velocidadeBase = 0;

    velocidadeMaxima = 0;


    // --------------------------------------------------------
    // Configuração padrão da velocidade adaptativa
    // --------------------------------------------------------

    erroVelocidadeMinimo = 1.0f;

    erroVelocidadeMaximo = 6.0f;

    velocidadeMinimaCurva = 500;


    // --------------------------------------------------------
    // Resultado
    // --------------------------------------------------------

    resultado.proporcional = 0.0f;

    resultado.integral = 0.0f;

    resultado.derivativo = 0.0f;

    resultado.correcao = 0.0f;

    resultado.velocidadeBaseAtual = 0;

    resultado.velocidadeEsquerda = 0;

    resultado.velocidadeDireita = 0;

}


// ============================================================
// CONFIGURA PID
// ============================================================

void ControleSegueLinha::configurarPID(
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
// CONFIGURA VELOCIDADE
// ============================================================

void ControleSegueLinha::configurarVelocidade(
    int base,
    int maxima
)
{

    velocidadeBase =
        base;

    velocidadeMaxima =
        maxima;

}


// ============================================================
// CONFIGURA VELOCIDADE ADAPTATIVA
// ============================================================

void ControleSegueLinha::configurarVelocidadeAdaptativa(
    float erroMinimo,
    float erroMaximo,
    int velocidadeMinima
)
{

    erroVelocidadeMinimo =
        erroMinimo;


    erroVelocidadeMaximo =
        erroMaximo;


    velocidadeMinimaCurva =
        velocidadeMinima;

}


// ============================================================
// CALCULA VELOCIDADE BASE
// ============================================================

int ControleSegueLinha::calcularVelocidadeBase(
    float erro
)
{

    float magnitude =
        fabs(erro);


    // --------------------------------------------------------
    // Erro pequeno
    //
    // Mantém velocidade máxima
    // --------------------------------------------------------

    if(
        magnitude <=
        erroVelocidadeMinimo
    )
    {

        return velocidadeBase;

    }


    // --------------------------------------------------------
    // Erro muito grande
    //
    // Usa velocidade mínima
    // --------------------------------------------------------

    if(
        magnitude >=
        erroVelocidadeMaximo
    )
    {

        return velocidadeMinimaCurva;

    }


    // --------------------------------------------------------
    // Entre os dois limites
    //
    // Interpolação linear
    // --------------------------------------------------------

    float proporcao =
        (magnitude - erroVelocidadeMinimo)
        /
        (erroVelocidadeMaximo - erroVelocidadeMinimo);


    int velocidade =
        velocidadeBase
        -
        (
            int
            (
                proporcao *
                (
                    velocidadeBase
                    -
                    velocidadeMinimaCurva
                )
            )
        );


    return velocidade;

}


// ============================================================
// RESET
// ============================================================

void ControleSegueLinha::reset()
{

    erroAnterior = 0.0f;

    integral = 0.0f;

    tempoAnterior = millis();


    resultado.proporcional = 0.0f;

    resultado.integral = 0.0f;

    resultado.derivativo = 0.0f;

    resultado.correcao = 0.0f;

}


// ============================================================
// UPDATE
// ============================================================

void ControleSegueLinha::update(
    const LinhaData& linha
)
{

    unsigned long agora =
        millis();


    float dt =
        (agora - tempoAnterior)
        / 1000.0f;


    tempoAnterior =
        agora;


    // --------------------------------------------------------
    // Proteção
    // --------------------------------------------------------

    if(dt <= 0.0f)
    {
        dt = 0.001f;
    }


    // ========================================================
    // ERRO
    // ========================================================

    float erro =
        linha.erro;


    // ========================================================
    // PROPORCIONAL
    // ========================================================

    float proporcional =
        erro;


    // ========================================================
    // INTEGRAL
    // ========================================================

    if(Ki != 0.0f)
    {

        integral +=
            erro * dt;


        if(integral > 100.0f)
        {
            integral = 100.0f;
        }


        if(integral < -100.0f)
        {
            integral = -100.0f;
        }

    }
    else
    {

        integral = 0.0f;

    }


    // ========================================================
    // DERIVATIVO
    // ========================================================

    float derivativo =
        (erro - erroAnterior)
        / dt;


    erroAnterior =
        erro;


    // --------------------------------------------------------
    // Limita derivativo
    // --------------------------------------------------------

    const float LIMITE_DERIVATIVO = 20.0f;


    if(derivativo > LIMITE_DERIVATIVO)
    {
        derivativo =
            LIMITE_DERIVATIVO;
    }


    if(derivativo < -LIMITE_DERIVATIVO)
    {
        derivativo =
            -LIMITE_DERIVATIVO;
    }


    // ========================================================
    // PID
    // ========================================================

    float correcao =
        (Kp * proporcional)
        +
        (Ki * integral)
        +
        (Kd * derivativo);


    // ========================================================
    // VELOCIDADE ADAPTATIVA
    // ========================================================

    int baseAtual =
        calcularVelocidadeBase(
            erro
        );

// LIMITE DA CORREÇÃO
//
// No segue-linha normal não permitimos que um lado
// fique negativo.
//
// Giro no próprio eixo será responsabilidade dos
// estados especiais de curva.
// ========================================================

float limiteCorrecao =
    (float)baseAtual * 0.90f;


// Limite absoluto

if(limiteCorrecao > velocidadeMaxima)
{
    limiteCorrecao =
        velocidadeMaxima;
}


if(correcao > limiteCorrecao)
{
    correcao =
        limiteCorrecao;
}


if(correcao < -limiteCorrecao)
{
    correcao =
        -limiteCorrecao;
}


    // ========================================================
    // MOTORES
    // ========================================================

    int esquerda =
        baseAtual
        +
        (int)correcao;


    int direita =
        baseAtual
        -
        (int)correcao;


    // ========================================================
    // LIMITES
    // ========================================================

    if(esquerda > velocidadeMaxima)
    {
        esquerda =
            velocidadeMaxima;
    }


    if(esquerda < -velocidadeMaxima)
    {
        esquerda =
            -velocidadeMaxima;
    }


    if(direita > velocidadeMaxima)
    {
        direita =
            velocidadeMaxima;
    }


    if(direita < -velocidadeMaxima)
    {
        direita =
            -velocidadeMaxima;
    }


    // ========================================================
    // RESULTADO
    // ========================================================

    resultado.proporcional =
        proporcional;

    resultado.integral =
        integral;

    resultado.derivativo =
        derivativo;

    resultado.correcao =
        correcao;

    resultado.velocidadeBaseAtual =
        baseAtual;

    resultado.velocidadeEsquerda =
        esquerda;

    resultado.velocidadeDireita =
        direita;

}


// ============================================================
// GET DATA
// ============================================================

ControleData ControleSegueLinha::getData()
{

    return resultado;

}