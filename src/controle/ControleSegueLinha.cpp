#include "ControleSegueLinha.h"


// ============================================================
// CONSTRUTOR
// ============================================================

ControleSegueLinha::ControleSegueLinha()
{

    // ========================================================
    // PID
    // ========================================================

    Kp = 0.0f;

    Ki = 0.0f;

    Kd = 0.0f;


    // ========================================================
    // MEMÓRIA
    // ========================================================

    erroAnterior = 0.0f;

    integral = 0.0f;

    tempoAnterior = millis();


    // ========================================================
    // VELOCIDADES
    // ========================================================

    velocidadeBase = 0;

    velocidadeMaxima = 0;


    // ========================================================
    // VELOCIDADE ADAPTATIVA
    // ========================================================

    erroVelocidadeMinimo = 1.0f;

    erroVelocidadeMaximo = 6.0f;

    velocidadeMinimaCurva = 500;


    // ========================================================
    // RESULTADO
    // ========================================================

    resultado.proporcional = 0.0f;

    resultado.integral = 0.0f;

    resultado.derivativo = 0.0f;

    resultado.correcao = 0.0f;

    resultado.velocidadeBaseAtual = 0;

    resultado.velocidadeEsquerda = 0;

    resultado.velocidadeDireita = 0;

}


// ============================================================
// LIMITAR VALOR
// ============================================================

float ControleSegueLinha::limitar(
    float valor,
    float minimo,
    float maximo
)
{

    if(valor < minimo)
    {
        return minimo;
    }


    if(valor > maximo)
    {
        return maximo;
    }


    return valor;

}


// ============================================================
// CALCULAR VELOCIDADE BASE
// ============================================================

int ControleSegueLinha::calcularVelocidadeBase(
    float erro
)
{

    float magnitude =
        fabsf(erro);


    // ========================================================
    // ROBÔ PRATICAMENTE CENTRALIZADO
    // ========================================================

    if(
        magnitude <=
        erroVelocidadeMinimo
    )
    {

        return velocidadeBase;

    }


    // ========================================================
    // ERRO MUITO GRANDE
    // ========================================================

    if(
        magnitude >=
        erroVelocidadeMaximo
    )
    {

        return velocidadeMinimaCurva;

    }


    // ========================================================
    // INTERPOLAÇÃO
    // ========================================================

    float proporcao =
        (
            magnitude -
            erroVelocidadeMinimo
        )
        /
        (
            erroVelocidadeMaximo -
            erroVelocidadeMinimo
        );


    float velocidade =
        velocidadeBase
        -
        (
            proporcao *
            (
                velocidadeBase -
                velocidadeMinimaCurva
            )
        );


    return (int)velocidade;

}


// ============================================================
// CONFIGURAR PID
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
// CONFIGURAR VELOCIDADE
// ============================================================

void ControleSegueLinha::configurarVelocidade(
    int base,
    int maxima
)
{

    velocidadeBase = base;

    velocidadeMaxima = maxima;

}


// ============================================================
// CONFIGURAR VELOCIDADE ADAPTATIVA
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

    resultado.velocidadeBaseAtual =
        velocidadeBase;

    resultado.velocidadeEsquerda = 0;

    resultado.velocidadeDireita = 0;

}


// ============================================================
// UPDATE
// ============================================================

void ControleSegueLinha::update(
    const LinhaData& linha
)
{

    // ========================================================
    // TEMPO
    // ========================================================

    unsigned long agora =
        millis();


    float dt =
        (
            float
            (
                agora -
                tempoAnterior
            )
        )
        /
        1000.0f;


    tempoAnterior =
        agora;


    // ========================================================
    // PROTEÇÃO CONTRA DT MUITO PEQUENO
    // ========================================================

    if(dt < 0.001f)
    {
        dt = 0.001f;
    }


    // ========================================================
    // PROTEÇÃO CONTRA PAUSA MUITO GRANDE
    // ========================================================

    if(dt > 0.100f)
    {
        dt = 0.100f;
    }


    // ========================================================
    // ERRO
    // ========================================================

    float erro =
        linha.erro;


    // ========================================================
    // TERMO PROPORCIONAL
    // ========================================================

    float proporcional =
        erro;


    // ========================================================
    // TERMO INTEGRAL
    // ========================================================

    if(Ki != 0.0f)
    {

        integral +=
            erro * dt;


        // ----------------------------------------------------
        // ANTI-WINDUP
        // ----------------------------------------------------

        integral =
            limitar(
                integral,
                -50.0f,
                50.0f
            );

    }
    else
    {

        integral = 0.0f;

    }


    // ========================================================
    // TERMO DERIVATIVO
    // ========================================================

    float derivativo =
        (
            erro -
            erroAnterior
        )
        /
        dt;


    erroAnterior =
        erro;


    // --------------------------------------------------------
    // LIMITAÇÃO DO DERIVATIVO
    // --------------------------------------------------------

    derivativo =
        limitar(
            derivativo,
            -20.0f,
            20.0f
        );


    // ========================================================
    // PID
    // ========================================================

    float correcao =
        (
            Kp *
            proporcional
        )
        +
        (
            Ki *
            integral
        )
        +
        (
            Kd *
            derivativo
        );


    // ========================================================
    // VELOCIDADE BASE ADAPTATIVA
    // ========================================================

    int baseAtual =
        calcularVelocidadeBase(
            erro
        );


    // ========================================================
    // LIMITE DA CORREÇÃO
    // ========================================================

    float limiteCorrecao =
        baseAtual * 0.90f;


    if(
        limiteCorrecao >
        velocidadeMaxima
    )
    {

        limiteCorrecao =
            velocidadeMaxima;

    }


    if(limiteCorrecao < 0.0f)
    {
        limiteCorrecao = 0.0f;
    }


    // ========================================================
    // LIMITA CORREÇÃO
    // ========================================================

    correcao =
        limitar(
            correcao,
            -limiteCorrecao,
            limiteCorrecao
        );


    // ========================================================
    // CALCULA MOTORES
    // ========================================================

    int esquerda =
        baseAtual +
        (int)correcao;


    int direita =
        baseAtual -
        (int)correcao;


    // ========================================================
    // LIMITES FINAIS
    // ========================================================

    esquerda =
        constrain(
            esquerda,
            -velocidadeMaxima,
            velocidadeMaxima
        );


    direita =
        constrain(
            direita,
            -velocidadeMaxima,
            velocidadeMaxima
        );


    // ========================================================
    // ARMAZENA RESULTADO
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