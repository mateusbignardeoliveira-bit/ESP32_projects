#include "ControleLinha.h"


// ============================================================
// CONSTRUTOR
// ============================================================

ControleLinha::ControleLinha(
    PIDLinha& controladorPID,
    MotorControlador& controladorMotores
)
:
pid(controladorPID),
motores(controladorMotores)
{

    ativo = false;

    velocidadeBase = 200;

    velocidadeMaxima = 1000;

    ultimoTempo = millis();

}



// ============================================================
// BEGIN
// ============================================================

void ControleLinha::begin()
{

    ativo = false;

    velocidadeBase = 200;

    velocidadeMaxima = 1000;

    ultimoTempo = millis();

    pid.reset();

}



// ============================================================
// START
// ============================================================

void ControleLinha::start()
{

    ativo = true;

    ultimoTempo = millis();

    pid.reset();

}



// ============================================================
// UPDATE
// ============================================================

void ControleLinha::update(
    const LinhaData& linha
)
{

    if(!ativo)
    {
        return;
    }


    // ========================================================
    // TEMPO
    // ========================================================

    unsigned long agora = millis();


    float deltaTime =
        (agora - ultimoTempo)
        /
        1000.0f;


    ultimoTempo = agora;


    if(deltaTime <= 0.0001f)
    {
        deltaTime = 0.001f;
    }


    if(deltaTime > 0.1f)
    {
        deltaTime = 0.1f;
    }


    // ========================================================
    // ATUALIZA PID
    // ========================================================

    pid.update(
        linha.erro,
        deltaTime
    );


    PIDData dadosPID =
        pid.getData();


    float correcao =
        dadosPID.correcao;


    // ========================================================
    // VELOCIDADES
    // ========================================================
    //
    // erro positivo:
    // linha para a direita
    //
    // correção positiva:
    // aumenta esquerda
    // diminui direita
    //
    // erro negativo:
    // linha para a esquerda
    //
    // correção negativa:
    // diminui esquerda
    // aumenta direita
    //
    // ========================================================

    int velocidadeEsquerda =
        velocidadeBase
        +
        (int)correcao;


    int velocidadeDireita =
        velocidadeBase
        -
        (int)correcao;


    // ========================================================
    // LIMITAÇÃO
    // ========================================================

    velocidadeEsquerda =
        limitarVelocidade(
            velocidadeEsquerda
        );


    velocidadeDireita =
        limitarVelocidade(
            velocidadeDireita
        );


    // ========================================================
    // ENVIA AOS QUATRO MOTORES
    // ========================================================

    motores.setSpeed(
        velocidadeEsquerda,
        velocidadeEsquerda,
        velocidadeDireita,
        velocidadeDireita
    );

}



// ============================================================
// STOP
// ============================================================

void ControleLinha::stop()
{

    ativo = false;

    pid.reset();

    motores.stop();

}



// ============================================================
// RESET
// ============================================================

void ControleLinha::reset()
{

    pid.reset();

    ultimoTempo = millis();

}



// ============================================================
// ESTADO
// ============================================================

bool ControleLinha::ativoAgora() const
{

    return ativo;

}



// ============================================================
// VELOCIDADE BASE
// ============================================================

void ControleLinha::setVelocidadeBase(
    int velocidade
)
{

    if(velocidade < 0)
    {
        velocidade = -velocidade;
    }


    if(velocidade > velocidadeMaxima)
    {
        velocidade = velocidadeMaxima;
    }


    velocidadeBase =
        velocidade;

}



// ============================================================
// GET VELOCIDADE BASE
// ============================================================

int ControleLinha::getVelocidadeBase() const
{

    return velocidadeBase;

}



// ============================================================
// VELOCIDADE MÁXIMA
// ============================================================

void ControleLinha::setVelocidadeMaxima(
    int velocidade
)
{

    if(velocidade < 1)
    {
        velocidade = 1;
    }


    velocidadeMaxima =
        velocidade;


    if(velocidadeBase > velocidadeMaxima)
    {
        velocidadeBase =
            velocidadeMaxima;
    }

}



// ============================================================
// LIMITADOR
// ============================================================

int ControleLinha::limitarVelocidade(
    int velocidade
)
{

    if(velocidade > velocidadeMaxima)
    {
        return velocidadeMaxima;
    }


    if(velocidade < -velocidadeMaxima)
    {
        return -velocidadeMaxima;
    }


    return velocidade;

}