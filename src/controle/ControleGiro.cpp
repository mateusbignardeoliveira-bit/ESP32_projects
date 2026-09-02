#include "ControleGiro.h"


// ============================================================
// CONSTRUTOR
// ============================================================

ControleGiro::ControleGiro(
    IMU& imu,
    MotorControlador& motorControlador
)
:
sensorIMU(imu),
motores(motorControlador)
{

    // --------------------------------------------------------
    // Estado
    // --------------------------------------------------------

    emExecucao = false;
    movimentoConcluido = false;


    // --------------------------------------------------------
    // Giro
    // --------------------------------------------------------

    headingInicial = 0.0f;
    headingAlvo = 0.0f;
    anguloSolicitado = 0.0f;


    // --------------------------------------------------------
    // PID
    // --------------------------------------------------------

    kp = 1.8f;
    ki = 0.0f;
    kd = 0.10f;

    erroAnterior = 0.0f;
    integral = 0.0f;

    ultimoTempo = 0;


    // --------------------------------------------------------
    // Velocidades
    // --------------------------------------------------------

    velocidadeMaxima = 100;
    velocidadeMinima = 30;


    // --------------------------------------------------------
    // Finalização
    // --------------------------------------------------------

    tolerancia = 2.0f;

    tempoEstabilizacao = 60;

    inicioEstabilizacao = 0;

}



// ============================================================
// BEGIN
// ============================================================

void ControleGiro::begin()
{

    emExecucao = false;
    movimentoConcluido = false;

    erroAnterior = 0.0f;
    integral = 0.0f;

}



// ============================================================
// CALCULAR ERRO ANGULAR
// ============================================================

float ControleGiro::calcularErroAngular(
    float atual,
    float alvo
)
{

    float erro = alvo - atual;


    // --------------------------------------------------------
    // Normaliza para -180 ... +180
    // --------------------------------------------------------

    while(erro > 180.0f)
    {
        erro -= 360.0f;
    }


    while(erro < -180.0f)
    {
        erro += 360.0f;
    }


    return erro;

}



// ============================================================
// GIRAR
// ============================================================

void ControleGiro::girar(float angulo)
{

    // --------------------------------------------------------
    // Cancela qualquer movimento anterior
    // --------------------------------------------------------

    if(emExecucao)
    {
        cancelar();
    }


    // --------------------------------------------------------
    // Atualiza IMU antes de capturar o heading
    // --------------------------------------------------------

    sensorIMU.update();


    headingInicial = sensorIMU.getHeading();


    // --------------------------------------------------------
    // Guarda ângulo solicitado
    // --------------------------------------------------------

    anguloSolicitado = angulo;


    // --------------------------------------------------------
    // Calcula alvo
    // --------------------------------------------------------

    headingAlvo = headingInicial + angulo;


    while(headingAlvo >= 360.0f)
    {
        headingAlvo -= 360.0f;
    }


    while(headingAlvo < 0.0f)
    {
        headingAlvo += 360.0f;
    }


    // --------------------------------------------------------
    // Reseta PID
    // --------------------------------------------------------

    erroAnterior = 0.0f;
    integral = 0.0f;

    ultimoTempo = millis();


    // --------------------------------------------------------
    // Estado
    // --------------------------------------------------------

    emExecucao = true;
    movimentoConcluido = false;

    inicioEstabilizacao = 0;


}



// ============================================================
// CURVA 90 DIREITA
// ============================================================

void ControleGiro::curva90Direita()
{

    girar(90.0f);

}



// ============================================================
// CURVA 90 ESQUERDA
// ============================================================

void ControleGiro::curva90Esquerda()
{

    girar(-90.0f);

}



// ============================================================
// CURVA 180
// ============================================================

void ControleGiro::curva180()
{

    girar(180.0f);

}



// ============================================================
// UPDATE
// ============================================================

void ControleGiro::update()
{

    if(!emExecucao)
    {
        return;
    }


    // --------------------------------------------------------
    // Atualiza IMU
    // --------------------------------------------------------

    sensorIMU.update();


    float headingAtual = sensorIMU.getHeading();


    // --------------------------------------------------------
    // Calcula erro
    // --------------------------------------------------------

    float erro = calcularErroAngular(
        headingAtual,
        headingAlvo
    );


    // --------------------------------------------------------
    // Tempo
    // --------------------------------------------------------

    unsigned long agora = millis();

    float dt =
        (agora - ultimoTempo) / 1000.0f;


    ultimoTempo = agora;


    if(dt <= 0.0f)
    {
        dt = 0.001f;
    }


    if(dt > 0.1f)
    {
        dt = 0.1f;
    }


    // --------------------------------------------------------
    // Integral
    // --------------------------------------------------------

    integral += erro * dt;


    // --------------------------------------------------------
    // Limita integral
    // --------------------------------------------------------

    const float LIMITE_INTEGRAL = 100.0f;


    if(integral > LIMITE_INTEGRAL)
    {
        integral = LIMITE_INTEGRAL;
    }


    if(integral < -LIMITE_INTEGRAL)
    {
        integral = -LIMITE_INTEGRAL;
    }


    // --------------------------------------------------------
    // Derivada
    // --------------------------------------------------------

    float derivada =
        (erro - erroAnterior) / dt;


    erroAnterior = erro;


    // --------------------------------------------------------
    // PID
    // --------------------------------------------------------

    float saida =
        kp * erro
        +
        ki * integral
        +
        kd * derivada;


    // --------------------------------------------------------
    // Verifica tolerância
    // --------------------------------------------------------

    if(abs(erro) <= tolerancia)
    {

        // ----------------------------------------------------
        // Começa período de estabilização
        // ----------------------------------------------------

        if(inicioEstabilizacao == 0)
        {

            inicioEstabilizacao = agora;

        }


        // ----------------------------------------------------
        // Se permaneceu dentro da tolerância, termina
        // ----------------------------------------------------

        if(
            agora - inicioEstabilizacao
            >= tempoEstabilizacao
        )
        {

            finalizar();

            return;

        }

    }
    else
    {

        // ----------------------------------------------------
        // Saiu da tolerância
        // ----------------------------------------------------

        inicioEstabilizacao = 0;

    }


    // --------------------------------------------------------
    // Determina velocidade
    // --------------------------------------------------------

    float magnitude = abs(saida);


    if(magnitude > velocidadeMaxima)
    {
        magnitude = velocidadeMaxima;
    }


    // --------------------------------------------------------
    // Velocidade mínima para vencer atrito
    // --------------------------------------------------------

    if(
        magnitude > 0.0f &&
        magnitude < velocidadeMinima
    )
    {

        magnitude = velocidadeMinima;

    }


    int velocidade = (int)magnitude;


    // --------------------------------------------------------
    // Proteção
    // --------------------------------------------------------

    if(velocidade > velocidadeMaxima)
    {
        velocidade = velocidadeMaxima;
    }


    // --------------------------------------------------------
    // Comando diferencial
    //
    // erro positivo:
    //     direita
    //
    // erro negativo:
    //     esquerda
    // --------------------------------------------------------

    if(erro > 0.0f)
    {

        // ----------------------------------------------------
        // DIREITA
        // ----------------------------------------------------

        motores.setSpeed(
            velocidade,
            velocidade,
            -velocidade,
            -velocidade
        );

    }
    else if(erro < 0.0f)
    {

        // ----------------------------------------------------
        // ESQUERDA
        // ----------------------------------------------------

        motores.setSpeed(
            -velocidade,
            -velocidade,
            velocidade,
            velocidade
        );

    }
    else
    {

        motores.stop();

    }

}



// ============================================================
// FINALIZAR
// ============================================================

void ControleGiro::finalizar()
{

    // --------------------------------------------------------
    // Primeiro zera a referência de velocidade pelo PID
    // --------------------------------------------------------

    motores.stop();


    // --------------------------------------------------------
    // Pequeno tempo para o controlador processar o comando
    // --------------------------------------------------------

    delay(200);


    // --------------------------------------------------------
    // Depois libera completamente os motores
    // --------------------------------------------------------

    motores.release();


    // --------------------------------------------------------
    // Estado
    // --------------------------------------------------------

    emExecucao = false;
    movimentoConcluido = true;


    // --------------------------------------------------------
    // Reseta PID do controle de giro
    // --------------------------------------------------------

    erroAnterior = 0.0f;
    integral = 0.0f;

    inicioEstabilizacao = 0;

}



// ============================================================
// EXECUTANDO
// ============================================================

bool ControleGiro::executando()
{

    return emExecucao;

}



// ============================================================
// TERMINOU
// ============================================================

bool ControleGiro::terminou()
{

    return movimentoConcluido;

}



// ============================================================
// CANCELAR
// ============================================================

void ControleGiro::cancelar()
{

    if(!emExecucao)
    {
        return;
    }


    // --------------------------------------------------------
    // Para pelo PID
    // --------------------------------------------------------

    motores.stop();


    delay(200);


    // --------------------------------------------------------
    // Libera motores
    // --------------------------------------------------------

    motores.release();


    // --------------------------------------------------------
    // Estado
    // --------------------------------------------------------

    emExecucao = false;
    movimentoConcluido = false;


    erroAnterior = 0.0f;
    integral = 0.0f;

    inicioEstabilizacao = 0;

}



// ============================================================
// SET KP
// ============================================================

void ControleGiro::setKp(float valor)
{

    kp = valor;

}



// ============================================================
// SET KI
// ============================================================

void ControleGiro::setKi(float valor)
{

    ki = valor;

}



// ============================================================
// SET KD
// ============================================================

void ControleGiro::setKd(float valor)
{

    kd = valor;

}



// ============================================================
// SET VELOCIDADE MAXIMA
// ============================================================

void ControleGiro::setVelocidadeMaxima(int valor)
{

    if(valor < 1)
    {
        return;
    }


    velocidadeMaxima = valor;


    if(velocidadeMinima > velocidadeMaxima)
    {
        velocidadeMinima = velocidadeMaxima;
    }

}



// ============================================================
// SET VELOCIDADE MINIMA
// ============================================================

void ControleGiro::setVelocidadeMinima(int valor)
{

    if(valor < 0)
    {
        return;
    }


    velocidadeMinima = valor;


    if(velocidadeMinima > velocidadeMaxima)
    {
        velocidadeMinima = velocidadeMaxima;
    }

}



// ============================================================
// SET TOLERÂNCIA
// ============================================================

void ControleGiro::setTolerancia(float valor)
{

    if(valor <= 0.0f)
    {
        return;
    }


    tolerancia = valor;

}



// ============================================================
// SET TEMPO DE ESTABILIZAÇÃO
// ============================================================

void ControleGiro::setTempoEstabilizacao(
    unsigned long tempo
)
{

    tempoEstabilizacao = tempo;

}