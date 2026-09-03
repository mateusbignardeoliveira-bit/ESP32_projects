#include "ControleGiro.h"

#include <Arduino.h>
#include <math.h>


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
    emExecucao = false;
    movimentoConcluido = false;

    headingInicial = 0.0f;
    headingAlvo = 0.0f;
    anguloSolicitado = 0.0f;

    kp = 1.8f;
    ki = 0.0f;
    kd = 0.10f;

    erroAnterior = 0.0f;
    integral = 0.0f;

    ultimoTempo = 0;

    velocidadeMaxima = 100;
    velocidadeMinima = 30;

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

    inicioEstabilizacao = 0;

    ultimoTempo = millis();
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

    while(erro > 180.0f)
        erro -= 360.0f;

    while(erro < -180.0f)
        erro += 360.0f;

    return erro;
}


// ============================================================
// GIRAR
// ============================================================

void ControleGiro::girar(float angulo)
{
    if(emExecucao)
    {
        cancelar();
    }


    // Atualiza IMU antes de capturar o heading inicial.

    sensorIMU.update();

    headingInicial = sensorIMU.getHeading();


    // Guarda comando.

    anguloSolicitado = angulo;


    // Calcula alvo.

    headingAlvo = headingInicial + anguloSolicitado;

    while(headingAlvo >= 360.0f)
        headingAlvo -= 360.0f;

    while(headingAlvo < 0.0f)
        headingAlvo += 360.0f;


    // Reseta controle.

    erroAnterior = 0.0f;
    integral = 0.0f;

    inicioEstabilizacao = 0;

    ultimoTempo = millis();


    // Estado.

    movimentoConcluido = false;
    emExecucao = true;


    // Debug.

    Serial.print("Giro iniciado | Inicial: ");
    Serial.print(headingInicial, 2);

    Serial.print(" | Alvo: ");
    Serial.println(headingAlvo, 2);
}


// ============================================================
// CURVA 90 DIREITA
// ============================================================

void ControleGiro::curva90Direita()
{
    // Heading positivo = direita.
    girar(90.0f);
}


// ============================================================
// CURVA 90 ESQUERDA
// ============================================================

void ControleGiro::curva90Esquerda()
{
    // Heading negativo = esquerda.
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
        return;


    // --------------------------------------------------------
    // Atualiza IMU
    // --------------------------------------------------------

    if(!sensorIMU.update())
        return;


    float headingAtual = sensorIMU.getHeading();


    // --------------------------------------------------------
    // Calcula erro
    // --------------------------------------------------------

    float erro = calcularErroAngular(
        headingAtual,
        headingAlvo
    );


    // --------------------------------------------------------
    // DEBUG
    // --------------------------------------------------------

    Serial.print("Atual: ");
    Serial.print(headingAtual, 2);

    Serial.print(" | Alvo: ");
    Serial.print(headingAlvo, 2);

    Serial.print(" | Erro: ");
    Serial.println(erro, 2);


    // --------------------------------------------------------
    // CHEGOU NO ALVO
    // --------------------------------------------------------

    if(fabsf(erro) <= tolerancia)
    {
        finalizar();
        return;
    }


    // --------------------------------------------------------
    // TEMPO
    // --------------------------------------------------------

    unsigned long agora = millis();

    float dt =
        (agora - ultimoTempo) / 1000.0f;

    ultimoTempo = agora;

    if(dt <= 0.0f)
        dt = 0.001f;

    if(dt > 0.1f)
        dt = 0.1f;


    // --------------------------------------------------------
    // INTEGRAL
    // --------------------------------------------------------

    integral += erro * dt;

    const float LIMITE_INTEGRAL = 100.0f;

    if(integral > LIMITE_INTEGRAL)
        integral = LIMITE_INTEGRAL;

    if(integral < -LIMITE_INTEGRAL)
        integral = -LIMITE_INTEGRAL;


    // --------------------------------------------------------
    // DERIVADA
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
    // MAGNITUDE
    // --------------------------------------------------------

    float magnitude = fabsf(saida);

    if(magnitude > velocidadeMaxima)
        magnitude = velocidadeMaxima;


    if(
        magnitude > 0.0f &&
        magnitude < velocidadeMinima
    )
    {
        magnitude = velocidadeMinima;
    }


    int velocidade = (int)magnitude;


    if(velocidade > velocidadeMaxima)
        velocidade = velocidadeMaxima;


    // --------------------------------------------------------
    // COMANDO DE GIRO
    //
    // IMPORTANTE:
    //
    // O MotorControlador da montagem atual está com o sentido
    // dos motores invertido para compensar a direção física.
    //
    // Portanto o diferencial abaixo também precisa ser
    // invertido em relação ao sinal lógico do heading.
    //
    // erro positivo:
    //     heading precisa aumentar
    //     -> giro físico para direita
    //
    // erro negativo:
    //     heading precisa diminuir
    //     -> giro físico para esquerda
    // --------------------------------------------------------

    if(erro > 0.0f)
    {
        // DIREITA FÍSICA

        motores.setSpeed(
            -velocidade,
            -velocidade,
            velocidade,
            velocidade
        );
    }
    else
    {
        // ESQUERDA FÍSICA

        motores.setSpeed(
            velocidade,
            velocidade,
            -velocidade,
            -velocidade
        );
    }
}


// ============================================================
// FINALIZAR
// ============================================================

void ControleGiro::finalizar()
{
    Serial.println("GIRO TERMINADO.");

    motores.stop();

    delay(100);

    motores.release();


    emExecucao = false;
    movimentoConcluido = true;

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
        return;


    motores.stop();

    delay(100);

    motores.release();


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
        return;

    velocidadeMaxima = valor;

    if(velocidadeMinima > velocidadeMaxima)
        velocidadeMinima = velocidadeMaxima;
}


// ============================================================
// SET VELOCIDADE MINIMA
// ============================================================

void ControleGiro::setVelocidadeMinima(int valor)
{
    if(valor < 0)
        return;

    velocidadeMinima = valor;

    if(velocidadeMinima > velocidadeMaxima)
        velocidadeMinima = velocidadeMaxima;
}


// ============================================================
// SET TOLERÂNCIA
// ============================================================

void ControleGiro::setTolerancia(float valor)
{
    if(valor <= 0.0f)
        return;

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