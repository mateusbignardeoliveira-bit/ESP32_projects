#include "ControleGiro.h"

#include <Arduino.h>
#include <math.h>

// =============================================================
// CONSTRUTOR
// =============================================================

ControleGiro::ControleGiro(
    IMU& imu,
    MotorControlador& motores
)
    : sensorIMU(imu),
      controladorMotores(motores),

      emCurva(false),
      curvaTerminada(false),

      anguloAlvo(0.0f),
      erroAnterior(0.0f),

      ultimoTempo(0),
      entrouNaTolerancia(0),

      velocidadeMaxima(100),
      velocidadeMinima(30),

      kp(1.8f),
      kd(0.10f),

      tolerancia(2.0f),
      tempoEstabilizacao(60)
{
}

// =============================================================
// BEGIN
// =============================================================

void ControleGiro::begin()
{
    emCurva = false;
    curvaTerminada = false;

    anguloAlvo = 0.0f;

    erroAnterior = 0.0f;

    entrouNaTolerancia = 0;

    ultimoTempo = micros();

    pararMotores();
}

// =============================================================
// UPDATE
// =============================================================

void ControleGiro::update()
{
    if (!emCurva)
        return;

    // ---------------------------------------------------------
    // Atualiza IMU
    // ---------------------------------------------------------

    if (!sensorIMU.update())
    {
        pararMotores();
        return;
    }

    // ---------------------------------------------------------
    // DT
    // ---------------------------------------------------------

    uint32_t agora = micros();

    float dt =
        (agora - ultimoTempo) /
        1000000.0f;

    ultimoTempo = agora;

    if (
        dt <= 0.0f ||
        dt > 0.1f
    )
    {
        dt = 0.01f;
    }

    // ---------------------------------------------------------
    // ERRO
    // ---------------------------------------------------------

    float erro =
        calcularErro();

    // ---------------------------------------------------------
    // CHEGOU?
    // ---------------------------------------------------------

    if (chegouAoAlvo())
    {
        pararMotores();

        emCurva = false;
        curvaTerminada = true;

        erroAnterior = 0.0f;

        return;
    }

    // ---------------------------------------------------------
    // DERIVATIVO
    // ---------------------------------------------------------

    float derivada =
        (erro - erroAnterior) /
        dt;

    erroAnterior = erro;

    // ---------------------------------------------------------
    // CONTROLE
    // ---------------------------------------------------------

    float correcao =
        kp * erro +
        kd * derivada;

    correcao =
        limitar(
            correcao,
            -velocidadeMaxima,
            velocidadeMaxima
        );

    controlarMotores(correcao);
}

// =============================================================
// 90 DIREITA
// =============================================================

void ControleGiro::curva90Direita()
{
    girar(90.0f);
}

// =============================================================
// 90 ESQUERDA
// =============================================================

void ControleGiro::curva90Esquerda()
{
    girar(-90.0f);
}

// =============================================================
// 180
// =============================================================

void ControleGiro::curva180()
{
    girar(180.0f);
}

// =============================================================
// GIRAR
// =============================================================

void ControleGiro::girar(float angulo)
{
    if (emCurva)
        return;

    if (fabsf(angulo) < 1.0f)
        return;

    float atual =
        sensorIMU.getHeading();

    anguloAlvo =
        atual + angulo;

    while (anguloAlvo >= 360.0f)
        anguloAlvo -= 360.0f;

    while (anguloAlvo < 0.0f)
        anguloAlvo += 360.0f;

    erroAnterior = 0.0f;

    entrouNaTolerancia = 0;

    ultimoTempo = micros();

    curvaTerminada = false;

    emCurva = true;
}

// =============================================================
// ERRO
// =============================================================

float ControleGiro::calcularErro() const
{
    return IMU::diferencaAngular(
        sensorIMU.getHeading(),
        anguloAlvo
    );
}

// =============================================================
// CONTROLE DOS MOTORES
// =============================================================

void ControleGiro::controlarMotores(
    float correcao
)
{
    /*
     * Convenção:
     *
     * M1/M2 = esquerda
     * M3/M4 = direita
     *
     * correcao positiva:
     *     gira para direita
     *
     * correcao negativa:
     *     gira para esquerda
     */

    float magnitude =
        fabsf(correcao);

    int velocidade =
        velocidadeMinima +
        (int)magnitude;

    velocidade =
        constrain(
            velocidade,
            velocidadeMinima,
            velocidadeMaxima
        );

    if (correcao > 0.0f)
    {
        // -----------------------------------------------------
        // DIREITA
        // -----------------------------------------------------

        controladorMotores.setSpeed(
            velocidade,
            velocidade,
            -velocidade,
            -velocidade
        );
    }
    else
    {
        // -----------------------------------------------------
        // ESQUERDA
        // -----------------------------------------------------

        controladorMotores.setSpeed(
            -velocidade,
            -velocidade,
            velocidade,
            velocidade
        );
    }
}

// =============================================================
// PARAR
// =============================================================

void ControleGiro::pararMotores()
{
    controladorMotores.setSpeed(
        0,
        0,
        0,
        0
    );
}

// =============================================================
// CHEGOU AO ALVO
// =============================================================

bool ControleGiro::chegouAoAlvo()
{
    float erro =
        fabsf(
            calcularErro()
        );

    if (erro <= tolerancia)
    {
        if (entrouNaTolerancia == 0)
        {
            entrouNaTolerancia =
                millis();
        }

        if (
            millis() -
            entrouNaTolerancia >=
            tempoEstabilizacao
        )
        {
            return true;
        }
    }
    else
    {
        entrouNaTolerancia = 0;
    }

    return false;
}

// =============================================================
// ESTADO
// =============================================================

bool ControleGiro::executando() const
{
    return emCurva;
}

bool ControleGiro::terminou() const
{
    return curvaTerminada;
}

// =============================================================
// CANCELAR
// =============================================================

void ControleGiro::cancelar()
{
    pararMotores();

    emCurva = false;
    curvaTerminada = false;

    anguloAlvo = 0.0f;

    erroAnterior = 0.0f;

    entrouNaTolerancia = 0;
}

// =============================================================
// CONFIGURAÇÕES
// =============================================================

void ControleGiro::setVelocidade(
    int velocidade
)
{
    velocidadeMaxima =
        abs(velocidade);

    if (
        velocidadeMinima >
        velocidadeMaxima
    )
    {
        velocidadeMinima =
            velocidadeMaxima;
    }
}

void ControleGiro::setVelocidadeMinima(
    int velocidade
)
{
    velocidadeMinima =
        abs(velocidade);

    if (
        velocidadeMinima >
        velocidadeMaxima
    )
    {
        velocidadeMinima =
            velocidadeMaxima;
    }
}

void ControleGiro::setKp(
    float valor
)
{
    kp = valor;
}

void ControleGiro::setKd(
    float valor
)
{
    kd = valor;
}

void ControleGiro::setTolerancia(
    float graus
)
{
    if (graus < 0.1f)
        graus = 0.1f;

    tolerancia = graus;
}

// =============================================================
// LIMITADOR
// =============================================================

float ControleGiro::limitar(
    float valor,
    float minimo,
    float maximo
) const
{
    if (valor < minimo)
        return minimo;

    if (valor > maximo)
        return maximo;

    return valor;
}