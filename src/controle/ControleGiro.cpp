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

      anguloInicial(0.0f),
      anguloAlvo(0.0f),

      erroAnterior(0.0f),
      integralErro(0.0f),

      ultimoTempo(0),

      velocidadeMaxima(100),
      velocidadeMinima(35),

      kp(2.0f),
      ki(0.0f),
      kd(0.15f),

      tolerancia(2.0f),

      tempoEstabilizacao(50),
      entrouNaTolerancia(0)
{
}

// =============================================================
// BEGIN
// =============================================================

void ControleGiro::begin()
{
    emCurva = false;
    curvaTerminada = false;

    erroAnterior = 0.0f;
    integralErro = 0.0f;

    ultimoTempo = millis();

    pararMotores();
}

// =============================================================
// UPDATE
// =============================================================

void ControleGiro::update()
{
    if (!emCurva) {
        return;
    }

    // ---------------------------------------------------------
    // ATUALIZAÇÃO DA IMU
    // ---------------------------------------------------------

    if (!sensorIMU.update()) {
        return;
    }

    // ---------------------------------------------------------
    // TEMPO
    // ---------------------------------------------------------

    uint32_t agora = millis();

    float dt =
        (agora - ultimoTempo) / 1000.0f;

    ultimoTempo = agora;

    if (dt <= 0.0f || dt > 0.2f) {
        dt = 0.01f;
    }

    // ---------------------------------------------------------
    // ERRO ANGULAR
    // ---------------------------------------------------------

    float erro = calcularErro();

    // ---------------------------------------------------------
    // VERIFICA CHEGADA
    // ---------------------------------------------------------

    if (chegouAoAlvo())
    {
        pararMotores();

        emCurva = false;
        curvaTerminada = true;

        erroAnterior = 0.0f;
        integralErro = 0.0f;

        return;
    }

    // ---------------------------------------------------------
    // PID
    // ---------------------------------------------------------

    integralErro += erro * dt;

    // Anti-windup
    integralErro =
        limitar(
            integralErro,
            -100.0f,
            100.0f
        );

    float derivada =
        (erro - erroAnterior) / dt;

    erroAnterior = erro;

    float correcao =
        kp * erro +
        ki * integralErro +
        kd * derivada;

    // ---------------------------------------------------------
    // LIMITA CORREÇÃO
    // ---------------------------------------------------------

    correcao =
        limitar(
            correcao,
            -velocidadeMaxima,
            velocidadeMaxima
        );

    // ---------------------------------------------------------
    // CONTROLA MOTORES
    // ---------------------------------------------------------

    controlarMotores(correcao);
}

// =============================================================
// CURVA 90 DIREITA
// =============================================================

void ControleGiro::curva90Direita()
{
    girar(90.0f);
}

// =============================================================
// CURVA 90 ESQUERDA
// =============================================================

void ControleGiro::curva90Esquerda()
{
    girar(-90.0f);
}

// =============================================================
// CURVA 180
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
    if (emCurva) {
        return;
    }

    // ---------------------------------------------------------
    // LEITURA DO ÂNGULO ATUAL
    // ---------------------------------------------------------

    float atual =
        sensorIMU.getHeading();

    anguloInicial = atual;

    // ---------------------------------------------------------
    // CALCULA ALVO
    // ---------------------------------------------------------

    anguloAlvo =
        atual + angulo;

    // Normaliza
    while (anguloAlvo >= 360.0f) {
        anguloAlvo -= 360.0f;
    }

    while (anguloAlvo < 0.0f) {
        anguloAlvo += 360.0f;
    }

    // ---------------------------------------------------------
    // RESETA PID
    // ---------------------------------------------------------

    erroAnterior = 0.0f;

    integralErro = 0.0f;

    entrouNaTolerancia = 0;

    ultimoTempo = millis();

    // ---------------------------------------------------------
    // ATIVA CURVA
    // ---------------------------------------------------------

    emCurva = true;

    curvaTerminada = false;
}

// =============================================================
// CALCULAR ERRO
// =============================================================

float ControleGiro::calcularErro()
{
    float atual =
        sensorIMU.getHeading();

    return IMU::diferencaAngular(
        atual,
        anguloAlvo
    );
}

// =============================================================
// CONTROLAR MOTORES
// =============================================================

void ControleGiro::controlarMotores(
    float correcao
)
{
    /*
     * Convenção:
     *
     * motor esquerdo  = m1/m2
     * motor direito   = m3/m4
     *
     * Para girar para a direita:
     *
     * esquerda  +
     * direita   -
     *
     * Para girar para a esquerda:
     *
     * esquerda  -
     * direita   +
     */

    int velocidadeEsquerda;
    int velocidadeDireita;

    if (correcao > 0.0f)
    {
        // -----------------------------------------------------
        // DIREITA
        // -----------------------------------------------------

        velocidadeEsquerda =
            velocidadeMinima +
            (int)fabsf(correcao);

        velocidadeDireita =
            -(
                velocidadeMinima +
                (int)fabsf(correcao)
            );
    }
    else
    {
        // -----------------------------------------------------
        // ESQUERDA
        // -----------------------------------------------------

        velocidadeEsquerda =
            -(
                velocidadeMinima +
                (int)fabsf(correcao)
            );

        velocidadeDireita =
            velocidadeMinima +
            (int)fabsf(correcao);
    }

    // ---------------------------------------------------------
    // LIMITA VELOCIDADES
    // ---------------------------------------------------------

    velocidadeEsquerda =
        (int)limitar(
            velocidadeEsquerda,
            -velocidadeMaxima,
            velocidadeMaxima
        );

    velocidadeDireita =
        (int)limitar(
            velocidadeDireita,
            -velocidadeMaxima,
            velocidadeMaxima
        );

    // ---------------------------------------------------------
    // ENVIA AOS QUATRO MOTORES
    // ---------------------------------------------------------

    controladorMotores.setSpeed(
        velocidadeEsquerda,
        velocidadeEsquerda,
        velocidadeDireita,
        velocidadeDireita
    );
}

// =============================================================
// PARAR MOTORES
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
        fabsf(calcularErro());

    if (erro <= tolerancia)
    {
        if (entrouNaTolerancia == 0)
        {
            entrouNaTolerancia = millis();
        }

        if (
            millis() - entrouNaTolerancia
            >= tempoEstabilizacao
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

    erroAnterior = 0.0f;

    integralErro = 0.0f;

    entrouNaTolerancia = 0;
}

// =============================================================
// CONFIGURAÇÃO
// =============================================================

void ControleGiro::setVelocidade(int velocidade)
{
    velocidadeMaxima =
        abs(velocidade);
}

void ControleGiro::setVelocidadeMinima(int velocidade)
{
    velocidadeMinima =
        abs(velocidade);
}

void ControleGiro::setKp(float valor)
{
    kp = valor;
}

void ControleGiro::setKi(float valor)
{
    ki = valor;
}

void ControleGiro::setKd(float valor)
{
    kd = valor;
}

// =============================================================
// LIMITADOR
// =============================================================

float ControleGiro::limitar(
    float valor,
    float minimo,
    float maximo
)
{
    if (valor < minimo) {
        return minimo;
    }

    if (valor > maximo) {
        return maximo;
    }

    return valor;
}
