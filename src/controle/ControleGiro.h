#ifndef CONTROLE_GIRO_H
#define CONTROLE_GIRO_H

#include <Arduino.h>

#include "sensores/IMU.h"
#include "hardware/MotorControlador.h"

class ControleGiro
{
public:

    ControleGiro(
        IMU& imu,
        MotorControlador& motores
    );

    // ---------------------------------------------------------
    // INICIALIZAÇÃO
    // ---------------------------------------------------------

    void begin();

    // ---------------------------------------------------------
    // ATUALIZAÇÃO
    // ---------------------------------------------------------

    // Deve ser chamado continuamente no loop.
    void update();

    // ---------------------------------------------------------
    // CURVAS
    // ---------------------------------------------------------

    void curva90Direita();

    void curva90Esquerda();

    void curva180();

    // Gira um determinado ângulo.
    //
    // positivo  = direita
    // negativo  = esquerda
    //
    void girar(float angulo);

    // ---------------------------------------------------------
    // ESTADO
    // ---------------------------------------------------------

    bool executando() const;

    bool terminou() const;

    void cancelar();

    // ---------------------------------------------------------
    // CONFIGURAÇÃO
    // ---------------------------------------------------------

    void setVelocidade(int velocidade);

    void setVelocidadeMinima(int velocidade);

    void setKp(float kp);

    void setKi(float ki);

    void setKd(float kd);

private:

    IMU& sensorIMU;

    MotorControlador& controladorMotores;

    // ---------------------------------------------------------
    // ESTADO DA CURVA
    // ---------------------------------------------------------

    bool emCurva;

    bool curvaTerminada;

    float anguloInicial;

    float anguloAlvo;

    float erroAnterior;

    float integralErro;

    uint32_t ultimoTempo;

    // ---------------------------------------------------------
    // CONFIGURAÇÃO
    // ---------------------------------------------------------

    int velocidadeMaxima;

    int velocidadeMinima;

    // PID da curva
    float kp;
    float ki;
    float kd;

    // Tolerância para considerar que chegou ao alvo.
    float tolerancia;

    // Tempo mínimo dentro da tolerância antes de terminar.
    uint32_t tempoEstabilizacao;

    uint32_t entrouNaTolerancia;

    // ---------------------------------------------------------
    // FUNÇÕES INTERNAS
    // ---------------------------------------------------------

    float calcularErro();

    void controlarMotores(float correcao);

    void pararMotores();

    bool chegouAoAlvo();

    float limitar(float valor, float minimo, float maximo);
};

#endif