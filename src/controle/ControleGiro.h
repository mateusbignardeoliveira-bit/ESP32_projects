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

    void begin();

    // Deve ser chamado continuamente.
    void update();

    // ---------------------------------------------------------
    // MANOBRAS
    // ---------------------------------------------------------

    void curva90Direita();

    void curva90Esquerda();

    void curva180();

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

    void setKd(float kd);

    void setTolerancia(float graus);

private:

    IMU& sensorIMU;

    MotorControlador& controladorMotores;

    bool emCurva;

    bool curvaTerminada;

    float anguloAlvo;

    float erroAnterior;

    uint32_t ultimoTempo;

    uint32_t entrouNaTolerancia;

    int velocidadeMaxima;

    int velocidadeMinima;

    float kp;

    float kd;

    float tolerancia;

    uint32_t tempoEstabilizacao;

    float calcularErro() const;

    void controlarMotores(
        float erro
    );

    void pararMotores();

    bool chegouAoAlvo();

    float limitar(
        float valor,
        float minimo,
        float maximo
    ) const;
};

#endif