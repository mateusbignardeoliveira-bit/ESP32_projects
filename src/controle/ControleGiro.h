#ifndef CONTROLE_GIRO_H
#define CONTROLE_GIRO_H

#include <Arduino.h>

#include "../Hardware/MotorControlador.h"
#include "../sensores/IMU.h"


class ControleGiro
{

private:

    // ========================================================
    // Referências
    // ========================================================

    IMU& sensorIMU;
    MotorControlador& motores;


    // ========================================================
    // Estado
    // ========================================================

    bool emExecucao;
    bool movimentoConcluido;


    // ========================================================
    // Giro atual
    // ========================================================

    float headingInicial;
    float headingAlvo;
    float anguloSolicitado;


    // ========================================================
    // PID do giro
    // ========================================================

    float kp;
    float ki;
    float kd;

    float erroAnterior;
    float integral;

    unsigned long ultimoTempo;


    // ========================================================
    // Limites de motor
    // ========================================================

    int velocidadeMaxima;
    int velocidadeMinima;


    // ========================================================
    // Finalização
    // ========================================================

    float tolerancia;
    unsigned long tempoEstabilizacao;
    unsigned long inicioEstabilizacao;


    // ========================================================
    // Calcula erro angular assinado
    //
    // positivo = necessário heading positivo
    // negativo = necessário heading negativo
    // ========================================================

    float calcularErroAngular(
        float atual,
        float alvo
    );


    // ========================================================
    // Finaliza movimento
    // ========================================================

    void finalizar();


public:

    ControleGiro(
        IMU& imu,
        MotorControlador& motorControlador
    );


    void begin();

    void update();


    void curva90Direita();

    void curva90Esquerda();

    void curva180();

    void girar(float angulo);


    bool executando();

    bool terminou();


    void cancelar();


    void setKp(float valor);

    void setKi(float valor);

    void setKd(float valor);

    void setVelocidadeMaxima(int valor);

    void setVelocidadeMinima(int valor);

    void setTolerancia(float valor);

    void setTempoEstabilizacao(
        unsigned long tempo
    );

};

#endif