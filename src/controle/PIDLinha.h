#ifndef PID_LINHA_H
#define PID_LINHA_H

#include <Arduino.h>


struct PIDData
{
    float erro;

    float proporcional;

    float derivativo;

    float integral;

    float correcao;
};


class PIDLinha
{

private:

    // ========================================================
    // GANHOS
    // ========================================================

    float Kp;
    float Ki;
    float Kd;


    // ========================================================
    // MEMÓRIA DO PID
    // ========================================================

    float erroAnterior;

    float integral;

    float derivadaFiltrada;


    // ========================================================
    // LIMITES
    // ========================================================

    float limiteIntegral;

    float limiteCorrecao;


    // ========================================================
    // FILTRO DA DERIVADA
    //
    // 0.0 = máxima filtragem
    // 1.0 = sem filtragem
    // ========================================================

    float fatorFiltroDerivativo;


    // ========================================================
    // RESULTADO
    // ========================================================

    PIDData resultado;


public:

    // ========================================================
    // CONSTRUTOR
    // ========================================================

    PIDLinha(
        float kp = 80.0f,
        float ki = 0.0f,
        float kd = 30.0f
    );


    // ========================================================
    // RESET
    // ========================================================

    void reset();


    // ========================================================
    // ATUALIZA PID
    //
    // erro:
    //     posição da linha em relação ao centro.
    //
    // deltaTime:
    //     tempo desde a última atualização em segundos.
    // ========================================================

    void update(
        float erro,
        float deltaTime
    );


    // ========================================================
    // DADOS DO PID
    // ========================================================

    PIDData getData();


    // ========================================================
    // GANHOS
    // ========================================================

    void setGanho(
        float kp,
        float ki,
        float kd
    );


    // ========================================================
    // LIMITE DA CORREÇÃO
    // ========================================================

    void setLimiteCorrecao(
        float limite
    );


    // ========================================================
    // LIMITE DA INTEGRAL
    // ========================================================

    void setLimiteIntegral(
        float limite
    );


    // ========================================================
    // FILTRO DA DERIVADA
    // ========================================================

    void setFiltroDerivativo(
        float fator
    );

};

#endif