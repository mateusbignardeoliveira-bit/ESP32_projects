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

    float Kp;
    float Ki;
    float Kd;


    float erroAnterior;

    float integral;


    float limiteIntegral;

    float limiteCorrecao;


    PIDData resultado;


public:

    PIDLinha(
        float kp = 80.0f,
        float ki = 0.0f,
        float kd = 30.0f
    );


    void reset();


    void update(
        float erro,
        float deltaTime
    );


    PIDData getData();


    void setGanho(
        float kp,
        float ki,
        float kd
    );


    void setLimiteCorrecao(
        float limite
    );

};

#endif